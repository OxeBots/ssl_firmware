#include "WheelController.h"

#include <esp_log.h>
#include <esp_timer.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <type_traits>

#include "BL48250.h"

static const char * TAG = "WheelController";

// ============================================================================
// SIGN HELPER
// ============================================================================

static inline int signf(float x)
{
    return (x > 0.0f) - (x < 0.0f);
}

// Clamps one parameter set to the ranges accepted by the controller and
// normalizes values below the FastPID fixed-point resolution.
static WheelController::Parameters clamp_parameters(const WheelController::Parameters & input)
{
    WheelController::Parameters clamped = input;
    clamped.kp = std::clamp(clamped.kp, 0.0f, 200.0f);
    // ki is the controller-managed integral gain (duty per rad/s per second),
    // not a FastPID coefficient: it is bounded by the calibration range
    // (WheelController::KI_MAX_CAL).
    clamped.ki = std::clamp(clamped.ki, 0.0f, 80.0f);
    clamped.kd = std::clamp(clamped.kd, 0.0f, 0.5f);
    clamped.feedforward = std::clamp(clamped.feedforward, -600.0f, 600.0f);
    clamped.velocity_ff = std::clamp(clamped.velocity_ff, 0.0f, 400.0f);
    clamped.static_friction = std::clamp(clamped.static_friction, 0.0f, 600.0f);
    clamped.feedforward_rev = std::clamp(clamped.feedforward_rev, -600.0f, 600.0f);
    clamped.velocity_ff_rev = std::clamp(clamped.velocity_ff_rev, 0.0f, 400.0f);
    clamped.static_friction_rev = std::clamp(clamped.static_friction_rev, 0.0f, 600.0f);
    clamped.angle_sign = (clamped.angle_sign < 0.0f) ? -1.0f : 1.0f;
    for (float & speed : clamped.continuous_min_rad_s)
        speed =
          std::isfinite(speed) ? std::clamp(speed, 0.0f, config::driver::WHEEL_MAX_VEL_RAD) : 0.0f;
    for (float & duty : clamped.pulse_duty)
        duty = std::isfinite(duty) ? std::clamp(duty, 0.0f, WheelController::MAX_PULSE_DUTY) : 0.0f;
    for (float & duty : clamped.pulse_off_duty)
        duty = std::isfinite(duty) ? std::clamp(duty, 0.0f, WheelController::MAX_PULSE_DUTY) : 0.0f;
    for (float & duty : clamped.pulse_floor_duty)
        duty = std::isfinite(duty) ? std::clamp(duty, 0.0f, WheelController::MAX_PULSE_DUTY) : 0.0f;
    for (float & response_s : clamped.pulse_response_s)
        if (!std::isfinite(response_s) || response_s < 0.0f)
            response_s = 0.0f;
    for (float & delay_s : clamped.pulse_coast_delay_s)
        if (!std::isfinite(delay_s) || delay_s < 0.0f)
            delay_s = 0.0f;
    // FastPID rejects a coefficient that rounds to zero and then zeroes every
    // gain, so drop values below its fixed-point resolution (ki is not a
    // FastPID coefficient anymore).
    if (clamped.kp > 0.0f && clamped.kp < 1.0f / 256.0f)
        clamped.kp = 0.0f;
    if (clamped.kd > 0.0f && clamped.kd < 1.0f / (100.0f * 256.0f))
        clamped.kd = 0.0f;
    return clamped;
}

// ============================================================================
// SINGLETON
// ============================================================================

WheelController & WheelController::get_instance()
{
    static WheelController instance;
    return instance;
}

// ============================================================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================================================

WheelController::WheelController()
: m_task_handle(nullptr),
  m_initialized(false),
  m_open_loop(false),
  m_low_speed_mode_enabled(true),
  m_target_velocity_rad_s({0, 0, 0, 0}),
  m_current_velocity_rad_s({0, 0, 0, 0}),
  m_pid_output_duty({0, 0, 0, 0}),
  m_max_velocity_rad_s(config::driver::WHEEL_MAX_VEL_RAD),
  m_position_angle_rad({0, 0, 0, 0}),
  m_last_raw_angle_rad({0, 0, 0, 0}),
  m_raw_angle_seeded({false, false, false, false}),
  m_position_angle_start_rad({0, 0, 0, 0}),
  m_position_target_rad_s({0, 0, 0, 0}),
  m_position_active_mask(0),
  m_position_done_mask(0),
  m_position_error(PositionError::NONE),
  m_position_start_us(0),
  m_position_max_velocity_rad_s(POSITION_DEFAULT_MAX_RAD_S),
  m_last_control_us(0),
  m_mutex(xSemaphoreCreateMutex())
{
    m_rip_min.fill(1e9f);
    m_rip_max.fill(-1e9f);
}

WheelController::~WheelController()
{
    deinit();
    if (m_mutex)
    {
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
    }
}

// ============================================================================
// INIT / DEINIT
// ============================================================================

esp_err_t WheelController::init()
{
    if (m_initialized)
        return ESP_OK;

    load_parameters_from_nvs();

    for (int i = 0; i < 4; ++i)
    {
        // FastPID: kp, ki, kd, update rate (Hz), 16-bit output, signed. The
        // integral term stays at zero: the controller manages it separately so
        // its sum cannot wind up inside the fixed-point PID.
        m_pids[i].configure(m_parameters[i].kp, 0.0f, m_parameters[i].kd, PID_HZ, 16, true);
        m_pids[i].set_output_range(-10230, 10230);  // ±1023 scaled ×10
    }

    gptimer_config_t timer_config = {};
    timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    timer_config.direction = GPTIMER_COUNT_UP;
    timer_config.resolution_hz = 1000000;
    esp_err_t err = gptimer_new_timer(&timer_config, &m_control_timer);
    if (err != ESP_OK)
        return err;

    gptimer_event_callbacks_t callbacks = {};
    callbacks.on_alarm = on_control_alarm;
    gptimer_alarm_config_t alarm = {};
    alarm.alarm_count = CONTROL_PERIOD_US;
    alarm.flags.auto_reload_on_alarm = true;
    err = gptimer_register_event_callbacks(m_control_timer, &callbacks, this);
    if (err == ESP_OK)
        err = gptimer_set_alarm_action(m_control_timer, &alarm);
    if (err == ESP_OK)
        err = gptimer_enable(m_control_timer);
    if (err != ESP_OK)
    {
        gptimer_del_timer(m_control_timer);
        m_control_timer = nullptr;
        return err;
    }
    if (xTaskCreatePinnedToCore(task_wrapper, "wheel_ctrl", 4096, this, 6, &m_task_handle, 1) !=
        pdPASS)
    {
        deinit();
        return ESP_ERR_NO_MEM;
    }
    err = gptimer_start(m_control_timer);
    if (err != ESP_OK)
    {
        deinit();
        return err;
    }
    m_initialized = true;

    for (int i = 0; i < 4; ++i)
    {
        ESP_LOGI(TAG,
                 "Motor %d: Kp=%.3f Ki=%.3f Kd=%.3f ff=%.1f/%.1f vff=%.3f/%.3f",
                 i,
                 static_cast<double>(m_parameters[i].kp),
                 static_cast<double>(m_parameters[i].ki),
                 static_cast<double>(m_parameters[i].kd),
                 static_cast<double>(m_parameters[i].feedforward),
                 static_cast<double>(m_parameters[i].feedforward_rev),
                 static_cast<double>(m_parameters[i].velocity_ff),
                 static_cast<double>(m_parameters[i].velocity_ff_rev));
    }
    ESP_LOGI(TAG,
             "Speed limit: %.1f RPM (%.2f rad/s).",
             static_cast<double>(m_max_velocity_rad_s * RAD_S_TO_RPM),
             static_cast<double>(m_max_velocity_rad_s));
    ESP_LOGI(TAG,
             "Fallback integral schedule: full Ki up to %.1f rad/s, zero from %.1f rad/s.",
             static_cast<double>(LOW_KI_FULL_RAD_S),
             static_cast<double>(LOW_KI_ZERO_RAD_S));
    return ESP_OK;
}

void WheelController::deinit()
{
    if (m_control_timer)
    {
        gptimer_stop(m_control_timer);
        gptimer_disable(m_control_timer);
        gptimer_del_timer(m_control_timer);
        m_control_timer = nullptr;
    }
    if (m_task_handle)
    {
        vTaskDelete(m_task_handle);
        m_task_handle = nullptr;
        ESP_LOGI(TAG, "Control task deleted.");
    }

    m_initialized = false;
    m_open_loop = false;
    m_position_active_mask = 0;
    m_position_done_mask = 0;

    m_target_velocity_rad_s.fill(0.0f);
    m_current_velocity_rad_s.fill(0.0f);
    m_pid_output_duty.fill(0.0f);
    m_integral_duty.fill(0.0f);
    m_kick_state.fill(KickState());
    m_last_target_sign.fill(0);
    for (int i = 0; i < 4; ++i) reset_low_speed_locked(i);
    m_ls_fault_ms.fill(0);
    m_ls_fault_sticky.fill(false);
    m_ls_blocked_warn_ms.fill(0);

    ESP_LOGI(TAG, "WheelController deinitialized.");
}

// ============================================================================
// PUBLIC API — Velocity targets
// ============================================================================

/**
 * @brief Sets the four wheel speed targets, cancelling any position move.
 */
void WheelController::set_target_velocities(const std::array<float, 4> & velocities)
{
    if (!m_mutex)
        return;
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    // A velocity target explicitly leaves position mode, like a stop command.
    m_position_active_mask = 0;
    m_position_done_mask = 0;
    m_position_error = PositionError::NONE;
    const float min_target_rad_s =
      std::min(MIN_SMOOTH_TARGET_RPM * RPM_TO_RAD_S, m_max_velocity_rad_s);
    for (int i = 0; i < 4; ++i)
    {
        float target = std::clamp(velocities[i], -m_max_velocity_rad_s, m_max_velocity_rad_s);
        // Below the smooth minimum RPM the pulse/chopper cycle becomes a slow
        // stop-and-go that looks nothing like the other wheels; clamp the
        // magnitude so all wheels start from the same, smooth operating point.
        if (target != 0.0f && std::fabs(target) < min_target_rad_s)
        {
            const float sign = (target > 0.0f) ? 1.0f : -1.0f;
            target = sign * min_target_rad_s;
        }
        m_target_velocity_rad_s[i] = target;
    }
    xSemaphoreGive(m_mutex);
}

std::array<float, 4> WheelController::get_target_velocities() const
{
    return m_target_velocity_rad_s;
}

uint8_t WheelController::get_low_speed_velocities_rad_s(std::array<float, 4> & velocities) const
{
    if (!m_mutex || xSemaphoreTake(m_mutex, 0) != pdTRUE)
        return 0;
    uint8_t mask = 0;
    for (size_t wheel = 0; wheel < m_low_speed.size(); ++wheel)
    {
        if (m_low_speed[wheel].active && m_last_target_sign[wheel] != 0)
        {
            velocities[wheel] = static_cast<float>(m_last_target_sign[wheel]) *
                                m_low_speed[wheel].last_window_rpm * RPM_TO_RAD_S;
            mask |= static_cast<uint8_t>(1U << wheel);
        }
    }
    xSemaphoreGive(m_mutex);
    return mask;
}

std::array<float, 4> WheelController::get_current_velocities() const
{
    return m_current_velocity_rad_s;
}

std::array<float, 4> WheelController::get_pid_outputs() const
{
    return m_pid_output_duty;
}

/**
 * @brief Publishes summaries from the serial task without formatting logs in
 * the motor task.
 */
void WheelController::log_diagnostics()
{
    std::array<RippleReport, 4> ripple;
    TimingReport timing;
    portENTER_CRITICAL(&m_diagnostic_mux);
    ripple = m_ripple_reports;
    timing = m_timing_report;
    m_ripple_reports = {};
    m_timing_report = {};
    portEXIT_CRITICAL(&m_diagnostic_mux);

    if (timing.steps != 0)
    {
        ESP_LOGI(TAG,
                 "CTL n=%lu miss=%lu gap=%lu age=%lu work=%lu us",
                 static_cast<unsigned long>(timing.steps),
                 static_cast<unsigned long>(timing.missed),
                 static_cast<unsigned long>(timing.max_gap_us),
                 static_cast<unsigned long>(timing.max_age_us),
                 static_cast<unsigned long>(timing.max_work_us));
    }
    for (size_t i = 0; i < ripple.size(); ++i)
    {
        const RippleReport & report = ripple[i];
        if (report.window_s == 0.0f)
            continue;
        ESP_LOGI(TAG,
                 "LS M%u t%.1f m%.1f on%.1f ff%.1f e%.1f i%.1f b%.0f",
                 static_cast<unsigned>(i + 1),
                 static_cast<double>(report.target_rpm),
                 static_cast<double>(report.mean_rpm),
                 static_cast<double>(report.on_ticks),
                 static_cast<double>(report.feedforward_ticks),
                 static_cast<double>(report.error_rpm),
                 static_cast<double>(report.integral_ticks),
                 static_cast<double>(report.boost_duty));
        ESP_LOGI(TAG,
                 "RIP M%u t%.1f m%.2f lo%.2f hi%.2f d%.4f w%.4f",
                 static_cast<unsigned>(i + 1),
                 static_cast<double>(report.target_rpm),
                 static_cast<double>(report.mean_rpm),
                 static_cast<double>(report.min_rpm),
                 static_cast<double>(report.max_rpm),
                 static_cast<double>(report.angle_delta),
                 static_cast<double>(report.window_s));
    }
}

float WheelController::get_max_velocity_rad_s() const
{
    return m_max_velocity_rad_s;
}

/**
 * @brief Replaces the common calibrated speed limit.
 *
 * The value stays in RAM: the calibration saves it explicitly once it is
 * validated. A non-positive value is rejected and the upper bound is the
 * drivetrain maximum.
 */
void WheelController::set_max_velocity_rad_s(float limit)
{
    if (!m_mutex || !(limit > 0.0f))
        return;
    const float clamped = std::min(limit, config::driver::WHEEL_MAX_VEL_RAD);
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    m_max_velocity_rad_s = clamped;
    xSemaphoreGive(m_mutex);
}

// ============================================================================
// PUBLIC API — Open-loop mode
// ============================================================================

/**
 * @brief Enters or leaves open-loop mode.
 *
 * In open loop the control task stops writing the motor outputs, so the caller
 * owns the four motors (used by the calibration routines). Entering also
 * zeroes the targets, cancels position moves and stops the motors; leaving
 * clears the PID state and restarts the control period with zero targets.
 */
void WheelController::set_open_loop(bool enabled)
{
    if (!m_mutex)
        return;

    xSemaphoreTake(m_mutex, portMAX_DELAY);
    m_open_loop = enabled;
    if (enabled)
    {
        m_target_velocity_rad_s.fill(0.0f);
        cancel_position_moves_locked();
        m_position_error = PositionError::NONE;
    }
    for (auto & pid : m_pids) pid.clear();
    for (int i = 0; i < 4; ++i) reset_control_state_locked(i);
    // The caller may have turned the wheels in open loop: do not accumulate a
    // bogus angle delta from the stale raw angle on the first control tick.
    m_raw_angle_seeded.fill(false);
    if (!enabled)
        m_last_control_us = static_cast<uint32_t>(esp_timer_get_time());
    xSemaphoreGive(m_mutex);

    if (enabled)
    {
        // Stop the motors before the caller drives them.
        auto & driver = BL48250::get_instance();
        driver.set_duties({}, driver.get_directions());
    }
}

bool WheelController::is_open_loop() const
{
    return m_open_loop;
}

/**
 * @brief Enables or disables the low-speed pulse/chopper mode.
 *
 * The calibration routine owns the wheel outputs through the continuous PID
 * law while it identifies the plant and validates the gains. Its validation
 * targets fall below the per-wheel continuous branch floor, which would hand
 * the wheel to the pulse/chopper mode and make the step metrics meaningless, so
 * the routine disables the low-speed mode for its whole run.
 */
void WheelController::set_low_speed_mode_enabled(bool enabled)
{
    if (!m_mutex)
        return;
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    m_low_speed_mode_enabled = enabled;
    xSemaphoreGive(m_mutex);
}

// ============================================================================
// PUBLIC API — Position control
// ============================================================================

/**
 * @brief Retrieves the continuous wheel-shaft angles in the motor frame.
 *
 * The estimator filter keeps an unwrapped angle in the encoder frame; the
 * motor frame is its negation, matching the velocity sign used by the loop.
 */
std::array<float, 4> WheelController::get_position_angles_rad() const
{
    // The accumulator updated by the control task is the position reference:
    // it derives the angle from the raw encoder deltas, which follow the real
    // motion even when the commanded direction disagrees with it.
    std::array<float, 4> angles = {};
    if (m_mutex && xSemaphoreTake(m_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        angles = m_position_angle_rad;
        xSemaphoreGive(m_mutex);
    }
    return angles;
}

/**
 * @brief Starts a position move for the masked wheels.
 *
 * Relative moves add the requested angle to the current continuous angle;
 * absolute moves target that angle (counted from boot). The move holds the
 * target when reached and times out after POSITION_TIMEOUT_US.
 */
esp_err_t WheelController::start_position_move(uint8_t wheel_mask,
                                               const std::array<float, 4> & angles_rad,
                                               bool relative,
                                               float max_velocity_rad_s)
{
    if (!m_mutex)
        return ESP_ERR_INVALID_STATE;
    const uint8_t mask = static_cast<uint8_t>(wheel_mask & 0x0F);
    if (mask == 0)
        return ESP_ERR_INVALID_ARG;

    // Every requested wheel needs a usable encoder before anything moves.
    auto & estimator = WheelStateEstimator::get_instance();
    for (size_t i = 0; i < 4; ++i)
    {
        if ((mask & (1U << i)) == 0)
            continue;
        if (!estimator.is_channel_calibrated(i))
        {
            if (m_mutex)
            {
                xSemaphoreTake(m_mutex, portMAX_DELAY);
                m_position_error = PositionError::ENCODER;
                xSemaphoreGive(m_mutex);
            }
            return ESP_ERR_INVALID_STATE;
        }
    }

    const auto current_angles = get_position_angles_rad();
    const float max_velocity = (max_velocity_rad_s > 0.1f)
                                 ? std::min(max_velocity_rad_s, config::driver::WHEEL_MAX_VEL_RAD)
                                 : POSITION_DEFAULT_MAX_RAD_S;

    xSemaphoreTake(m_mutex, portMAX_DELAY);
    for (size_t i = 0; i < 4; ++i)
    {
        if ((mask & (1U << i)) == 0)
            continue;
        m_position_target_rad_s[i] = relative ? current_angles[i] + angles_rad[i] : angles_rad[i];
        m_position_angle_start_rad[i] = current_angles[i];
        m_target_velocity_rad_s[i] = 0.0f;
    }
    m_position_active_mask = mask;
    m_position_done_mask = 0;
    m_position_error = PositionError::NONE;
    m_position_start_us = static_cast<uint32_t>(esp_timer_get_time());
    m_position_max_velocity_rad_s = max_velocity;
    for (auto & pid : m_pids) pid.clear();
    for (int i = 0; i < 4; ++i) reset_control_state_locked(i);
    xSemaphoreGive(m_mutex);

    ESP_LOGI(TAG,
             "Position move%s mask=0x%X max=%.1f rad/s",
             relative ? " (relative)" : " (absolute)",
             static_cast<unsigned>(mask),
             static_cast<double>(max_velocity));
    return ESP_OK;
}

/**
 * @brief Cancels the position moves and returns the wheels to velocity mode.
 */
void WheelController::stop_position_moves()
{
    if (!m_mutex)
        return;
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    const uint8_t active = m_position_active_mask;
    for (size_t i = 0; i < 4; ++i)
    {
        if ((active & (1U << i)) != 0)
            m_target_velocity_rad_s[i] = 0.0f;
    }
    cancel_position_moves_locked();
    m_position_error = PositionError::NONE;
    for (auto & pid : m_pids) pid.clear();
    for (int i = 0; i < 4; ++i) reset_control_state_locked(i);
    xSemaphoreGive(m_mutex);
}

void WheelController::cancel_position_moves_locked()
{
    m_position_active_mask = 0;
    m_position_done_mask = 0;
}

/**
 * @brief Drops one wheel's controller-managed integral and start-up kick.
 *
 * Called whenever the PID state is cleared (zero override, parameter change,
 * open-loop entry/exit, position mode): the integrator must not carry duty
 * across those transitions.
 */
void WheelController::reset_control_state_locked(int wheel)
{
    if (wheel < 0 || wheel >= 4)
        return;
    m_integral_duty[wheel] = 0.0f;
    m_kick_state[wheel] = KickState();
    m_wheel_state[wheel].target_direction_pending = false;
    m_last_target_sign[wheel] = 0;
    m_ls_fault_ms[wheel] = 0;
    m_ls_fault_sticky[wheel] = false;
    m_ls_blocked_warn_ms[wheel] = 0;
    reset_low_speed_locked(wheel);
}

/**
 * @brief Drops one wheel's low-speed pulse-tracker state.
 *
 * The feedback-fault latch is not cleared here: it must survive a pulse-mode
 * re-entry and is only released together with the control state (target
 * release, direction change, parameter change or open-loop entry).
 */
void WheelController::reset_low_speed_locked(int wheel)
{
    if (wheel < 0 || wheel >= 4)
        return;
    m_low_speed[wheel] = LowSpeedState();
}

uint8_t WheelController::get_position_active_mask() const
{
    return m_position_active_mask;
}

uint8_t WheelController::get_position_done_mask() const
{
    return m_position_done_mask;
}

WheelController::PositionError WheelController::get_position_error() const
{
    return m_position_error;
}

std::array<float, 4> WheelController::get_position_targets_rad() const
{
    return m_position_target_rad_s;
}

// ============================================================================
// PUBLIC API — Parameters and persistence
// ============================================================================

std::array<WheelController::Parameters, 4> WheelController::get_parameters() const
{
    if (!m_mutex)
        return m_parameters;
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    const auto parameters = m_parameters;
    xSemaphoreGive(m_mutex);
    return parameters;
}

/**
 * @brief Applies one wheel's control parameters to the running PID.
 */
void WheelController::apply_parameters(int wheel, const Parameters & parameters)
{
    if (!m_mutex || wheel < 0 || wheel >= 4)
        return;
    const Parameters clamped = clamp_parameters(parameters);

    xSemaphoreTake(m_mutex, portMAX_DELAY);
    m_parameters[wheel] = clamped;
    // FastPID runs P and D only; ki is the controller-managed integral gain.
    m_pids[wheel].configure(clamped.kp, 0.0f, clamped.kd, PID_HZ, 16, true);
    m_pids[wheel].set_output_range(-10230, 10230);
    m_pids[wheel].clear();
    reset_control_state_locked(wheel);
    xSemaphoreGive(m_mutex);
}

// Order of the float fields in DebugWheelTuning. The pulse pattern flags
// follow them in the tuning array.
template <typename Parameters>
static auto tuning_fields(Parameters & p)
{
    return std::array{&p.kp,
                      &p.ki,
                      &p.kd,
                      &p.feedforward,
                      &p.velocity_ff,
                      &p.static_friction,
                      &p.continuous_min_rad_s[0],
                      &p.pulse_duty[0],
                      &p.pulse_off_duty[0],
                      &p.pulse_floor_duty[0],
                      &p.pulse_response_s[0],
                      &p.pulse_coast_delay_s[0],
                      &p.feedforward_rev,
                      &p.velocity_ff_rev,
                      &p.static_friction_rev,
                      &p.continuous_min_rad_s[1],
                      &p.pulse_duty[1],
                      &p.pulse_off_duty[1],
                      &p.pulse_floor_duty[1],
                      &p.pulse_response_s[1],
                      &p.pulse_coast_delay_s[1]};
}

WheelController::VelocityMode WheelController::get_live_tuning(int wheel,
                                                               TuningValues & values) const
{
    if (!m_mutex || wheel < 0 || wheel >= static_cast<int>(m_parameters.size()))
        return VelocityMode::STOPPED;

    xSemaphoreTake(m_mutex, portMAX_DELAY);
    const Parameters & p = m_parameters[wheel];
    const auto fields = tuning_fields(p);
    for (size_t i = 0; i < fields.size(); ++i) values[i] = *fields[i];
    for (size_t i = 0; i < p.pulse_interleaved.size(); ++i)
        values[fields.size() + i] = p.pulse_interleaved[i] != 0;

    VelocityMode mode = VelocityMode::STOPPED;
    if (!m_open_loop && m_last_target_sign[wheel] != 0)
    {
        const auto & pulse = m_low_speed[wheel];
        mode = pulse.active
                 ? VelocityMode::PULSED
                 : (pulse.selected ? VelocityMode::PULSE_BLOCKED : VelocityMode::CONTINUOUS);
    }
    xSemaphoreGive(m_mutex);
    return mode;
}

/**
 * @brief Changes selected parameters together without restarting the wheel loop.
 */
esp_err_t WheelController::apply_live_tuning(int wheel,
                                             uint32_t fields,
                                             const TuningValues & values)
{
    if (!m_mutex)
        return ESP_ERR_INVALID_STATE;
    if (wheel < 0 || wheel >= static_cast<int>(m_parameters.size()) || fields == 0 ||
        fields >> values.size())
        return ESP_ERR_INVALID_ARG;

    for (size_t i = 0; i < values.size(); ++i)
        if ((fields & (1U << i)) && !std::isfinite(values[i]))
            return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(m_mutex, portMAX_DELAY);
    if (m_open_loop || !m_low_speed_mode_enabled)
    {
        xSemaphoreGive(m_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    const Parameters & current = m_parameters[wheel];
    Parameters updated = current;
    const auto changes = tuning_fields(updated);
    static_assert(changes.size() + updated.pulse_interleaved.size() == TuningValues{}.size());
    for (size_t i = 0; i < changes.size(); ++i)
        if (fields & (1U << i))
            *changes[i] = values[i];
    for (size_t i = 0; i < updated.pulse_interleaved.size(); ++i)
        if (fields & (1U << (changes.size() + i)))
            updated.pulse_interleaved[i] = values[changes.size() + i] != 0.0f;
    updated = clamp_parameters(updated);

    if (updated.kp != current.kp || updated.kd != current.kd)
        m_pids[wheel].set_coefficients(updated.kp, 0.0f, updated.kd, PID_HZ);
    if ((current.ki < LEGACY_KI_THRESHOLD) != (updated.ki < LEGACY_KI_THRESHOLD))
        m_integral_duty[wheel] = 0.0f;
    m_parameters[wheel] = updated;
    xSemaphoreGive(m_mutex);
    return ESP_OK;
}

// One write keeps the gains and motor measurements for a wheel together. A
// read queued behind the write confirms the data before reporting success.
static esp_err_t store_pid_blob(const char * key, const void * data, size_t length)
{
    const esp_err_t queued = NVSManager::save_blob("pid", key, data, length);
    if (queued != ESP_OK)
        return queued;

    std::array<uint8_t, NVSManager::BLOB_MAX> stored;
    size_t size = length;
    const esp_err_t read = NVSManager::load_blob("pid", key, stored.data(), &size);
    if (read != ESP_OK || size != length || std::memcmp(data, stored.data(), length) != 0)
    {
        ESP_LOGE(TAG, "Could not verify saved PID data for %s.", key);
        return read == ESP_OK ? ESP_FAIL : read;
    }
    return ESP_OK;
}

esp_err_t WheelController::save_parameters(int wheel)
{
    static_assert(std::is_trivially_copyable_v<Parameters>);
    static_assert(sizeof(Parameters) <= NVSManager::BLOB_MAX);
    if (wheel < 0 || wheel >= static_cast<int>(m_parameters.size()))
        return ESP_ERR_INVALID_ARG;

    const Parameters parameters = get_parameters()[wheel];
    const std::string key = std::string(MOTOR_KEYS[wheel]) + PARAMETERS_SUFFIX;
    const esp_err_t err = store_pid_blob(key.c_str(), &parameters, sizeof(parameters));
    if (err == ESP_OK)
        ESP_LOGI(TAG, "Saved control parameters for wheel %d.", wheel + 1);
    return err;
}

void WheelController::save_all_parameters()
{
    for (size_t wheel = 0; wheel < m_parameters.size(); ++wheel) save_parameters(wheel);
}

esp_err_t WheelController::save_max_velocity()
{
    if (!m_mutex)
        return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    const float limit = m_max_velocity_rad_s;
    xSemaphoreGive(m_mutex);
    const esp_err_t err = store_pid_blob(MAX_VELOCITY_KEY, &limit, sizeof(limit));
    if (err == ESP_OK)
        ESP_LOGI(
          TAG, "Saved shared speed limit: %.1f RPM.", static_cast<double>(limit * RAD_S_TO_RPM));
    return err;
}

void WheelController::load_parameters_from_nvs()
{
    m_max_velocity_rad_s = config::driver::WHEEL_MAX_VEL_RAD;
    float stored_limit = 0.0f;
    size_t length = sizeof(stored_limit);
    if (NVSManager::load_blob("pid", MAX_VELOCITY_KEY, &stored_limit, &length) == ESP_OK &&
        length == sizeof(stored_limit) && stored_limit > 0.0f &&
        stored_limit <= config::driver::WHEEL_MAX_VEL_RAD)
        m_max_velocity_rad_s = stored_limit;

    for (size_t wheel = 0; wheel < m_parameters.size(); ++wheel)
    {
        const std::string base = MOTOR_KEYS[wheel];
        const auto load = [&base](const char * suffix, auto & value) {
            const std::string key = base + suffix;
            size_t size = sizeof(value);
            return NVSManager::load_blob("pid", key.c_str(), &value, &size) == ESP_OK &&
                   size == sizeof(value);
        };
        Parameters parameters;
        if (!load(PARAMETERS_SUFFIX, parameters))
        {
            // Preserve settings from firmware that wrote individual fields.
            parameters = Parameters();
            const auto load_float = [&load](const char * suffix, float & value, float fallback) {
                if (!load(suffix, value))
                    value = fallback;
            };
            load_float(PID_SUFFIXES[0], parameters.kp, PID_DEFAULT_GAINS[0]);
            load_float(PID_SUFFIXES[1], parameters.ki, PID_DEFAULT_GAINS[1]);
            load_float(PID_SUFFIXES[2], parameters.kd, PID_DEFAULT_GAINS[2]);
            load_float(FEEDFORWARD_SUFFIX, parameters.feedforward, STATIC_FRICTION_THRESHOLD);
            load_float(VELOCITY_FF_SUFFIX, parameters.velocity_ff, 0.0f);
            load_float(
              STATIC_FRICTION_SUFFIX, parameters.static_friction, STATIC_FRICTION_THRESHOLD);
            load_float(FEEDFORWARD_REV_SUFFIX, parameters.feedforward_rev, -parameters.feedforward);
            load_float(VELOCITY_FF_REV_SUFFIX, parameters.velocity_ff_rev, parameters.velocity_ff);
            load_float(STATIC_FRICTION_REV_SUFFIX,
                       parameters.static_friction_rev,
                       parameters.static_friction);
            load_float(ANGLE_SIGN_SUFFIX, parameters.angle_sign, -1.0f);
            load(CONTINUOUS_MIN_SUFFIX, parameters.continuous_min_rad_s);
            load(PULSE_DUTY_SUFFIX, parameters.pulse_duty);
            load(PULSE_OFF_DUTY_SUFFIX, parameters.pulse_off_duty);
            load(PULSE_FLOOR_SUFFIX, parameters.pulse_floor_duty);
            load(PULSE_RESPONSE_SUFFIX, parameters.pulse_response_s);
            load(PULSE_COAST_SUFFIX, parameters.pulse_coast_delay_s);
            load(PULSE_PHASE_SUFFIX, parameters.pulse_interleaved);
        }
        m_parameters[wheel] = clamp_parameters(parameters);
    }
}

// ============================================================================
// CONTROL TASK
// ============================================================================

void WheelController::task_wrapper(void * arg)
{
    static_cast<WheelController *>(arg)->control_task();
}

bool IRAM_ATTR WheelController::on_control_alarm(gptimer_handle_t,
                                                 const gptimer_alarm_event_data_t *,
                                                 void * context)
{
    auto * controller = static_cast<WheelController *>(context);
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(controller->m_task_handle, &woken);
    return woken == pdTRUE;
}

/**
 * @brief Updates the four wheel outputs from the latest encoder samples.
 */
void WheelController::control_task()
{
    m_last_control_us = static_cast<uint32_t>(esp_timer_get_time());
    m_last_slow_us = m_last_control_us;
    m_last_rip_report_us = m_last_control_us;
#if CONFIG_ENCODER_DIAG
    uint32_t diag_start_us = m_last_control_us;
    uint32_t diag_steps = 0;
    uint32_t diag_missed = 0;
    uint32_t diag_max_gap_us = 0;
    uint32_t diag_max_work_us = 0;
    uint32_t diag_max_age_us = 0;
#endif

    while (true)
    {
        // Coalesce elapsed alarms into one calculation from the latest samples.
        const uint32_t alarms = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const int64_t start_us = esp_timer_get_time();
        const uint32_t now_us = static_cast<uint32_t>(start_us);
        const uint32_t gap_us = now_us - m_last_control_us;
        m_last_control_us = now_us;
        const uint32_t slow_elapsed_us = now_us - m_last_slow_us;
        const bool slow_tick = slow_elapsed_us >= SLOW_CONTROL_PERIOD_US;
        const float slow_dt_ms = static_cast<float>(slow_elapsed_us) / 1000.0f;
        if (slow_tick)
            m_last_slow_us = now_us - slow_elapsed_us % SLOW_CONTROL_PERIOD_US;
        const uint32_t now_ms = static_cast<uint32_t>(start_us / 1000);
        const bool report_ripple = now_us - m_last_rip_report_us >= RIP_REPORT_US;
#if CONFIG_ENCODER_DIAG
        ++diag_steps;
        diag_missed += alarms - 1;
        diag_max_gap_us = std::max(diag_max_gap_us, gap_us);
#else
        (void)alarms;
        (void)gap_us;
#endif

        if (!m_mutex)
            continue;

        // Skip the loop in open-loop mode (a calibration routine owns the motors).
        bool open_loop = false;
        if (xSemaphoreTake(m_mutex, 0) == pdTRUE)
        {
            open_loop = m_open_loop;
            xSemaphoreGive(m_mutex);
        }
        if (open_loop)
            continue;

        WheelStateEstimator::get_instance().get_control_snapshot(&m_control_snapshot);
        const int64_t snapshot_read_us = esp_timer_get_time();
        // Read the filtered angular velocity (encoder frame) and the raw
        // encoder angles before taking the controller mutex.
        const std::array<float, 4> & current_rad_s = m_control_snapshot.smoothed_velocity_rad_s;
        const std::array<float, 4> & raw_angles = m_control_snapshot.measured_angle_rad;
        // Low-speed drive needs a working analog encoder.
        const uint8_t stuck_analog_mask = m_control_snapshot.stuck_analog_mask;
        // Instantaneous clamp check: a momentary dead sample of an intermittent
        // harness must not be accumulated as a bogus angle delta.
        const uint8_t clamped_raw_mask = m_control_snapshot.clamped_raw_mask;
        std::array<uint32_t, 4> duties;
        std::array<uint8_t, 4> dirs;
        std::array<int32_t, 4> pulse_counts = {};
        bool pulse_counts_read = false;

        xSemaphoreTake(m_mutex, portMAX_DELAY);

        // Open-loop mode owns the motor outputs: never write them from the PID.
        if (m_open_loop)
        {
            xSemaphoreGive(m_mutex);
            continue;
        }

        auto & driver = BL48250::get_instance();
        const auto latched_directions = driver.get_directions();
        const auto applied_duties = driver.get_duty_cycles();
        for (int i = 0; i < 4; ++i)
        {
            WheelState & wheel = m_wheel_state[i];
            const int8_t latched_direction =
              latched_directions[i] == config::driver::MOTOR_CW ? 1 : -1;
            const size_t latched_branch = latched_direction > 0 ? 0 : 1;
            // Only pulse mode uses the tighter encoder jump limit. A wheel
            // entering it can still be coasting from a continuous target.
            const float maximum_tracked_rate =
              m_low_speed[i].active
                ? std::max({LS_MAX_WHEEL_RATE_RAD_S,
                            1.5f * m_parameters[i].continuous_min_rad_s[latched_branch],
                            std::fabs(current_rad_s[i])})
                : config::driver::WHEEL_MAX_VEL_RAD;
            // Negate because the estimator reports the encoder frame, where CW
            // rotation is negative; targets and the PID use the motor frame.
            float measured_rad_s = -current_rad_s[i];
            m_current_velocity_rad_s[i] = measured_rad_s;

            // Accumulate the continuous position from the raw encoder deltas.
            // The raw angle decreases for positive motor rotation. A clamped
            // sample (dead input) is skipped and forces a re-seed, so an
            // intermittent harness cannot inject a bogus angle jump. The
            // accumulated angle is also the feedback of the low-speed pulse
            // tracker below.
            bool angle_glitch = false;
            bool new_angle = false;
            const int64_t sample_us = m_control_snapshot.sample_us[i];
            const int64_t sample_age_us = snapshot_read_us - sample_us;
            const bool snapshot_fresh = sample_us != 0 && sample_age_us <= LS_SNAPSHOT_MAX_AGE_US;
#if CONFIG_ENCODER_DIAG
            if (sample_us != 0)
                diag_max_age_us = std::max(diag_max_age_us, static_cast<uint32_t>(sample_age_us));
#endif
            if (!snapshot_fresh || (clamped_raw_mask & (1U << i)) != 0)
                m_raw_angle_seeded[i] = false;
            else if (sample_us != m_last_checked_sample_us[i] || !m_raw_angle_seeded[i])
            {
                new_angle = true;
                m_last_checked_sample_us[i] = sample_us;
                if (m_raw_angle_seeded[i])
                {
                    float delta = raw_angles[i] - m_last_raw_angle_rad[i];
                    if (delta > RAW_ANGLE_PERIOD_RAD * 0.5f)
                        delta -= RAW_ANGLE_PERIOD_RAD;
                    else if (delta < -RAW_ANGLE_PERIOD_RAD * 0.5f)
                        delta += RAW_ANGLE_PERIOD_RAD;
                    // angle_sign says whether the raw angle increases (+1) or
                    // decreases (-1) for a positive motor rotation.
                    const float wheel_angle_delta =
                      m_parameters[i].angle_sign * delta * RAW_ANGLE_TO_WHEEL;
                    const float sample_interval_s =
                      static_cast<float>(sample_us - m_last_raw_snapshot_us[i]) * 1e-6f;
                    const float max_delta =
                      maximum_tracked_rate * sample_interval_s + ENCODER_DELTA_MARGIN_RAD;
                    angle_glitch = std::fabs(wheel_angle_delta) > max_delta;
                    if (!angle_glitch)
                        m_position_angle_rad[i] += wheel_angle_delta;
                }
                // Keep the last good sample as the reference when a jump is rejected.
                if (!angle_glitch)
                {
                    m_last_raw_angle_rad[i] = raw_angles[i];
                    m_last_raw_snapshot_us[i] = sample_us;
                    m_raw_angle_seeded[i] = true;
                }
            }

            const bool velocity_feedback_available =
              snapshot_fresh && (m_control_snapshot.smoothed_valid_mask & (1U << i)) != 0 &&
              ((stuck_analog_mask | clamped_raw_mask) & (1U << i)) == 0 &&
              std::isfinite(measured_rad_s);

            // ---- Resolve the target: position mode or a velocity target ----
            bool position = (m_position_active_mask & (1U << i)) != 0;
            float target_rad_s = m_target_velocity_rad_s[i];
            if (position)
            {
                const bool done = (m_position_done_mask & (1U << i)) != 0;
                const float travelled = m_position_angle_rad[i] - m_position_angle_start_rad[i];
                const float requested = m_position_target_rad_s[i] - m_position_angle_start_rad[i];
                const bool overtravel =
                  std::fabs(travelled) > std::fabs(requested) + POSITION_OVERTRAVEL_RAD;
                if (!done && ((now_us - m_position_start_us) > POSITION_TIMEOUT_US || overtravel))
                {
                    m_position_active_mask &= static_cast<uint8_t>(~(1U << i));
                    m_position_done_mask &= static_cast<uint8_t>(~(1U << i));
                    m_position_error = PositionError::TIMEOUT;
                    target_rad_s = 0.0f;
                }
                else
                {
                    const float error = m_position_target_rad_s[i] - m_position_angle_rad[i];
                    if (std::fabs(error) <= POSITION_TOLERANCE_RAD &&
                        std::fabs(measured_rad_s) < POSITION_VELOCITY_THRESHOLD_RAD_S)
                    {
                        m_position_done_mask |= static_cast<uint8_t>(1U << i);
                        target_rad_s = 0.0f;
                    }
                    else
                    {
                        m_position_done_mask &= static_cast<uint8_t>(~(1U << i));
                        target_rad_s = POSITION_KP * error - POSITION_KD * measured_rad_s;
                        target_rad_s = std::clamp(target_rad_s,
                                                  -m_position_max_velocity_rad_s,
                                                  m_position_max_velocity_rad_s);
                        if (std::fabs(error) > POSITION_TOLERANCE_RAD &&
                            std::fabs(target_rad_s) < POSITION_CREEP_RAD_S)
                        {
                            target_rad_s =
                              (error > 0.0f) ? POSITION_CREEP_RAD_S : -POSITION_CREEP_RAD_S;
                        }
                    }
                }
            }

            // ---- Calibrated speed limit ----
            // Enforce it on every target source, including the position-mode
            // velocity commands resolved above.
            target_rad_s = std::clamp(target_rad_s, -m_max_velocity_rad_s, m_max_velocity_rad_s);
            const int target_sign = signf(target_rad_s);

            // A zero target disables drive while the wheel coasts. Evaluating
            // the velocity model here can sustain motion through its intercept.
            if (target_rad_s == 0.0f)
            {
                duties[i] = 0;
                const bool was_pulsing = m_low_speed[i].active;
                m_pids[i].clear();
                reset_control_state_locked(i);
                if (was_pulsing)
                    ESP_LOGI(TAG, "Motor %d continuous law restored.", i);
                // The kick re-arms only with the target at zero and the wheel
                // stopped: the next engagement starts with exactly one pulse.
                if (std::fabs(measured_rad_s) < KICK_REARM_RAD_S)
                    m_kick_state[i].armed = true;
                wheel.duty = 0;
                wheel.direction = latched_direction;
                dirs[i] = latched_directions[i];
                continue;
            }

            const Parameters & parameters = m_parameters[i];
            // Stored parameters from the pre-kick firmware carry ki = 0: those
            // wheels keep the legacy law until they are recalibrated. The kick
            // and the integral action need a calibrated integral gain.
            const bool legacy_law = parameters.ki < LEGACY_KI_THRESHOLD;

            // ---- Target sign bookkeeping ----
            // A large magnitude change clears the previous operating point's integral.
            if (std::fabs(target_rad_s - m_last_target_rad_s[i]) > 0.5f)
                m_integral_duty[i] = 0.0f;
            m_last_target_rad_s[i] = target_rad_s;
            const bool target_direction_changed = target_sign != m_last_target_sign[i];
            if (target_direction_changed)
            {
                m_pids[i].clear();
                reset_control_state_locked(i);
                m_last_target_sign[i] = static_cast<int8_t>(target_sign);
                m_kick_state[i].armed = true;
                // Start the driver's delay on the fast tick. Hold the new
                // direction through the delay even if the controller asks to coast.
                wheel.target_direction_pending = target_sign != latched_direction;
                wheel.direction = static_cast<int8_t>(target_sign);
                wheel.duty = 0;
            }
            if (target_sign == latched_direction)
                wheel.target_direction_pending = false;

            // ---- Feed-forward: the fitted static curve of the target
            // direction. Each direction has its own branch, so the intercept
            // and the velocity gain follow the target sign. The breakaway duty
            // is not a steady floor: it is applied below as a start-up kick.
            const float direction = static_cast<float>(target_sign);
            const bool forward = target_rad_s >= 0.0f;
            const float ff_intercept =
              forward ? parameters.feedforward : parameters.feedforward_rev;
            const float velocity_ff = forward ? parameters.velocity_ff : parameters.velocity_ff_rev;
            const float breakaway =
              forward ? parameters.static_friction : parameters.static_friction_rev;
            const float model_ff = ff_intercept + velocity_ff * target_rad_s;
            float ff_pwm = std::clamp(model_ff, -1023.0f, 1023.0f);

            // ---- Low-speed pulse mode selection ----
            // Use pulses below the measured continuous boundary for this
            // direction. Hysteresis keeps nearby targets in the same mode.
            LowSpeedState & low_speed = m_low_speed[i];
            const float target_magnitude_abs = std::fabs(target_rad_s);
            const bool angle_feedback_available =
              snapshot_fresh && (stuck_analog_mask & (1U << i)) == 0;
            if (!m_ls_fault_sticky[i] && m_ls_fault_ms[i] != 0 && angle_feedback_available &&
                now_ms - m_ls_fault_ms[i] >= LS_FAULT_RETRY_MS)
                m_ls_fault_ms[i] = 0;
            const bool feedback_available = angle_feedback_available && m_ls_fault_ms[i] == 0;
            const size_t branch = forward ? 0 : 1;
            const float calibrated_gate = parameters.continuous_min_rad_s[branch];
            float continuous_gate =
              calibrated_gate > 0.0f ? calibrated_gate : LS_FALLBACK_CONTINUOUS_GATE_RAD_S;
            if (calibrated_gate == 0.0f && !legacy_law && velocity_ff > 0.0f && breakaway > 0.0f)
            {
                // The fitted duty must exceed the measured breakaway in the
                // requested direction before the wheel can use continuous drive.
                const float model_gate =
                  (breakaway + LS_BREAKAWAY_MARGIN_DUTY - direction * ff_intercept) / velocity_ff;
                if (model_gate > 0.0f)
                    continuous_gate = std::min(continuous_gate, model_gate);
            }
            const float gate =
              continuous_gate * (low_speed.selected ? 1.0f + PULSE_HANDOFF_HYSTERESIS_FRACTION
                                                    : 1.0f - PULSE_HANDOFF_HYSTERESIS_FRACTION);
            const bool pulse_requested = LS_MODE_ENABLED && m_low_speed_mode_enabled && !position &&
                                         target_magnitude_abs <= gate;
            const bool pulse_eligible = pulse_requested && feedback_available;
            // Below the floor the continuous law inherits the legacy breakaway
            // floor when the chopper cannot run. Holding the wheel still is
            // safer than spinning it far above the requested speed.
            const bool low_speed_blocked = pulse_requested && !feedback_available;
            if (pulse_eligible != low_speed.active)
            {
                low_speed = LowSpeedState();
                low_speed.active = pulse_eligible;
                // Crossing the two laws must not carry the other law's state:
                // clear the PID and the controller-managed integral.
                m_pids[i].clear();
                m_integral_duty[i] = 0.0f;
                m_kick_state[i] = KickState();
                if (pulse_eligible)
                {
                    // A same-direction handoff starts from the current motion.
                    low_speed.starting = direction * measured_rad_s < LS_MOVING_RPM * RPM_TO_RAD_S;
                    low_speed.filtered_rpm = direction * measured_rad_s * RAD_S_TO_RPM;
                    ESP_LOGI(TAG,
                             "M%d low-speed pulse mode: target=%.2f rad/s",
                             i + 1,
                             static_cast<double>(target_rad_s));
                }
                else
                {
                    ESP_LOGI(TAG, "M%d continuous law restored.", i + 1);
                }
            }
            low_speed.selected = pulse_requested;

            // The continuous law keeps its validated 100 Hz timing: on the fast
            // ticks hold the last output and let the pulse mode (below) run only
            // when it owns the wheel.
            if (!low_speed.active && !low_speed_blocked && !slow_tick)
            {
                duties[i] = wheel.duty;
                const bool reverse = wheel.direction < 0;
                dirs[i] = reverse ? config::driver::MOTOR_CCW : config::driver::MOTOR_CW;
                continue;
            }

            float u = 0.0f;
            if (low_speed.active)
            {
                const float startup_duty = breakaway + LS_BREAKAWAY_MARGIN_DUTY;
                const float pulse_duty = parameters.pulse_duty[branch];
                // Once the wheel is moving, calibration can use a lower pulse
                // amplitude than it takes to start the wheel from rest.
                const float minimum_duty = low_speed.starting || pulse_duty == 0.0f
                                             ? std::max(startup_duty, pulse_duty)
                                             : pulse_duty;
                const float base_duty =
                  std::min(std::max(minimum_duty, direction * model_ff + LS_BREAKAWAY_MARGIN_DUTY),
                           MAX_PULSE_DUTY);
                const float drive_duty = std::min(base_duty + low_speed.boost_duty, MAX_PULSE_DUTY);
                const float off_duty = std::min(parameters.pulse_off_duty[branch], drive_duty);
                const float wheel_angle = m_position_angle_rad[i];
                constexpr float period_ticks = static_cast<float>(LS_CHOPPER_PERIOD_TICKS);

                // Ignore isolated outliers. A persistent run of rejected samples
                // means the pulse controller can no longer track wheel travel.
                if (new_angle)
                {
                    if (!angle_glitch)
                        low_speed.glitch_first_us = 0;
                    else if (low_speed.glitch_first_us == 0)
                        low_speed.glitch_first_us = now_us;
                }
                if (low_speed.glitch_first_us != 0 &&
                    now_us - low_speed.glitch_first_us >= LS_GLITCH_CONFIRM_US)
                {
                    m_ls_fault_ms[i] = now_ms;
                    m_ls_fault_sticky[i] = true;
                    low_speed.active = false;
                    ESP_LOGE(TAG, "LS M%d persistent angle fault; pulse drive off.", i + 1);
                }

                if (!low_speed.active)
                {
                    m_pids[i].clear();
                    m_integral_duty[i] = 0.0f;
                    m_kick_state[i] = KickState();
                    u = 0.0f;
                    ESP_LOGE(TAG, "LS M%d angle feedback unusable; low-speed drive off.", i + 1);
                }
                else
                {
                    const float target_magnitude = std::fabs(target_rad_s);
                    const float runaway_limit =
                      std::max(LS_RUNAWAY_FACTOR * target_magnitude, LS_RUNAWAY_FLOOR_RAD_S);
                    if (std::fabs(measured_rad_s) > runaway_limit)
                    {
                        // Coast during the overspeed. Keep the learned drive
                        // and travel history so recovery does not restart from rest.
                        low_speed.cycle_start_us = 0;
                        low_speed.stall_since_us = 0;
                        low_speed.feedback_us = 0;
                        low_speed.fast_sample_us = 0;
                        low_speed.fast_acceleration_rpm_s = 0.0f;
                        low_speed.filtered_rpm = direction * measured_rad_s * RAD_S_TO_RPM;
                        u = 0.0f;
                        if (now_ms - low_speed.last_warn_ms >= 1000)
                        {
                            low_speed.last_warn_ms = now_ms;
                            ESP_LOGW(TAG,
                                     "LS M%d speed guard: v=%.1f rad/s target=%.2f rad/s",
                                     i + 1,
                                     static_cast<double>(measured_rad_s),
                                     static_cast<double>(target_rad_s));
                        }
                    }
                    else
                    {
                        // Use signed travel. Taking the magnitude of every short
                        // window turns encoder jitter at rest into forward motion.
                        if (low_speed.window_start_us == 0 ||
                            now_us - low_speed.window_start_us >= RIP_WINDOW_US)
                        {
                            if (low_speed.window_start_us != 0)
                            {
                                const float window_s = std::max(
                                  0.001f,
                                  static_cast<float>(now_us - low_speed.window_start_us) * 1e-6f);
                                low_speed.last_window_delta_rad =
                                  wheel_angle - low_speed.window_start_angle_rad;
                                low_speed.last_window_s = window_s;
                                const float window_speed =
                                  low_speed.last_window_delta_rad / window_s;
                                m_rip_last_mean[i] = window_speed;
                                low_speed.last_window_rpm = direction * window_speed * RAD_S_TO_RPM;
                                if (low_speed.last_window_rpm >= LS_MOVING_RPM)
                                {
                                    low_speed.starting = false;
                                    // Return to the tested duty once the restart has worked.
                                    if (low_speed.boost_duty > 0.0f)
                                    {
                                        low_speed.boost_duty = 0.0f;
                                        low_speed.stall_since_us = 0;
                                    }
                                }
                                if (window_speed < m_rip_min[i])
                                    m_rip_min[i] = window_speed;
                                if (window_speed > m_rip_max[i])
                                    m_rip_max[i] = window_speed;
                                ++m_rip_windows[i];
                            }
                            low_speed.window_start_angle_rad = wheel_angle;
                            low_speed.window_start_us = now_us;
                        }

                        const float target_rpm = target_magnitude * RAD_S_TO_RPM;
                        const bool travel_valid =
                          !angle_glitch && snapshot_fresh && (clamped_raw_mask & (1U << i)) == 0;
                        const bool fast_valid =
                          travel_valid && sample_age_us <= LS_CHOPPER_PERIOD_US &&
                          (m_control_snapshot.fast_valid_mask & (1U << i)) != 0;
                        const float fast_rpm =
                          fast_valid ? direction * parameters.angle_sign *
                                         m_control_snapshot.fast_velocity_rad_s[i] * RAD_S_TO_RPM
                                     : low_speed.last_window_rpm;
                        if (!fast_valid)
                        {
                            low_speed.fast_sample_us = 0;
                            low_speed.fast_acceleration_rpm_s = 0.0f;
                        }
                        else if (sample_us != low_speed.fast_sample_us)
                        {
                            const int64_t sample_gap_us = sample_us - low_speed.fast_sample_us;
                            if (low_speed.fast_sample_us != 0 && sample_gap_us > 0 &&
                                sample_gap_us <= LS_CHOPPER_PERIOD_US)
                            {
                                constexpr float acceleration_filter_s =
                                  LS_CHOPPER_PERIOD_US * 1e-6f;
                                const float elapsed_s = static_cast<float>(sample_gap_us) * 1e-6f;
                                const float delta_rpm = fast_rpm - low_speed.previous_fast_rpm;
                                low_speed.fast_acceleration_rpm_s +=
                                  (delta_rpm - elapsed_s * low_speed.fast_acceleration_rpm_s) /
                                  (acceleration_filter_s + elapsed_s);
                            }
                            else
                                low_speed.fast_acceleration_rpm_s = 0.0f;
                            low_speed.fast_sample_us = sample_us;
                            low_speed.previous_fast_rpm = fast_rpm;
                        }
                        // A low-speed surge can accelerate faster than the continuous
                        // plant fit predicts. Use the encoder slope and check for
                        // overshoot on every control tick, including within a pulse.
                        const float coast_delay_s = parameters.pulse_coast_delay_s[branch];
                        const float projected_rpm =
                          fast_rpm +
                          coast_delay_s * std::max(0.0f, low_speed.fast_acceleration_rpm_s);
                        const bool coasting = coast_delay_s > 0.0f && projected_rpm >= target_rpm;

                        // Use the nearest scheduled edge. Interrupt jitter
                        // around an edge must not insert an extra drive slot.
                        uint32_t phase_us =
                          now_us - low_speed.cycle_start_us + CONTROL_PERIOD_US / 2;
                        const bool new_cycle =
                          low_speed.cycle_start_us == 0 || phase_us >= LS_CHOPPER_PERIOD_US;
                        if (new_cycle)
                        {
                            if (low_speed.cycle_start_us == 0)
                            {
                                // Spread simultaneous wheel pulses over the cycle.
                                const uint32_t phase_offset =
                                  (static_cast<uint32_t>(i) * LS_CHOPPER_PERIOD_TICKS /
                                   duties.size()) *
                                  CONTROL_PERIOD_US;
                                low_speed.cycle_start_us = now_us - phase_offset;
                                phase_us = phase_offset + CONTROL_PERIOD_US / 2;
                            }
                            else
                            {
                                const uint32_t phase = phase_us % LS_CHOPPER_PERIOD_US;
                                low_speed.cycle_start_us += phase_us - phase;
                                phase_us = phase;
                            }
                            // Invert the same directional fit used to select duty.
                            // A wheel without a fitted slope starts from the common gate.
                            const float full_drive_rad_s =
                              velocity_ff > 0.0f
                                ? (drive_duty - direction * ff_intercept) / velocity_ff
                                : LS_PULSE_REFERENCE_SPEED_RAD_S;
                            const float full_drive_rpm =
                              std::max(full_drive_rad_s, target_magnitude) * RAD_S_TO_RPM;
                            const float duty_span = drive_duty - off_duty;
                            const float minimum_on_fraction =
                              duty_span > 0.0f
                                ? std::clamp(
                                    (parameters.pulse_floor_duty[branch] - off_duty) / duty_span,
                                    0.0f,
                                    1.0f)
                                : 0.0f;
                            const float on_fraction =
                              minimum_on_fraction +
                              (1.0f - minimum_on_fraction) * target_rpm / full_drive_rpm;
                            const float feedforward = period_ticks * on_fraction;
                            // Normalize the correction to the fitted full-drive range
                            // and integrate at the measured motor response time.
                            const float kp = period_ticks / full_drive_rpm;
                            constexpr float velocity_filter_s = RIP_WINDOW_US * 1e-6f;
                            const float response_s =
                              std::max(parameters.pulse_response_s[branch],
                                       config::driver::BL48250_FB_WINDOW_MS * 1e-3f);
                            if (travel_valid && low_speed.feedback_us != 0)
                            {
                                const float elapsed_s = (now_us - low_speed.feedback_us) * 1e-6f;
                                const float travel =
                                  direction * (wheel_angle - low_speed.feedback_angle_rad);
                                const float alpha = elapsed_s / (elapsed_s + velocity_filter_s);
                                const float travel_rpm_s = travel * RAD_S_TO_RPM;
                                low_speed.filtered_rpm +=
                                  alpha * (travel_rpm_s / elapsed_s - low_speed.filtered_rpm);
                                const float error_rpm = target_rpm - low_speed.filtered_rpm;
                                const float demand =
                                  feedforward + kp * error_rpm + low_speed.integral_ticks;
                                const float travel_error = target_rpm * elapsed_s - travel_rpm_s;
                                // Use travel each cycle. Quantized motor counts stay
                                // independent for stall detection and calibration.
                                if (!low_speed.starting &&
                                    ((!coasting && demand < period_ticks && travel_error > 0.0f) ||
                                     (demand > 0.0f && travel_error < 0.0f)))
                                    low_speed.integral_ticks = std::clamp(
                                      low_speed.integral_ticks + kp * travel_error / response_s,
                                      -feedforward,
                                      period_ticks - feedforward);
                            }
                            low_speed.feedback_us = travel_valid ? now_us : 0;
                            low_speed.feedback_angle_rad = wheel_angle;
                            const float error_rpm = target_rpm - low_speed.filtered_rpm;
                            low_speed.last_ff_ticks = feedforward;
                            low_speed.last_error_rpm = error_rpm;
                            low_speed.ton_ticks =
                              std::clamp(feedforward + kp * error_rpm + low_speed.integral_ticks,
                                         0.0f,
                                         period_ticks);
                        }

                        // Increase drive if the wheel cannot start with the fitted duty.
                        const bool full_drive =
                          !coasting && (low_speed.starting || low_speed.ton_ticks >= period_ticks);
                        if (full_drive)
                        {
                            if (low_speed.stall_since_us == 0 ||
                                now_us - low_speed.stall_since_us >= LS_STALL_TIMEOUT_US)
                            {
                                if (!pulse_counts_read)
                                {
                                    pulse_counts = driver.pulse_counts();
                                    pulse_counts_read = true;
                                }
                                const int32_t count = pulse_counts[i];
                                if (low_speed.stall_since_us != 0 &&
                                    direction * (wheel_angle - low_speed.stall_angle_rad) <
                                      LS_STALL_ANGLE_RAD)
                                {
                                    const bool encoder_disagrees =
                                      static_cast<int64_t>(count) - low_speed.stall_pulse_count > 1;
                                    if (encoder_disagrees || drive_duty >= MAX_PULSE_DUTY)
                                    {
                                        m_ls_fault_ms[i] = now_ms;
                                        m_ls_fault_sticky[i] = true;
                                        low_speed.active = false;
                                        ESP_LOGE(TAG,
                                                 "LS M%d drive off: %s",
                                                 i + 1,
                                                 encoder_disagrees
                                                   ? "encoder and motor pulses disagree"
                                                   : "stalled at pulse duty limit");
                                    }
                                    else
                                    {
                                        low_speed.boost_duty =
                                          std::min(low_speed.boost_duty + LS_BOOST_STEP_DUTY,
                                                   MAX_PULSE_DUTY - base_duty);
                                    }
                                }
                                low_speed.stall_since_us = now_us;
                                low_speed.stall_angle_rad = wheel_angle;
                                low_speed.stall_pulse_count = count;
                            }
                        }
                        else
                        {
                            // Normal chopper operation: keep the learned
                            // breakaway margin, reset only the stall window.
                            low_speed.stall_since_us = 0;
                        }

                        if (!low_speed.active)
                        {
                            m_pids[i].clear();
                            m_integral_duty[i] = 0.0f;
                            m_kick_state[i] = KickState();
                            u = 0.0f;
                        }
                        else
                        {
                            if (new_cycle)
                            {
                                // Carry sub-tick drive time into the next cycle instead of
                                // rounding every cycle to a whole control interval.
                                if (low_speed.starting || low_speed.ton_ticks >= period_ticks)
                                    low_speed.drive_ticks = LS_CHOPPER_PERIOD_TICKS;
                                else
                                {
                                    const float ticks =
                                      low_speed.ton_ticks + low_speed.drive_tick_remainder;
                                    low_speed.drive_ticks = static_cast<uint8_t>(ticks);
                                    low_speed.drive_tick_remainder = ticks - low_speed.drive_ticks;
                                }
                            }
                            const uint32_t phase_tick = phase_us / CONTROL_PERIOD_US;
                            const bool driving =
                              parameters.pulse_interleaved[branch]
                                ? (phase_tick * low_speed.drive_ticks) % LS_CHOPPER_PERIOD_TICKS <
                                    low_speed.drive_ticks
                                : phase_tick < low_speed.drive_ticks;
                            u = coasting ? 0.0f : direction * (driving ? drive_duty : off_duty);
                        }
                    }
                }

                // Position-window ripple summary, once per second.
                if (report_ripple)
                {
                    if (m_rip_windows[i] > 0)
                    {
                        const RippleReport report = {std::fabs(target_rad_s) * RAD_S_TO_RPM,
                                                     m_rip_last_mean[i] * RAD_S_TO_RPM,
                                                     m_rip_min[i] * RAD_S_TO_RPM,
                                                     m_rip_max[i] * RAD_S_TO_RPM,
                                                     low_speed.last_window_delta_rad,
                                                     low_speed.last_window_s,
                                                     low_speed.ton_ticks,
                                                     low_speed.last_ff_ticks,
                                                     low_speed.last_error_rpm,
                                                     low_speed.integral_ticks,
                                                     low_speed.boost_duty};
                        portENTER_CRITICAL(&m_diagnostic_mux);
                        m_ripple_reports[i] = report;
                        portEXIT_CRITICAL(&m_diagnostic_mux);
                    }
                    m_rip_min[i] = 1e9f;
                    m_rip_max[i] = -1e9f;
                    m_rip_windows[i] = 0;
                }
            }
            else if (low_speed_blocked)
            {
                // The pulse mode cannot run (angle feedback invalid) and the
                // continuous law has no usable operating point below the gate:
                // hold the wheel still instead of inheriting the breakaway
                // floor at ~66 RPM.
                m_pids[i].clear();
                m_integral_duty[i] = 0.0f;
                m_kick_state[i] = KickState();
                u = 0.0f;
                if (now_ms - m_ls_blocked_warn_ms[i] >= 5000)
                {
                    m_ls_blocked_warn_ms[i] = now_ms;
                    ESP_LOGW(TAG, "M%d low-speed drive blocked.", i + 1);
                }
            }
            else
            {
                KickState & kick = m_kick_state[i];
                if (legacy_law)
                {
                    // Legacy law (pre-kick firmware parameters): the breakaway duty
                    // is a permanent feedforward floor, with no kick and no integral
                    // action. It keeps the old bench behavior for uncalibrated
                    // wheels instead of the low-speed stick-slip of the new law.
                    if (target_sign != 0 && std::fabs(ff_pwm) < breakaway)
                        ff_pwm = direction * breakaway;
                    kick = KickState();
                    m_integral_duty[i] = 0.0f;
                }
                else
                {
                    // ---- Start-up kick ----
                    // Short pulse to break the wheel free from rest: release as soon
                    // as it starts turning (or after a few ticks) and let the
                    // integral action ramp the sustain duty. The kick is not
                    // repeated for a stalling wheel; only the stall rescue below may
                    // re-arm it. No kick when the model feedforward already covers
                    // the breakaway.
                    if (kick.armed && !kick.active && target_sign != 0 &&
                        std::fabs(measured_rad_s) < KICK_START_RAD_S &&
                        now_ms >= kick.cooldown_until_ms && std::fabs(model_ff) < breakaway)
                    {
                        kick.active = true;
                        kick.armed = false;
                        kick.direction = static_cast<int8_t>(target_sign);
                        kick.start_ms = now_ms;
                        kick.stall_since_ms = 0;
                    }
                    if (kick.active)
                    {
                        const bool release = target_sign == 0 || kick.direction != target_sign ||
                                             std::fabs(measured_rad_s) >= KICK_STOP_RAD_S ||
                                             (now_ms - kick.start_ms) >= KICK_TIMEOUT_MS;
                        if (release)
                        {
                            kick.active = false;
                            kick.cooldown_until_ms = now_ms + KICK_COOLDOWN_MS;
                        }
                        else
                        {
                            // Keep the pulse at the breakaway duty: enough to break
                            // the wheel free, small enough not to over-speed it.
                            ff_pwm = direction * breakaway;
                        }
                    }

                    // ---- Stall rescue ----
                    // A wheel that stays genuinely stuck (well below the breakaway
                    // speed) with a target cannot be restarted by the integral alone
                    // once its duty is below the breakaway: re-arm one more pulse
                    // after a full stall window without a pulse. This is not the old
                    // continuous re-kick; the energy injection stays bounded.
                    if (!kick.active)
                    {
                        if (target_sign != 0 && std::fabs(measured_rad_s) < KICK_STALL_RAD_S)
                        {
                            if (kick.stall_since_ms == 0)
                                kick.stall_since_ms = now_ms;
                            else if (!kick.armed &&
                                     (now_ms - kick.stall_since_ms) >= KICK_STALL_RESCUE_MS)
                            {
                                kick.armed = true;
                                kick.stall_since_ms = 0;
                            }
                        }
                        else
                        {
                            kick.stall_since_ms = 0;
                        }
                    }
                }
                ff_pwm = std::clamp(ff_pwm, -1023.0f, 1023.0f);

                // ---- Convert to FastPID input (scaled ×10 for resolution) ----
                int16_t target_pid = static_cast<int16_t>(target_rad_s * 10.0f);
                int16_t measured_pid = static_cast<int16_t>(measured_rad_s * 10.0f);

                // ---- Dynamic output range for anti-windup (P and D stages) ----
                int16_t ff_scaled = static_cast<int16_t>(ff_pwm * 10.0f);
                int16_t range_min = static_cast<int16_t>(-10230 - ff_scaled);
                int16_t range_max = static_cast<int16_t>(10230 - ff_scaled);
                m_pids[i].set_output_range(range_min, range_max);

                // ---- PID step (P and D only; the integral is managed below) ----
                int16_t pid_output = m_pids[i].step(target_pid, measured_pid);

                // ---- Controller-managed integral action ----
                // Integrate only while the actuator is not saturated in the
                // direction of the error (conditional anti-windup). Parameters.ki
                // is the integral gain in duty per rad/s per second. Position
                // moves and uncalibrated wheels use the fallback schedule.
                const float error_rad_s = target_rad_s - measured_rad_s;
                const float dt_s = slow_dt_ms / 1000.0f;
                const float target_magnitude = std::fabs(target_rad_s);
                float integral_scale = 0.0f;
                const bool calibrated_velocity = !position && calibrated_gate > 0.0f;
                if (calibrated_velocity || target_magnitude <= LOW_KI_FULL_RAD_S)
                    integral_scale = 1.0f;
                else if (target_magnitude < LOW_KI_ZERO_RAD_S)
                    integral_scale = (LOW_KI_ZERO_RAD_S - target_magnitude) /
                                     (LOW_KI_ZERO_RAD_S - LOW_KI_FULL_RAD_S);

                // Freeze the integral only while the kick pulse is active: the
                // pulse already supplies the breakaway duty. After the single pulse
                // the integrator is free to ramp up to the true sustain duty and
                // hold the low-speed equilibrium.
                const bool stall_frozen = kick.active;

                u = static_cast<float>(pid_output) / 10.0f + ff_pwm + m_integral_duty[i];
                const bool saturated_high = u >= 1023.0f && error_rad_s > 0.0f;
                const bool saturated_low = u <= -1023.0f && error_rad_s < 0.0f;
                // Correct an underestimated feedforward even when it leaves the
                // wheel stalled far below the requested speed.
                if (!legacy_law && !stall_frozen && !saturated_high && !saturated_low &&
                    wheel.direction == latched_direction && velocity_feedback_available)
                {
                    // Keep the calibrated integral active up to maximum speed
                    // without accumulating a large correction during startup.
                    const float error_limit = 0.1f * target_magnitude;
                    const float integral_error =
                      calibrated_velocity ? std::clamp(error_rad_s, -error_limit, error_limit)
                                          : error_rad_s;
                    m_integral_duty[i] += integral_scale * parameters.ki * integral_error * dt_s;
                    m_integral_duty[i] =
                      std::clamp(m_integral_duty[i], -INTEGRAL_LIMIT_DUTY, INTEGRAL_LIMIT_DUTY);
                }

                // ---- Combine PID + feed-forward + integral ----
                u = std::clamp(static_cast<float>(pid_output) / 10.0f + ff_pwm + m_integral_duty[i],
                               -1023.0f,
                               1023.0f);
            }

            // ---- Slew-rate limit (continuous laws only) ----
            // The pulse tracker writes its bursts directly: the slew limiter
            // would smooth them into a slower ramp.
            if (!pulse_eligible && !low_speed_blocked)
            {
                float max_delta = PWM_SLEW_RATE_PER_MS * slow_dt_ms;
                const float current_signed_pwm =
                  wheel.target_direction_pending
                    ? 0.0f
                    : static_cast<float>(applied_duties[i]) * latched_direction;
                float pwm_diff = u - current_signed_pwm;

                if (pwm_diff > max_delta)
                    u = current_signed_pwm + max_delta;
                else if (pwm_diff < -max_delta)
                    u = current_signed_pwm - max_delta;
            }

            // ---- Write outputs ----
            const bool follow_target_direction =
              target_direction_changed || wheel.target_direction_pending || low_speed.active;
            if (follow_target_direction && target_sign * u < 0.0f)
                u = 0.0f;
            int abs_u = static_cast<int>(std::fabs(u));
            if (abs_u > 1023)
                abs_u = 1023;
            const int8_t requested_direction =
              follow_target_direction ? target_sign : (abs_u == 0 ? latched_direction : signf(u));

            if (requested_direction != wheel.direction && requested_direction != latched_direction)
            {
                m_pids[i].clear();
                m_integral_duty[i] = 0.0f;
                m_kick_state[i] = KickState();
                m_low_speed[i] = LowSpeedState();
            }

            duties[i] = static_cast<uint32_t>(abs_u);
            dirs[i] =
              (requested_direction > 0) ? config::driver::MOTOR_CW : config::driver::MOTOR_CCW;
            wheel.direction = requested_direction;
            wheel.duty = duties[i];
        }

        if (report_ripple)
            m_last_rip_report_us = now_us;
        driver.update_duties(duties, dirs);
        const auto output_duties = driver.get_duty_cycles();
        const auto output_directions = driver.get_directions();
        for (size_t i = 0; i < output_duties.size(); ++i)
        {
            const float duty = static_cast<float>(output_duties[i]);
            m_pid_output_duty[i] = output_directions[i] == config::driver::MOTOR_CW ? duty : -duty;
        }
        xSemaphoreGive(m_mutex);
#if CONFIG_ENCODER_DIAG
        const uint32_t work_us = static_cast<uint32_t>(esp_timer_get_time() - start_us);
        diag_max_work_us = std::max(diag_max_work_us, work_us);
        if (now_us - diag_start_us >= 10000000)
        {
            portENTER_CRITICAL(&m_diagnostic_mux);
            m_timing_report = {
              diag_steps, diag_missed, diag_max_gap_us, diag_max_age_us, diag_max_work_us};
            portEXIT_CRITICAL(&m_diagnostic_mux);
            diag_start_us = now_us;
            diag_steps = 0;
            diag_missed = 0;
            diag_max_gap_us = 0;
            diag_max_age_us = 0;
            diag_max_work_us = 0;
        }
#endif
    }
}
