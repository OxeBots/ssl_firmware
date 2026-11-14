#include "WheelController.h"

#include <esp_log.h>
#include <esp_timer.h>

#include <algorithm>
#include <cmath>
#include <string>

#include "BL48250.h"

static const char * TAG = "WheelController";

// ============================================================================
// SIGN HELPER
// ============================================================================

static inline int signf(float x)
{
    return (x > 0.0f) - (x < 0.0f);
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
  m_target_velocity_rad_s({0, 0, 0, 0}),
  m_current_velocity_rad_s({0, 0, 0, 0}),
  m_pid_output_duty({0, 0, 0, 0}),
  m_position_angle_rad({0, 0, 0, 0}),
  m_last_raw_angle_rad({0, 0, 0, 0}),
  m_raw_angle_seeded(false),
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
        // FastPID: kp, ki, kd, update rate (Hz), 16-bit output, signed
        m_pids[i].configure(
          m_parameters[i].kp, m_parameters[i].ki, m_parameters[i].kd, PID_HZ, 16, true);
        m_pids[i].set_output_range(-10230, 10230);  // ±1023 scaled ×10
    }

    xTaskCreatePinnedToCore(task_wrapper, "wheel_ctrl", 4096, this, 6, &m_task_handle, 1);
    m_initialized = true;

    for (int i = 0; i < 4; ++i)
    {
        ESP_LOGI(TAG,
                 "Motor %d: Kp=%.3f Ki=%.3f Kd=%.3f ff=%.1f vff=%.3f",
                 i,
                 static_cast<double>(m_parameters[i].kp),
                 static_cast<double>(m_parameters[i].ki),
                 static_cast<double>(m_parameters[i].kd),
                 static_cast<double>(m_parameters[i].feedforward),
                 static_cast<double>(m_parameters[i].velocity_ff));
    }
    return ESP_OK;
}

void WheelController::deinit()
{
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
    for (int i = 0; i < 4; ++i)
    {
        m_target_velocity_rad_s[i] = std::clamp(
          velocities[i], -config::driver::WHEEL_MAX_VEL_RAD, config::driver::WHEEL_MAX_VEL_RAD);
    }
    xSemaphoreGive(m_mutex);
}

std::array<float, 4> WheelController::get_target_velocities() const
{
    return m_target_velocity_rad_s;
}

std::array<float, 4> WheelController::get_current_velocities() const
{
    return m_current_velocity_rad_s;
}

std::array<float, 4> WheelController::get_pid_outputs() const
{
    return m_pid_output_duty;
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
    if (!enabled)
        m_last_control_us = static_cast<uint32_t>(esp_timer_get_time());
    xSemaphoreGive(m_mutex);

    if (enabled)
    {
        // Stop the motors before the caller drives them.
        BL48250::get_instance().set_duties({0, 0, 0, 0},
                                           {config::driver::MOTOR_CW,
                                            config::driver::MOTOR_CW,
                                            config::driver::MOTOR_CW,
                                            config::driver::MOTOR_CW});
    }
}

bool WheelController::is_open_loop() const
{
    return m_open_loop;
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
    xSemaphoreGive(m_mutex);
}

void WheelController::cancel_position_moves_locked()
{
    m_position_active_mask = 0;
    m_position_done_mask = 0;
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
    return m_parameters;
}

/**
 * @brief Applies one wheel's control parameters to the running PID.
 */
void WheelController::apply_parameters(int wheel, const Parameters & parameters)
{
    if (!m_mutex || wheel < 0 || wheel >= 4)
        return;
    Parameters clamped = parameters;
    clamped.kp = std::clamp(clamped.kp, 0.0f, 200.0f);
    clamped.ki = std::clamp(clamped.ki, 0.0f, 4000.0f);
    clamped.kd = std::clamp(clamped.kd, 0.0f, 0.5f);
    clamped.feedforward = std::clamp(clamped.feedforward, -600.0f, 600.0f);
    clamped.velocity_ff = std::clamp(clamped.velocity_ff, 0.0f, 400.0f);
    clamped.static_friction = std::clamp(clamped.static_friction, 0.0f, 600.0f);
    clamped.angle_sign = (clamped.angle_sign < 0.0f) ? -1.0f : 1.0f;
    // FastPID rejects a coefficient that rounds to zero and then zeroes every
    // gain, so drop values below its fixed-point resolution.
    if (clamped.kp > 0.0f && clamped.kp < 1.0f / 256.0f)
        clamped.kp = 0.0f;
    if (clamped.ki > 0.0f && clamped.ki < 0.390625f)
        clamped.ki = 0.0f;
    if (clamped.kd > 0.0f && clamped.kd < 1.0f / (100.0f * 256.0f))
        clamped.kd = 0.0f;

    xSemaphoreTake(m_mutex, portMAX_DELAY);
    m_parameters[wheel] = clamped;
    m_pids[wheel].configure(clamped.kp, clamped.ki, clamped.kd, PID_HZ, 16, true);
    m_pids[wheel].set_output_range(-10230, 10230);
    m_pids[wheel].clear();
    xSemaphoreGive(m_mutex);
}

void WheelController::save_parameters(int wheel)
{
    if (wheel < 0 || wheel >= 4)
        return;
    const Parameters parameters = m_parameters[wheel];
    const std::string key_base = MOTOR_KEYS[wheel];
    const std::string kp_key = key_base + PID_SUFFIXES[0];
    const std::string ki_key = key_base + PID_SUFFIXES[1];
    const std::string kd_key = key_base + PID_SUFFIXES[2];
    const std::string ff_key = key_base + FEEDFORWARD_SUFFIX;
    const std::string vff_key = key_base + VELOCITY_FF_SUFFIX;
    const std::string sf_key = key_base + STATIC_FRICTION_SUFFIX;
    const std::string asign_key = key_base + ANGLE_SIGN_SUFFIX;

    NVSManager::save_blob("pid", kp_key.c_str(), &parameters.kp, sizeof(parameters.kp));
    NVSManager::save_blob("pid", ki_key.c_str(), &parameters.ki, sizeof(parameters.ki));
    NVSManager::save_blob("pid", kd_key.c_str(), &parameters.kd, sizeof(parameters.kd));
    NVSManager::save_blob(
      "pid", ff_key.c_str(), &parameters.feedforward, sizeof(parameters.feedforward));
    NVSManager::save_blob(
      "pid", vff_key.c_str(), &parameters.velocity_ff, sizeof(parameters.velocity_ff));
    NVSManager::save_blob(
      "pid", sf_key.c_str(), &parameters.static_friction, sizeof(parameters.static_friction));
    NVSManager::save_blob(
      "pid", asign_key.c_str(), &parameters.angle_sign, sizeof(parameters.angle_sign));
    ESP_LOGI(TAG,
             "Saved %s: Kp=%.3f Ki=%.3f Kd=%.4f ff=%.1f vff=%.3f",
             MOTOR_KEYS[wheel],
             static_cast<double>(parameters.kp),
             static_cast<double>(parameters.ki),
             static_cast<double>(parameters.kd),
             static_cast<double>(parameters.feedforward),
             static_cast<double>(parameters.velocity_ff));
}

void WheelController::save_all_parameters()
{
    for (int i = 0; i < 4; ++i) save_parameters(i);
}

// ============================================================================
// NVS LOAD
// ============================================================================

/**
 * @brief Loads the per-wheel control parameters, falling back to the defaults.
 */
void WheelController::load_parameters_from_nvs()
{
    for (int wheel = 0; wheel < 4; ++wheel)
    {
        Parameters parameters;
        float * gains[3] = {&parameters.kp, &parameters.ki, &parameters.kd};
        for (int g = 0; g < 3; ++g)
        {
            const std::string key = std::string(MOTOR_KEYS[wheel]) + PID_SUFFIXES[g];
            size_t length = sizeof(float);
            if (NVSManager::load_blob("pid", key.c_str(), gains[g], &length) != ESP_OK)
                *gains[g] = PID_DEFAULT_GAINS[g];
        }

        const std::string ff_key = std::string(MOTOR_KEYS[wheel]) + FEEDFORWARD_SUFFIX;
        size_t ff_length = sizeof(float);
        if (NVSManager::load_blob("pid", ff_key.c_str(), &parameters.feedforward, &ff_length) !=
            ESP_OK)
            parameters.feedforward = STATIC_FRICTION_THRESHOLD;

        const std::string vff_key = std::string(MOTOR_KEYS[wheel]) + VELOCITY_FF_SUFFIX;
        size_t vff_length = sizeof(float);
        if (NVSManager::load_blob("pid", vff_key.c_str(), &parameters.velocity_ff, &vff_length) !=
            ESP_OK)
            parameters.velocity_ff = 0.0f;

        const std::string sf_key = std::string(MOTOR_KEYS[wheel]) + STATIC_FRICTION_SUFFIX;
        size_t sf_length = sizeof(float);
        if (NVSManager::load_blob("pid", sf_key.c_str(), &parameters.static_friction, &sf_length) !=
            ESP_OK)
            parameters.static_friction = STATIC_FRICTION_THRESHOLD;

        const std::string asign_key = std::string(MOTOR_KEYS[wheel]) + ANGLE_SIGN_SUFFIX;
        size_t asign_length = sizeof(float);
        if (NVSManager::load_blob(
              "pid", asign_key.c_str(), &parameters.angle_sign, &asign_length) != ESP_OK)
            parameters.angle_sign = -1.0f;

        m_parameters[wheel] = parameters;
    }
}

// ============================================================================
// CONTROL TASK
// ============================================================================

void WheelController::task_wrapper(void * arg)
{
    static_cast<WheelController *>(arg)->control_task();
}

/**
 * @brief Computes one wheel's velocity command while position mode is active.
 */
void WheelController::control_task()
{
    m_last_control_us = static_cast<uint32_t>(esp_timer_get_time());

    while (true)
    {
        // Yield to FreeRTOS for 1 tick (10 ms at 100 Hz tick rate).
        // Must NOT use esp_rom_delay_us — it disables interrupts and
        // starves the IDLE task, triggering the interrupt watchdog.
        vTaskDelay(1);

        uint32_t now_us = static_cast<uint32_t>(esp_timer_get_time());
        float dt_ms = static_cast<float>(now_us - m_last_control_us) / 1000.0f;
        if (dt_ms < 0.001f)
            dt_ms = 0.001f;
        m_last_control_us = now_us;

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

        // Read the filtered angular velocity (encoder frame) and the raw
        // encoder angles before taking the controller mutex.
        const std::array<float, 4> current_rad_s =
          WheelStateEstimator::get_instance().get_smoothed_velocity_rad_s();
        const std::array<float, 4> raw_angles =
          WheelStateEstimator::get_instance().get_measured_angle_rad();

        std::array<uint32_t, 4> duties;
        std::array<uint8_t, 4> dirs;

        xSemaphoreTake(m_mutex, portMAX_DELAY);

        // Open-loop mode owns the motor outputs: never write them from the PID.
        if (m_open_loop)
        {
            xSemaphoreGive(m_mutex);
            continue;
        }

        for (int i = 0; i < 4; ++i)
        {
            // Negate because the estimator reports the encoder frame, where CW
            // rotation is negative; targets and the PID use the motor frame.
            float measured_rad_s = -current_rad_s[i];
            m_current_velocity_rad_s[i] = measured_rad_s;

            // Accumulate the continuous position from the raw encoder deltas.
            // The raw angle decreases for positive motor rotation.
            if (m_raw_angle_seeded)
            {
                const float delta =
                  std::remainder(raw_angles[i] - m_last_raw_angle_rad[i], RAW_ANGLE_PERIOD_RAD);
                // angle_sign says whether the raw angle increases (+1) or
                // decreases (-1) for a positive motor rotation.
                m_position_angle_rad[i] += m_parameters[i].angle_sign * delta * RAW_ANGLE_TO_WHEEL;
            }
            m_last_raw_angle_rad[i] = raw_angles[i];

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

            // ---- Zero velocity target override ----
            if (target_rad_s == 0.0f && std::fabs(measured_rad_s) < ZERO_VELOCITY_THRESHOLD_RAD_S)
            {
                duties[i] = 0;
                dirs[i] = config::driver::MOTOR_CW;
                m_pid_output_duty[i] = 0.0f;
                m_pids[i].clear();
                m_wheel_state[i].duty = 0;
                m_wheel_state[i].direction = 1;
                continue;
            }

            const Parameters & parameters = m_parameters[i];

            // ---- Convert to FastPID input (scaled ×10 for resolution) ----
            int16_t target_pid = static_cast<int16_t>(target_rad_s * 10.0f);
            int16_t measured_pid = static_cast<int16_t>(measured_rad_s * 10.0f);

            // ---- Feed-forward: static friction with the target sign plus the
            // identified velocity gain. ----
            const float direction = static_cast<float>(signf(target_rad_s));
            // The fitted static curve is only valid around the operating point;
            // near zero speed the wheel still needs the breakaway duty to move.
            float ff_pwm = parameters.feedforward + parameters.velocity_ff * target_rad_s;
            if (target_rad_s != 0.0f && std::fabs(ff_pwm) < parameters.static_friction)
                ff_pwm = direction * parameters.static_friction;

            // No startup boost: the static-friction minimum of the feedforward
            // already breaks the wheel free. Adding a boost pulse on top of the
            // velocity feedforward saturated the first duty step and made every
            // validated step overshoot.
            ff_pwm = std::clamp(ff_pwm, -1023.0f, 1023.0f);

            // ---- Dynamic output range for anti-windup ----
            int16_t ff_scaled = static_cast<int16_t>(ff_pwm * 10.0f);
            int16_t range_min = static_cast<int16_t>(-10230 - ff_scaled);
            int16_t range_max = static_cast<int16_t>(10230 - ff_scaled);
            m_pids[i].set_output_range(range_min, range_max);

            // ---- PID step ----
            int16_t pid_output = m_pids[i].step(target_pid, measured_pid);

            // ---- Combine PID + feed-forward ----
            int32_t u_scaled = static_cast<int32_t>(pid_output) + static_cast<int32_t>(ff_scaled);

            // ---- Clamp ----
            if (u_scaled > 10230)
                u_scaled = 10230;
            else if (u_scaled < -10230)
                u_scaled = -10230;

            float u = static_cast<float>(u_scaled) / 10.0f;

            // ---- Slew-rate limit ----
            float max_delta = PWM_SLEW_RATE_PER_MS * dt_ms;
            float current_signed_pwm = static_cast<float>(m_wheel_state[i].duty) *
                                       static_cast<float>(m_wheel_state[i].direction);
            float pwm_diff = u - current_signed_pwm;

            if (pwm_diff > max_delta)
                u = current_signed_pwm + max_delta;
            else if (pwm_diff < -max_delta)
                u = current_signed_pwm - max_delta;

            // ---- Write outputs ----
            int abs_u = static_cast<int>(std::fabs(u));
            if (abs_u > 1023)
                abs_u = 1023;
            duties[i] = static_cast<uint32_t>(abs_u);
            dirs[i] = (u >= 0.0f) ? config::driver::MOTOR_CW : config::driver::MOTOR_CCW;

            m_pid_output_duty[i] = u;
            m_wheel_state[i].duty = duties[i];
            m_wheel_state[i].direction = (u >= 0.0f) ? 1 : -1;
        }

        xSemaphoreGive(m_mutex);
        m_raw_angle_seeded = true;

        BL48250::get_instance().set_duties(duties, dirs);
    }
}
