#include "WheelController.h"

#include <esp_log.h>
#include <esp_timer.h>

#include <algorithm>
#include <cmath>

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
  m_tuning_task_handle(nullptr),
  m_initialized(false),
  m_tuning_mode(false),
  m_tuning_complete(false),
  m_target_velocity_rad_s({0, 0, 0, 0}),
  m_current_velocity_rad_s({0, 0, 0, 0}),
  m_pid_output_duty({0, 0, 0, 0}),
  m_kp({PID_DEFAULT_GAINS[0], PID_DEFAULT_GAINS[0], PID_DEFAULT_GAINS[0], PID_DEFAULT_GAINS[0]}),
  m_ki({PID_DEFAULT_GAINS[1], PID_DEFAULT_GAINS[1], PID_DEFAULT_GAINS[1], PID_DEFAULT_GAINS[1]}),
  m_kd({PID_DEFAULT_GAINS[2], PID_DEFAULT_GAINS[2], PID_DEFAULT_GAINS[2], PID_DEFAULT_GAINS[2]}),
  m_tuning_done_cb(nullptr),
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

    load_pid_from_nvs();

    for (int i = 0; i < 4; ++i)
    {
        // FastPID: kp, ki, kd, update rate (Hz), 16-bit output, signed
        m_pids[i].configure(m_kp[i], m_ki[i], m_kd[i], PID_HZ, 16, true);
        m_pids[i].setOutputRange(-10230, 10230);  // ±1023 scaled ×10
    }

    xTaskCreatePinnedToCore(task_wrapper, "wheel_ctrl", 4096, this, 6, &m_task_handle, 1);
    m_initialized = true;

    ESP_LOGI(TAG,
             "WheelController initialized. FL Kp=%.3f Ki=%.3f Kd=%.3f | "
             "BL Kp=%.3f Ki=%.3f Kd=%.3f | "
             "BR Kp=%.3f Ki=%.3f Kd=%.3f | "
             "FR Kp=%.3f Ki=%.3f Kd=%.3f",
             m_kp[0],
             m_ki[0],
             m_kd[0],
             m_kp[1],
             m_ki[1],
             m_kd[1],
             m_kp[2],
             m_ki[2],
             m_kd[2],
             m_kp[3],
             m_ki[3],
             m_kd[3]);
    return ESP_OK;
}

void WheelController::deinit()
{
    if (m_tuning_task_handle)
    {
        vTaskDelete(m_tuning_task_handle);
        m_tuning_task_handle = nullptr;
    }

    if (m_task_handle)
    {
        vTaskDelete(m_task_handle);
        m_task_handle = nullptr;
        ESP_LOGI(TAG, "Control task deleted.");
    }

    m_initialized = false;
    m_tuning_mode = false;
    m_tuning_complete = false;

    m_target_velocity_rad_s.fill(0.0f);
    m_current_velocity_rad_s.fill(0.0f);
    m_pid_output_duty.fill(0.0f);

    ESP_LOGI(TAG, "WheelController deinitialized.");
}

// ============================================================================
// PUBLIC API — Velocity targets
// ============================================================================

void WheelController::set_target_velocities(const std::array<float, 4> & velocities)
{
    if (!m_mutex)
        return;
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    for (int i = 0; i < 4; ++i)
    {
        m_target_velocity_rad_s[i] = std::clamp(
          velocities[i], -config::driver::BL48250_MAX_VEL_RAD, config::driver::BL48250_MAX_VEL_RAD);
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
// PUBLIC API — PID tunings
// ============================================================================

void WheelController::set_pid_tunings(const std::array<float, 4> & kp,
                                      const std::array<float, 4> & ki,
                                      const std::array<float, 4> & kd)
{
    if (!m_mutex)
        return;
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
    for (int i = 0; i < 4; ++i)
    {
        m_pids[i].configure(m_kp[i], m_ki[i], m_kd[i], PID_HZ, 16, true);
        m_pids[i].setOutputRange(-10230, 10230);
        m_pids[i].clear();
    }
    xSemaphoreGive(m_mutex);
}

// ============================================================================
// PUBLIC API — NVS persistence
// ============================================================================

void WheelController::save_pid_to_nvs(int motor_idx)
{
    std::string kp_key = std::string(MOTOR_KEYS[motor_idx]) + PID_SUFFIXES[0];
    std::string ki_key = std::string(MOTOR_KEYS[motor_idx]) + PID_SUFFIXES[1];
    std::string kd_key = std::string(MOTOR_KEYS[motor_idx]) + PID_SUFFIXES[2];

    NVSManager::save_blob("pid", kp_key.c_str(), &m_kp[motor_idx], sizeof(m_kp[motor_idx]));
    NVSManager::save_blob("pid", ki_key.c_str(), &m_ki[motor_idx], sizeof(m_ki[motor_idx]));
    NVSManager::save_blob("pid", kd_key.c_str(), &m_kd[motor_idx], sizeof(m_kd[motor_idx]));
    ESP_LOGI(TAG,
             "Saved PID for motor %d (%s): Kp=%.3f Ki=%.3f Kd=%.3f",
             motor_idx,
             MOTOR_KEYS[motor_idx],
             m_kp[motor_idx],
             m_ki[motor_idx],
             m_kd[motor_idx]);
}

void WheelController::save_all_pid_to_nvs()
{
    for (int i = 0; i < 4; ++i) save_pid_to_nvs(i);
}

// ============================================================================
// NVS LOAD
// ============================================================================

void WheelController::load_pid_from_nvs()
{
    float * gains[3] = {m_kp.data(), m_ki.data(), m_kd.data()};

    for (int m = 0; m < 4; ++m)
    {
        for (int g = 0; g < 3; ++g)
        {
            std::string key = std::string(MOTOR_KEYS[m]) + PID_SUFFIXES[g];
            size_t len = sizeof(float);
            if (NVSManager::load_blob("pid", key.c_str(), &gains[g][m], &len) != ESP_OK)
                gains[g][m] = PID_DEFAULT_GAINS[g];
        }
    }

    ESP_LOGI(TAG,
             "Loaded per-motor PID from NVS: "
             "FL Kp=%.3f Ki=%.3f Kd=%.3f | BL Kp=%.3f Ki=%.3f Kd=%.3f | "
             "BR Kp=%.3f Ki=%.3f Kd=%.3f | FR Kp=%.3f Ki=%.3f Kd=%.3f",
             m_kp[0],
             m_ki[0],
             m_kd[0],
             m_kp[1],
             m_ki[1],
             m_kd[1],
             m_kp[2],
             m_ki[2],
             m_kd[2],
             m_kp[3],
             m_ki[3],
             m_kd[3]);
}

// ============================================================================
// CONTROL TASK
// ============================================================================

void WheelController::task_wrapper(void * arg)
{
    static_cast<WheelController *>(arg)->control_task();
}

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
        {
            vTaskDelay(1);
            continue;
        }

        // Skip PID control while tuning is active.
        bool tuning = false;
        xSemaphoreTake(m_mutex, 0);
        tuning = m_tuning_mode;
        xSemaphoreGive(m_mutex);
        if (tuning)
            continue;

        // Read filtered angular velocity in rad/s directly.
        std::array<float, 4> current_rad_s =
          WheelStateEstimator::get_instance().get_filtered_velocity_rad_s();

        std::array<uint32_t, 4> duties;
        std::array<uint8_t, 4> dirs;

        xSemaphoreTake(m_mutex, portMAX_DELAY);

        for (int i = 0; i < 4; ++i)
        {
            // Negate because AS5600 encoder reads CW rotation as negative velocity.
            float measured_rad_s = -current_rad_s[i];
            m_current_velocity_rad_s[i] = measured_rad_s;

            float target_rad_s = m_target_velocity_rad_s[i];

            // ---- Zero velocity target override ----
            if (target_rad_s == 0.0f && fabsf(measured_rad_s) < ZERO_VELOCITY_THRESHOLD_RAD_S)
            {
                duties[i] = 0;
                dirs[i] = config::driver::MOTOR_CW;
                m_pid_output_duty[i] = 0.0f;
                m_pids[i].clear();
                m_wheel_state[i].duty = 0;
                m_wheel_state[i].direction = 1;
                m_wheel_state[i].boost_until_us = 0;
                continue;
            }

            // ---- Convert to FastPID input (scaled ×10 for resolution) ----
            int16_t target_pid = static_cast<int16_t>(target_rad_s * 10.0f);
            int16_t measured_pid = static_cast<int16_t>(measured_rad_s * 10.0f);

            // ---- Feed-forward ----
            float ff_pwm = STATIC_FRICTION_THRESHOLD * static_cast<float>(signf(target_rad_s));

            // ---- Startup boost ----
            bool is_stuck = fabsf(measured_rad_s) < STUCK_VELOCITY_THRESHOLD_RAD_S;
            bool was_stuck =
              fabsf(m_wheel_state[i].last_measured_velocity_rad_s) < STUCK_VELOCITY_THRESHOLD_RAD_S;

            if (is_stuck && was_stuck && fabsf(target_rad_s) > 0.5f)
                m_wheel_state[i].boost_until_us = now_us + BOOST_DURATION_US;
            else if (!is_stuck)
                m_wheel_state[i].boost_until_us = 0;

            m_wheel_state[i].last_measured_velocity_rad_s = measured_rad_s;

            float ff_total = ff_pwm;
            if (now_us < m_wheel_state[i].boost_until_us)
                ff_total = static_cast<float>(signf(target_rad_s)) *
                           (STATIC_FRICTION_THRESHOLD * BOOST_MULTIPLIER);

            // ---- Dynamic output range for anti-windup ----
            int16_t ff_scaled = static_cast<int16_t>(ff_total * 10.0f);
            m_pids[i].setOutputRange(-10230 - ff_scaled, 10230 - ff_scaled);

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
            int abs_u = static_cast<int>(fabsf(u));
            if (abs_u > 1023)
                abs_u = 1023;
            duties[i] = static_cast<uint32_t>(abs_u);
            dirs[i] = (u >= 0.0f) ? config::driver::MOTOR_CW : config::driver::MOTOR_CCW;

            m_pid_output_duty[i] = u;
            m_wheel_state[i].duty = duties[i];
            m_wheel_state[i].direction = (u >= 0.0f) ? 1 : -1;
        }

        xSemaphoreGive(m_mutex);

        BL48250::get_instance().set_duties(duties, dirs);
    }
}

// ============================================================================
// RELAY AUTO-TUNING
// ============================================================================

void WheelController::set_tuning_done_callback(TuningDoneCallback cb)
{
    m_tuning_done_cb = cb;
}

void WheelController::start_relay_tuning()
{
    if (m_tuning_mode || m_tuning_task_handle)
        return;

    if (m_mutex)
        xSemaphoreTake(m_mutex, portMAX_DELAY);
    m_tuning_mode = true;
    m_tuning_complete = false;
    if (m_mutex)
        xSemaphoreGive(m_mutex);

    // Pin tuning task to Core 0 (same as control task) to avoid starving
    // Core 1's ADC/encoder ISR which runs at priority 15.
    xTaskCreatePinnedToCore(
      tuning_task_wrapper, "relay_tune", 8192, this, 4, &m_tuning_task_handle, 0);
    ESP_LOGI(TAG, "Relay tuning task started.");
}

bool WheelController::is_tuning_active() const
{
    return m_tuning_mode;
}

bool WheelController::is_tuning_complete() const
{
    return m_tuning_complete;
}

std::array<bool, 4> WheelController::get_tuning_success() const
{
    std::array<bool, 4> ok = {false, false, false, false};
    for (int i = 0; i < 4; ++i) ok[i] = m_tuners[i].is_success();
    return ok;
}

void WheelController::tuning_task_wrapper(void * arg)
{
    static_cast<WheelController *>(arg)->tuning_task();
}

void WheelController::tuning_task()
{
    static const char * motor_names[4] = {"FL", "BL", "BR", "FR"};
    ESP_LOGI(TAG, "Relay auto-tuning started on all 4 motors.");

    // Phase 1: apply bias (u0) and measure steady-state velocity as setpoint.
    // The relay will oscillate around this point with output = u0 ± relay_amp.
    constexpr float BIAS_DUTY = RELAY_AMPLITUDE * 0.5f;   // 100
    constexpr float RELAY_STEP = RELAY_AMPLITUDE * 0.5f;  // ±100 → swings 0-200

    std::array<float, 4> setpoints_rad_s = {0.0f, 0.0f, 0.0f, 0.0f};
    {
        std::array<uint32_t, 4> duties = {static_cast<uint32_t>(BIAS_DUTY),
                                          static_cast<uint32_t>(BIAS_DUTY),
                                          static_cast<uint32_t>(BIAS_DUTY),
                                          static_cast<uint32_t>(BIAS_DUTY)};

        std::array<uint8_t, 4> dirs = {config::driver::MOTOR_CW,
                                       config::driver::MOTOR_CW,
                                       config::driver::MOTOR_CW,
                                       config::driver::MOTOR_CW};
        BL48250::get_instance().set_duties(duties, dirs);

        // Settle at bias for 1 s so velocity stabilizes.
        uint64_t t0 = esp_timer_get_time();
        while ((esp_timer_get_time() - t0) < 1000000)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        auto velocities = WheelStateEstimator::get_instance().get_filtered_velocity_rad_s();
        for (int i = 0; i < 4; ++i)
        {
            setpoints_rad_s[i] = -velocities[i];  // negate encoder direction
            ESP_LOGI(TAG, "Motor %s bias velocity: %.2f rad/s", motor_names[i], setpoints_rad_s[i]);
        }
    }

    // Phase 2: start relay tuners — keep motors at bias so SETTLING output (u0)
    // matches the velocity the setpoint was measured at.
    for (int i = 0; i < 4; ++i)
    {
        if (std::fabs(setpoints_rad_s[i]) < 1.0f)
        {
            ESP_LOGW(TAG,
                     "Motor %s: bias velocity too low (%.2f). Marking failed.",
                     motor_names[i],
                     setpoints_rad_s[i]);
            continue;
        }
        m_tuners[i].start(
          setpoints_rad_s[i], RELAY_STEP, BIAS_DUTY, 1.0f, TUNE_MAX_MS, TUNE_MIN_PERIODS);
    }

    // Phase 3: relay loop at 100 Hz.
    std::array<float, 4> outputs = {0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, 4> prev_outputs = {0.0f, 0.0f, 0.0f, 0.0f};
    bool all_done = false;
    uint32_t loop_count = 0;

    while (!all_done)
    {
        vTaskDelay(1);  // 1 tick = 10 ms at 100 Hz tick rate

        auto velocities = WheelStateEstimator::get_instance().get_filtered_velocity_rad_s();
        all_done = true;
        int active = 0;

        for (int i = 0; i < 4; ++i)
        {
            float vel = -velocities[i];  // negate encoder direction
            RelayFeedbackTuner::State st = m_tuners[i].get_state();
            if (st == RelayFeedbackTuner::State::SETTLING ||
                st == RelayFeedbackTuner::State::OSCILLATING)
            {
                float out = 0.0f;
                m_tuners[i].update(vel, out);
                outputs[i] = out;
                active++;
                all_done = false;
            }
            else
            {
                outputs[i] = 0.0f;
            }
        }

        // Only update hardware when outputs actually changed.
        bool changed = false;
        for (int i = 0; i < 4; ++i)
            if (outputs[i] != prev_outputs[i])
                changed = true;

        if (changed)
        {
            std::array<uint32_t, 4> duties = {0, 0, 0, 0};
            std::array<uint8_t, 4> dirs = {0, 0, 0, 0};
            for (int i = 0; i < 4; ++i)
            {
                if (outputs[i] >= 0.0f)
                {
                    dirs[i] = config::driver::MOTOR_CW;
                    duties[i] = static_cast<uint32_t>(outputs[i]);
                }
                else
                {
                    dirs[i] = config::driver::MOTOR_CCW;
                    duties[i] = static_cast<uint32_t>(-outputs[i]);
                }
            }
            BL48250::get_instance().set_duties(duties, dirs);
            prev_outputs = outputs;
        }

        loop_count++;
        if (loop_count % 250 == 0)  // ~2.5 s
        {
            ESP_LOGI(TAG, "Tuning loop %lu, active=%d", (unsigned long)loop_count, active);
        }
    }

    // Stop all motors
    BL48250::get_instance().set_duties({0, 0, 0, 0}, {0, 0, 0, 0});

    // Phase 4: apply tunings
    std::array<float, 4> tuned_kp = m_kp;
    std::array<float, 4> tuned_ki = m_ki;
    std::array<float, 4> tuned_kd = m_kd;
    bool all_success = true;

    for (int i = 0; i < 4; ++i)
    {
        if (m_tuners[i].is_success())
        {
            float kp, ki, kd;
            m_tuners[i].get_tunings(kp, ki, kd);
            tuned_kp[i] = kp;
            tuned_ki[i] = ki;
            tuned_kd[i] = kd;
            m_kp[i] = kp;
            m_ki[i] = ki;
            m_kd[i] = kd;
            save_pid_to_nvs(i);
            if (m_mutex)
                xSemaphoreTake(m_mutex, portMAX_DELAY);
            m_pids[i].configure(kp, ki, kd, PID_HZ, 16, true);
            m_pids[i].setOutputRange(-10230, 10230);
            m_pids[i].clear();
            if (m_mutex)
                xSemaphoreGive(m_mutex);
            ESP_LOGI(TAG,
                     "Motor %s tuned: Kp=%.3f Ki=%.3f Kd=%.3f Ku=%.3f Tu=%.3fs a=%.3f",
                     motor_names[i],
                     kp,
                     ki,
                     kd,
                     m_tuners[i].get_Ku(),
                     m_tuners[i].get_Tu(),
                     m_tuners[i].get_amplitude());
        }
        else
        {
            all_success = false;
            ESP_LOGW(TAG, "Motor %s tuning failed.", motor_names[i]);
        }
    }

    ESP_LOGI(TAG,
             "Relay tuning complete. "
             "FL Kp=%.3f Ki=%.3f Kd=%.3f | BL Kp=%.3f Ki=%.3f Kd=%.3f | "
             "BR Kp=%.3f Ki=%.3f Kd=%.3f | FR Kp=%.3f Ki=%.3f Kd=%.3f",
             tuned_kp[0],
             tuned_ki[0],
             tuned_kd[0],
             tuned_kp[1],
             tuned_ki[1],
             tuned_kd[1],
             tuned_kp[2],
             tuned_ki[2],
             tuned_kd[2],
             tuned_kp[3],
             tuned_ki[3],
             tuned_kd[3]);

    if (m_tuning_done_cb)
        m_tuning_done_cb(all_success);

    if (m_mutex)
        xSemaphoreTake(m_mutex, portMAX_DELAY);
    m_tuning_mode = false;
    m_tuning_complete = true;
    if (m_mutex)
        xSemaphoreGive(m_mutex);

    m_tuning_task_handle = nullptr;
    vTaskDelete(nullptr);
}
