#include "WheelController.h"

#include <esp_log.h>

#include <algorithm>
#include <cmath>

static const char * TAG = "WheelController";

WheelController & WheelController::get_instance()
{
    static WheelController instance;
    return instance;
}

WheelController::WheelController()
: m_driver(nullptr),
  m_task_handle(nullptr),
  m_initialized(false),
  m_tuning_mode(false),
  m_target_velocities({0, 0, 0, 0}),
  m_current_setpoints({0, 0, 0, 0}),
  m_current_velocities({0, 0, 0, 0}),
  m_pid_outputs({0, 0, 0, 0}),
  m_kp(1.0f),
  m_ki(0.5f),
  m_kd(0.01f)
{
}

WheelController::~WheelController()
{
    deinit();
}

/**
 * @brief Initialize the controller: loads PID gains from NVS, configures four
 * QuickPID instances, and launches the 100 Hz control task on Core 1.
 *
 * @param driver Pointer to a live BL48250 instance. Must remain valid
 *               for the lifetime of the controller (or until deinit()).
 * @return ESP_OK on success
 */
esp_err_t WheelController::init(BL48250 * driver)
{
    if (m_initialized)
        return ESP_OK;
    if (!driver)
        return ESP_ERR_INVALID_ARG;

    m_driver = driver;

    load_pid_from_nvs();

    for (int i = 0; i < 4; ++i)
    {
        m_pids[i] = QuickPID(&m_current_velocities[i], &m_pid_outputs[i], &m_current_setpoints[i]);
        m_pids[i].SetTunings(m_kp, m_ki, m_kd);
        m_pids[i].SetMode(QuickPID::Control::automatic);
        m_pids[i].SetSampleTimeUs(10000);  // 100 Hz = 10 ms
        m_pids[i].SetOutputLimits(-1023.0f, 1023.0f);
        m_pids[i].SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
    }

    xTaskCreatePinnedToCore(task_wrapper, "wheel_ctrl", 4096, this, 6, &m_task_handle, 1);
    m_initialized = true;

    ESP_LOGI(TAG, "WheelController initialized. Kp=%.3f Ki=%.3f Kd=%.3f", m_kp, m_ki, m_kd);
    return ESP_OK;
}

/**
 * @brief down the controller: deletes the FreeRTOS control task, nulls the
 * driver pointer, and resets m_initialized so that init() can be called
 * again.
 *
 * Safe to call from test tearDown() — prevents dangling task access after
 * the BL48250 driver is destroyed.
 */
void WheelController::deinit()
{
    if (m_task_handle)
    {
        vTaskDelete(m_task_handle);
        m_task_handle = nullptr;
        ESP_LOGI(TAG, "Control task deleted.");
    }

    m_driver = nullptr;
    m_initialized = false;
    m_tuning_mode = false;

    // Reset setpoints so the next init() starts from a clean state.
    m_target_velocities.fill(0.0f);
    m_current_setpoints.fill(0.0f);
    m_current_velocities.fill(0.0f);
    m_pid_outputs.fill(0.0f);

    ESP_LOGI(TAG, "WheelController deinitialized.");
}

/**
 * @brief Update the target velocity for each wheel (rad/s, clamped to ±MAX_VEL).
 *
 * @param velocities Array of 4 target velocities in rad/s
 */
void WheelController::set_target_velocities(const std::array<float, 4> & velocities)
{
    for (int i = 0; i < 4; ++i)
    {
        m_target_velocities[i] =
          std::clamp(velocities[i], -config::driver::BL48250_MAX_VEL_RAD, config::driver::BL48250_MAX_VEL_RAD);
    }
}

/**
 * @brief Apply PID gains to all four controllers.
 *
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
void WheelController::set_pid_tunings(float kp, float ki, float kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
    for (int i = 0; i < 4; ++i) m_pids[i].SetTunings(m_kp, m_ki, m_kd);
}

/**
 * @brief Load PID gains from NVS (namespace "pid", keys "kp", "ki", "kd").
 *
 * Falls back to default gains (Kp=1.0, Ki=0.5, Kd=0.01) if NVS keys are missing.
 */
void WheelController::load_pid_from_nvs()
{
    size_t len = sizeof(float);
    if (NVSManager::load_blob("pid", "kp", &m_kp, &len) != ESP_OK)
        m_kp = 1.0f;
    len = sizeof(float);
    if (NVSManager::load_blob("pid", "ki", &m_ki, &len) != ESP_OK)
        m_ki = 0.5f;
    len = sizeof(float);
    if (NVSManager::load_blob("pid", "kd", &m_kd, &len) != ESP_OK)
        m_kd = 0.01f;

    ESP_LOGI(TAG, "Loaded PID from NVS: kp=%.3f ki=%.3f kd=%.3f", m_kp, m_ki, m_kd);
}

/**
 * @brief Save PID gains to NVS (namespace "pid", keys "kp", "ki", "kd").
 *
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
void WheelController::save_pid_to_nvs(float kp, float ki, float kd)
{
    NVSManager::save_blob("pid", "kp", &kp, sizeof(kp));
    NVSManager::save_blob("pid", "ki", &ki, sizeof(ki));
    NVSManager::save_blob("pid", "kd", &kd, sizeof(kd));
    ESP_LOGI(TAG, "Saved PID to NVS: kp=%.3f ki=%.3f kd=%.3f", kp, ki, kd);
}

/**
 * @brief FreeRTOS task entry point — casts argument to WheelController and calls control_task().
 *
 * @param arg Pointer to WheelController instance
 */
void WheelController::task_wrapper(void * arg)
{
    static_cast<WheelController *>(arg)->control_task();
}

/**
 * @brief Main control loop — runs PID control at 100Hz for all 4 wheels.
 *
 * Reads filtered RPM from wheel state estimator, converts to rad/s, applies
 * soft-start ramp to setpoints, computes PID output, and sends duty cycles
 * and directions to BL48250 motor driver. Skips control when in tuning mode.
 */
void WheelController::control_task()
{
    TickType_t xLastWake = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(10);  // 100 Hz
    const float dt = 0.01f;

    while (true)
    {
        vTaskDelayUntil(&xLastWake, xPeriod);

        if (m_tuning_mode)
            continue;

        std::array<float, 4> current_rpm = WheelStateEstimator::get_instance().get_filtered_rpm();

        std::array<uint32_t, 4> duties;
        std::array<uint8_t, 4> dirs;

        for (int i = 0; i < 4; ++i)
        {
            // Convert encoder RPM to rad/s for the PID input.
            m_current_velocities[i] = current_rpm[i] * (2.0f * M_PI / 60.0f);

            // Soft-start: ramp the internal setpoint toward the target.
            float diff = m_target_velocities[i] - m_current_setpoints[i];
            float max_change = MAX_ACCEL * dt;

            if (diff > max_change)
                m_current_setpoints[i] += max_change;
            else if (diff < -max_change)
                m_current_setpoints[i] -= max_change;
            else
                m_current_setpoints[i] = m_target_velocities[i];

            m_pids[i].Compute();

            float output = m_pid_outputs[i];
            if (output >= 0.0f)
            {
                dirs[i] = config::driver::MOTOR_CW;
                duties[i] = static_cast<uint32_t>(output);
            }
            else
            {
                dirs[i] = config::driver::MOTOR_CCW;
                duties[i] = static_cast<uint32_t>(-output);
            }
        }

        m_driver->set_duties(duties, dirs);
    }
}

/**
 * @brief Run the sTune auto-tuning routine on motors (blocking).
 *
 * This method configures sTune, runs the auto-tuning algorithm, and saves
 * the resulting PID gains to NVS. The controller must be initialized before
 * calling this method.
 */
void WheelController::tune_pid()
{
    ESP_LOGI(TAG, "Starting PID auto-tuning on motor 0...");
    m_tuning_mode = true;

    float input = 0.0f, output = 0.0f;
    sTune tuner = sTune(&input, &output, sTune::ZN_PID, sTune::directIP, sTune::printSUMMARY);
    tuner.Configure(config::driver::BL48250_MAX_VEL_RAD, 1023, 0, 500, 5, 2, 50);

    TickType_t xLastWake = xTaskGetTickCount();

    while (true)
    {
        if (tuner.Run() == sTune::TunerStatus::tunings)
            break;

        input = WheelStateEstimator::get_instance().get_filtered_rpm()[0] * (2.0f * M_PI / 60.0f);

        std::array<uint32_t, 4> duties = {0, 0, 0, 0};
        std::array<uint8_t, 4> dirs = {config::driver::MOTOR_CW, config::driver::MOTOR_CW, config::driver::MOTOR_CW,
                                       config::driver::MOTOR_CW};

        if (output >= 0.0f)
        {
            duties[0] = static_cast<uint32_t>(output);
            dirs[0] = config::driver::MOTOR_CW;
        }
        else
        {
            duties[0] = static_cast<uint32_t>(-output);
            dirs[0] = config::driver::MOTOR_CCW;
        }

        m_driver->set_duties(duties, dirs);
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(10));
    }

    // Stop all motors.
    m_driver->set_duties({0, 0, 0, 0}, {0, 0, 0, 0});

    float kp = tuner.GetKp();
    float ki = tuner.GetKi();
    float kd = tuner.GetKd();

    ESP_LOGI(TAG, "Auto-tuning complete. Kp=%.3f Ki=%.3f Kd=%.3f", kp, ki, kd);
    set_pid_tunings(kp, ki, kd);
    save_pid_to_nvs(kp, ki, kd);

    m_tuning_mode = false;
}
