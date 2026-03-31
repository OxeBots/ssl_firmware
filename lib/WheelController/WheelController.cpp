#include "WheelController.h"
#include <esp_log.h>
#include <algorithm>
#include <cmath>

static const char* TAG = "WheelController";

WheelController& WheelController::get_instance() {
    static WheelController instance;
    return instance;
}

WheelController::WheelController() : 
    m_driver(nullptr), m_task_handle(nullptr), m_initialized(false), m_tuning_mode(false),
    m_target_velocities({0, 0, 0, 0}), m_current_setpoints({0, 0, 0, 0}), 
    m_current_velocities({0, 0, 0, 0}), m_pid_outputs({0, 0, 0, 0}),
    m_kp(1.0f), m_ki(0.5f), m_kd(0.01f) 
{}

WheelController::~WheelController() {
    if (m_task_handle) {
        vTaskDelete(m_task_handle);
    }
}

esp_err_t WheelController::init(BL48250* driver) {
    if (m_initialized) return ESP_OK;
    if (!driver) return ESP_ERR_INVALID_ARG;

    m_driver = driver;

    load_pid_from_nvs();

    for (int i = 0; i < 4; ++i) {
        m_pids[i] = QuickPID(&m_current_velocities[i], &m_pid_outputs[i], &m_current_setpoints[i]);
        m_pids[i].SetTunings(m_kp, m_ki, m_kd);
        m_pids[i].SetMode(QuickPID::Control::automatic);
        m_pids[i].SetSampleTimeUs(10000); // 100Hz = 10ms
        m_pids[i].SetOutputLimits(-1023.0f, 1023.0f); // Limit output to max PWM duty cycle
        m_pids[i].SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
    }

    xTaskCreatePinnedToCore(control_task_wrapper, "wheel_ctrl", 4096, this, 6, &m_task_handle, 1);
    m_initialized = true;
    return ESP_OK;
}

void WheelController::set_target_velocities(const std::array<float, 4>& velocities) {
    for (int i = 0; i < 4; ++i) {
        m_target_velocities[i] = std::clamp(velocities[i], 
                                            -config::driver::BL48250_MAX_VEL_RAD, 
                                            config::driver::BL48250_MAX_VEL_RAD);
    }
}

void WheelController::set_pid_tunings(float kp, float ki, float kd) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
    for (int i = 0; i < 4; ++i) {
        m_pids[i].SetTunings(m_kp, m_ki, m_kd);
    }
}

void WheelController::load_pid_from_nvs() {
    size_t len = sizeof(float);
    if (NVSManager::load_blob("pid", "kp", &m_kp, &len) != ESP_OK) m_kp = 1.0f;
    if (NVSManager::load_blob("pid", "ki", &m_ki, &len) != ESP_OK) m_ki = 0.5f;
    if (NVSManager::load_blob("pid", "kd", &m_kd, &len) != ESP_OK) m_kd = 0.01f;
    ESP_LOGI(TAG, "Loaded PID from NVS: kp=%.3f, ki=%.3f, kd=%.3f", m_kp, m_ki, m_kd);
}

void WheelController::save_pid_to_nvs(float kp, float ki, float kd) {
    NVSManager::save_blob("pid", "kp", &kp, sizeof(kp));
    NVSManager::save_blob("pid", "ki", &ki, sizeof(ki));
    NVSManager::save_blob("pid", "kd", &kd, sizeof(kd));
    ESP_LOGI(TAG, "Saved PID to NVS: kp=%.3f, ki=%.3f, kd=%.3f", kp, ki, kd);
}

void WheelController::control_task_wrapper(void* arg) {
    static_cast<WheelController*>(arg)->control_task();
}

void WheelController::control_task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz = 10ms
    const float dt = 0.01f;

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (m_tuning_mode) continue;

        std::array<float, 4> current_rpm = WheelStateEstimator::get_instance().get_filtered_rpm();
        
        std::array<uint32_t, 4> duties;
        std::array<uint8_t, 4> dirs;

        for (int i = 0; i < 4; ++i) {
            m_current_velocities[i] = current_rpm[i] * (2.0f * M_PI / 60.0f);

            // Soft-start logic
            float diff = m_target_velocities[i] - m_current_setpoints[i];
            float max_change = MAX_ACCEL * dt;
            if (diff > max_change) {
                m_current_setpoints[i] += max_change;
            } else if (diff < -max_change) {
                m_current_setpoints[i] -= max_change;
            } else {
                m_current_setpoints[i] = m_target_velocities[i];
            }

            m_pids[i].Compute();

            float output = m_pid_outputs[i];
            if (output >= 0) {
                dirs[i] = config::driver::MOTOR_FORWARD;
                duties[i] = static_cast<uint32_t>(output);
            } else {
                dirs[i] = config::driver::MOTOR_BACKWARD;
                duties[i] = static_cast<uint32_t>(-output);
            }
        }

        m_driver->set_duties(duties, dirs);
    }
}

void WheelController::tune_pid() {
    ESP_LOGI(TAG, "Starting PID auto-tuning on motor 0...");
    m_tuning_mode = true;

    // We need to pass float to sTune, using current velocities as input and output is duty cycle
    float input = 0, output = 0;
    sTune tuner = sTune(&input, &output, sTune::ZN_PID, sTune::directIP, sTune::printSUMMARY);
    
    // config::driver::BL48250_MAX_VEL_RAD is roughly max input speed. 
    // Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples)
    tuner.Configure(config::driver::BL48250_MAX_VEL_RAD, 1023, 0, 500, 5, 2, 50);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true) {
        uint8_t status = tuner.Run();
        if (status == sTune::TunerStatus::tunings) {
            break;
        }

        // Read current velocity from encoder 0
        input = WheelStateEstimator::get_instance().get_filtered_rpm()[0] * (2.0f * M_PI / 60.0f);
        
        std::array<uint32_t, 4> duties = {0, 0, 0, 0};
        std::array<uint8_t, 4> dirs = {
            config::driver::MOTOR_FORWARD, config::driver::MOTOR_FORWARD, 
            config::driver::MOTOR_FORWARD, config::driver::MOTOR_FORWARD
        };

        if (output >= 0) {
            duties[0] = static_cast<uint32_t>(output);
            dirs[0] = config::driver::MOTOR_FORWARD;
        } else {
            duties[0] = static_cast<uint32_t>(-output);
            dirs[0] = config::driver::MOTOR_BACKWARD;
        }

        m_driver->set_duties(duties, dirs);
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }

    // Stop all motors
    m_driver->set_duties({0, 0, 0, 0}, {0, 0, 0, 0}); 

    float kp = tuner.GetKp();
    float ki = tuner.GetKi();
    float kd = tuner.GetKd();
    
    ESP_LOGI(TAG, "Auto-tuning completed. Kp: %.3f, Ki: %.3f, Kd: %.3f", kp, ki, kd);

    set_pid_tunings(kp, ki, kd);
    save_pid_to_nvs(kp, ki, kd);

    m_tuning_mode = false;
}
