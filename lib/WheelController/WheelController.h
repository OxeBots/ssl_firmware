#ifndef WHEEL_CONTROLLER_H
#define WHEEL_CONTROLLER_H

#include <array>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>

#include "BL48250.h"
#include "wheel_state_estimator.h"
#include "QuickPID.h"
#include "NVSManager.h"
#include "sTune.h"

class WheelController {
public:
    static WheelController& get_instance();

    esp_err_t init(BL48250* driver);
    void set_target_velocities(const std::array<float, 4>& velocities);
    void tune_pid();
    void set_pid_tunings(float kp, float ki, float kd);

private:
    WheelController();
    ~WheelController();
    WheelController(const WheelController&) = delete;
    WheelController& operator=(const WheelController&) = delete;

    static void control_task_wrapper(void* arg);
    void control_task();
    void load_pid_from_nvs();
    void save_pid_to_nvs(float kp, float ki, float kd);

    BL48250* m_driver;
    TaskHandle_t m_task_handle;
    bool m_initialized;
    bool m_tuning_mode;

    std::array<float, 4> m_target_velocities;
    std::array<float, 4> m_current_setpoints; // For soft start
    std::array<float, 4> m_current_velocities;
    std::array<float, 4> m_pid_outputs;
    
    std::array<QuickPID, 4> m_pids;

    float m_kp;
    float m_ki;
    float m_kd;
    
    const float MAX_ACCEL = 100.0f; // rad/s^2 per second limit for soft-start
};

#endif // WHEEL_CONTROLLER_H
