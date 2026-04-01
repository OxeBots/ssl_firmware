#ifndef WHEEL_CONTROLLER_H
#define WHEEL_CONTROLLER_H

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <array>

#include "BL48250.h"
#include "NVSManager.h"
#include "QuickPID.h"
#include "sTune.h"
#include "wheel_state_estimator.h"

class WheelController
{
   public:
    static WheelController & get_instance();

    esp_err_t init(BL48250 * driver);

    void deinit();

    void set_target_velocities(const std::array<float, 4> & velocities);

    void set_pid_tunings(float kp, float ki, float kd);

    void tune_pid();

   private:
    WheelController();
    ~WheelController();
    WheelController(const WheelController &) = delete;
    WheelController & operator=(const WheelController &) = delete;

    static void task_wrapper(void * arg);
    void control_task();
    void load_pid_from_nvs();
    void save_pid_to_nvs(float kp, float ki, float kd);

    BL48250 * m_driver;
    TaskHandle_t m_task_handle;
    bool m_initialized;
    bool m_tuning_mode;

    std::array<float, 4> m_target_velocities;
    std::array<float, 4> m_current_setpoints;
    std::array<float, 4> m_current_velocities;
    std::array<float, 4> m_pid_outputs;

    std::array<QuickPID, 4> m_pids;

    float m_kp;
    float m_ki;
    float m_kd;

    static constexpr float MAX_ACCEL = 100.0f;
};

#endif  // WHEEL_CONTROLLER_H
