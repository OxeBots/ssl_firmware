#ifndef WHEEL_CONTROLLER_H
#define WHEEL_CONTROLLER_H

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <array>

#include "FastPID.h"
#include "NVSManager.h"
#include "RelayFeedbackTuner.h"
#include "constants.h"
#include "wheel_state_estimator.h"

/**
 * @brief PID controller with feed-forward, startup boost, slew-rate limiting,
 * dynamic anti-windup, and relay-feedback auto-tuning — ported from the
 * Arduino Uno reference code.
 *
 * Each wheel has its own FastPID instance (fixed-point, no FP in step()),
 * feed-forward to overcome static friction, a boost pulse when the motor
 * is stuck at zero velocity, and relay-based PID auto-tuning.
 */
class WheelController
{
   public:
    static WheelController & get_instance();

    esp_err_t init();
    void deinit();

    void set_target_velocities(const std::array<float, 4> & velocities);

    void set_pid_tunings(const std::array<float, 4> & kp,
                         const std::array<float, 4> & ki,
                         const std::array<float, 4> & kd);

    using TuningDoneCallback = void (*)(bool all_success);
    void set_tuning_done_callback(TuningDoneCallback cb);
    void start_relay_tuning();
    bool is_tuning_active() const;
    bool is_tuning_complete() const;
    std::array<bool, 4> get_tuning_success() const;

    std::array<float, 4> get_target_velocities() const;
    std::array<float, 4> get_current_velocities() const;
    std::array<float, 4> get_pid_outputs() const;

    void save_pid_to_nvs(int motor_idx);
    void save_all_pid_to_nvs();

   private:
    WheelController();
    ~WheelController();
    WheelController(const WheelController &) = delete;
    WheelController & operator=(const WheelController &) = delete;

    static void task_wrapper(void * arg);
    void control_task();
    static void tuning_task_wrapper(void * arg);
    void tuning_task();
    void load_pid_from_nvs();

    TaskHandle_t m_task_handle;
    TaskHandle_t m_tuning_task_handle;
    bool m_initialized;
    bool m_tuning_mode;
    bool m_tuning_complete;

    std::array<float, 4> m_target_velocity_rad_s;
    std::array<float, 4> m_current_velocity_rad_s;
    std::array<float, 4> m_pid_output_duty;  // signed PWM duty (±1023)

    std::array<FastPID, 4> m_pids;
    std::array<RelayFeedbackTuner, 4> m_tuners;

    std::array<float, 4> m_kp;
    std::array<float, 4> m_ki;
    std::array<float, 4> m_kd;

    TuningDoneCallback m_tuning_done_cb;

    // Per-wheel state (boost, last velocity, current signed PWM)
    struct WheelState
    {
        uint32_t boost_until_us = 0;
        float last_measured_velocity_rad_s = 0.0f;
        int8_t direction = 1;
        uint32_t duty = 0;
    };
    std::array<WheelState, 4> m_wheel_state;

    // PID loop timing
    uint32_t m_last_control_us;

    // ── Motor model constants (from Arduino reference) ─────────────────
    static constexpr float STATIC_FRICTION_THRESHOLD = 175.0f;
    static constexpr float STUCK_VELOCITY_THRESHOLD_RAD_S = 0.5f;
    static constexpr float BOOST_MULTIPLIER = 1.1f;
    static constexpr uint32_t BOOST_DURATION_US = 150;
    static constexpr float PWM_SLEW_RATE_PER_MS = 10.0f;
    static constexpr float ZERO_VELOCITY_THRESHOLD_RAD_S = 0.5f;

    // PID update rate: 100 Hz (matches CONFIG_FREERTOS_HZ=100)
    static constexpr float PID_HZ = 100.0f;

    // ── Relay tuning constants ─────────────────────────────────────────
    static constexpr float RELAY_AMPLITUDE = 200.0f;
    static constexpr uint32_t TUNE_MAX_MS = 30000;  // 30 s max per motor
    static constexpr uint8_t TUNE_MIN_PERIODS = 4;

    // ── NVS keys (private — implementation detail) ─────────────────────
    static constexpr const char * MOTOR_KEYS[4] = {"fl", "bl", "br", "fr"};
    static constexpr const char * PID_SUFFIXES[3] = {"_kp", "_ki", "_kd"};
    static constexpr float PID_DEFAULT_GAINS[3] = {1.0f, 0.5f, 0.01f};

    SemaphoreHandle_t m_mutex;
};

#endif  // WHEEL_CONTROLLER_H
