#ifndef _WHEEL_TUNING_H_
#define _WHEEL_TUNING_H_

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <array>
#include <atomic>
#include <cstdint>

#include "TuningMath.h"

// Per-wheel velocity-control calibration. For each wheel the routine measures
// the static friction breakaway duty and the plant gain from open-loop duty
// steps, synthesizes PID gains plus friction/velocity feedforward, validates
// the closed-loop speed step and refines the candidate until the step meets
// the acceptance limits. The winning parameters are persisted in NVS through
// WheelController.
class WheelTuning
{
   public:
    enum class State : uint8_t
    {
        IDLE = 0,
        RUNNING = 1,
        DONE = 2,
        FAILED = 3,
    };

    enum class Phase : uint8_t
    {
        IDLE = 0,
        FRICTION = 1,
        IDENTIFY = 2,
        VALIDATE = 3,
        REFINE = 4,
        STORE = 5,
        DONE = 6,
    };

    enum class Error : uint8_t
    {
        NONE = 0,
        NO_MOVEMENT = 1,
        UNSTABLE = 2,
        TIMEOUT = 3,
        ABORTED = 4,
    };

    struct Status
    {
        State state = State::IDLE;
        uint8_t wheel = 4;  // wheel being calibrated (0..3), 4 when none
        Phase phase = Phase::IDLE;
        uint8_t progress = 0;  // progress of the whole routine (0..100)
        uint8_t result_mask = 0;
        Error error = Error::NONE;
        std::array<uint16_t, 4> rise_ms = {};
        std::array<uint8_t, 4> overshoot_pct = {};
        std::array<uint8_t, 4> ss_error_pct = {};
    };

    static WheelTuning & get_instance();
    WheelTuning(const WheelTuning &) = delete;
    WheelTuning & operator=(const WheelTuning &) = delete;

    // Starts the routine in the background. ESP_ERR_INVALID_STATE while a run
    // or the motor/encoder wiring calibration is already active.
    esp_err_t start();
    // Aborts a running routine and stops the motors. Safe when idle.
    void abort();
    bool is_running() const;
    // Thread-safe snapshot of the latest status.
    Status get_status() const;

   private:
    WheelTuning();
    static void task_wrapper(void * arg);
    void task_loop();

    // Publishes the working status snapshot under the status mutex.
    void publish();
    // Sets the current wheel/phase and the whole-routine progress derived from
    // the per-wheel progress (0..100).
    void set_progress(uint8_t wheel, Phase phase, uint8_t wheel_progress);

    // 0 = keep going, 1 = aborted, 2 = runtime budget exhausted.
    int check_stop(uint32_t start_us);
    // Sleeps for the duration, returning false when the routine must stop.
    bool wait_ms(uint32_t duration_ms, uint32_t start_us);
    // Drives one wheel in open loop, holding the other three at zero duty.
    void drive_wheel(int wheel, int duty);
    // Mean motor-frame velocity of one wheel over a sampling window.
    bool sample_velocity(int wheel, uint32_t duration_ms, uint32_t start_us, float * velocity);
    // Records one open-loop step response (times in seconds, motor frame).
    size_t sample_step(int wheel,
                       uint32_t duration_ms,
                       uint32_t start_us,
                       float * times_s,
                       float * velocities,
                       size_t capacity);

    // Measures the breakaway duty of one wheel in open loop and the sign of the
    // raw encoder angle for a positive rotation.
    bool measure_breakaway(int wheel, uint32_t start_us, int * breakaway_duty, float * angle_sign);
    // Identifies the plant model with two open-loop duty steps.
    bool identify_plant(int wheel,
                        int breakaway_duty,
                        uint32_t start_us,
                        tuning::PlantModel * plant);
    // Runs one closed-loop speed step through WheelController and measures it.
    bool validate_step(int wheel,
                       float target_rad_s,
                       uint32_t start_us,
                       tuning::StepMetrics * metrics);
    // Calibrates one wheel; returns true when valid parameters were stored.
    bool tune_wheel(int wheel, uint32_t start_us);

    static constexpr uint32_t RUN_BUDGET_US = 150000000;  // hard runtime budget
    static constexpr uint32_t SAMPLE_PERIOD_MS = 10;      // 100 Hz sampling
    static constexpr size_t STEP_SAMPLE_CAPACITY = 240;   // 2.4 s

    static constexpr int BREAKAWAY_START_DUTY = 120;
    static constexpr int BREAKAWAY_STEP_DUTY = 40;
    static constexpr int BREAKAWAY_MAX_DUTY = 600;
    static constexpr uint32_t BREAKAWAY_HOLD_MS = 220;
    static constexpr float MOVEMENT_THRESHOLD_RAD_S = 2.0f;

    // The EKF velocity estimate settles slowly after the acceleration
    // transient: hold each duty step 2 s and average the last 0.5 s.
    static constexpr uint32_t IDENTIFY_HOLD_MS = 2000;
    static constexpr size_t IDENTIFY_STEADY_SAMPLES = 50;
    static constexpr uint32_t IDENTIFY_STOP_MS = 500;
    static constexpr float IDENTIFY_MIN_GAIN = 1.0e-4f;

    static constexpr uint32_t VALIDATE_SETTLE_MS = 700;
    static constexpr uint32_t VALIDATE_HOLD_MS = 2000;
    static constexpr uint8_t MAX_REFINEMENTS = 5;
    // A step with fewer valid samples is treated as a failed validation.
    static constexpr size_t VALIDATE_MIN_SAMPLES = 50;

    SemaphoreHandle_t m_status_mutex;
    Status m_status;   // published snapshot read by other tasks
    Status m_working;  // task-local state, published after every change
    std::atomic_bool m_running;
    std::atomic_bool m_abort;
    TaskHandle_t m_task_handle;
};

#endif  // _WHEEL_TUNING_H_
