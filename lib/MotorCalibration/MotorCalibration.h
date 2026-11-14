#ifndef _MOTOR_CALIBRATION_H_
#define _MOTOR_CALIBRATION_H_

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <array>
#include <atomic>
#include <cstdint>

// Open-loop motor/encoder wiring calibration. Spins one motor at a time,
// watches every analog and I2C encoder input and derives which encoder input
// belongs to each wheel. Encoder analog ranges are tracked during the same
// spins. The resulting wiring map is applied and persisted by
// WheelStateEstimator.
class MotorCalibration
{
   public:
    enum class State : uint8_t
    {
        IDLE = 0,
        RUNNING = 1,
        DONE = 2,
        FAILED = 3,
    };

    enum class Error : uint8_t
    {
        NONE = 0,
        NO_MOVEMENT = 1,
        AMBIGUOUS_ANALOG = 2,
        AMBIGUOUS_I2C = 3,
        BAD_MAPPING = 4,
        ABORTED = 5,
        I2C = 6,
    };

    struct Status
    {
        State state = State::IDLE;
        uint8_t wheel = 4;        // Wheel being calibrated (0..3), 4 when none
        uint8_t progress = 0;     // Progress of the whole routine (0..100)
        uint8_t result_mask = 0;  // Bit i: wheel i matched on both paths
        uint8_t i2c_ready = 0;    // Bit i: wheel i matched on the I2C path
        std::array<uint8_t, 4> analog_map = {0, 1, 2, 3};
        std::array<uint8_t, 4> mux_map = {0, 1, 2, 3};
        std::array<int16_t, 4> analog_level = {};
        std::array<int16_t, 4> i2c_level = {};
        uint8_t range_mask = 0;
        Error error = Error::NONE;
    };

    static MotorCalibration & get_instance();
    MotorCalibration(const MotorCalibration &) = delete;
    MotorCalibration & operator=(const MotorCalibration &) = delete;

    // Starts the routine in a background task. ESP_ERR_INVALID_STATE while a
    // run is already active.
    esp_err_t start();
    // Aborts a running routine and stops the motors. Safe when idle.
    void abort();
    bool is_running() const;
    // Thread-safe snapshot of the latest status.
    Status get_status() const;

   private:
    // Minimum and maximum raw value of every input inside one sample window.
    struct WindowStats
    {
        std::array<uint16_t, 4> analog_min;
        std::array<uint16_t, 4> analog_max;
        std::array<uint16_t, 4> i2c_min;
        std::array<uint16_t, 4> i2c_max;
    };

    // Minimum-to-maximum spread of every input over one sample window.
    struct WindowSpans
    {
        std::array<uint16_t, 4> analog;
        std::array<uint16_t, 4> i2c;
    };

    MotorCalibration();
    static void task_wrapper(void * arg);
    void task_loop();
    // Publishes a status snapshot under the status mutex.
    void publish(const Status & status);

    void reset_window(WindowStats * window);
    // Samples every analog input and every mux channel once.
    void sample_once(WindowStats * window);
    WindowSpans window_spans(const WindowStats & window) const;
    // Samples for the given time and reports the per-input spreads. Publishes
    // live movement levels when live_status is not null.
    bool run_window(WindowStats * window, uint32_t duration_ms, Status * live_status);
    // Waits for the motors to stop. Returns false when the routine must abort.
    bool run_settle(uint32_t duration_ms);
    // Picks the only input whose baseline-corrected score passed the movement
    // test. Returns Error::NONE when exactly one input moved.
    Error pick_moving_input(const std::array<uint16_t, 4> & spans,
                            const std::array<uint16_t, 4> & baseline,
                            Error ambiguous_error,
                            uint8_t * best_input) const;
    bool is_permutation(const std::array<uint8_t, 4> & map) const;
    // True when the abort flag is set or the runtime budget is exhausted.
    bool should_stop() const;

    static constexpr uint32_t CAL_DUTY = 500;           // PWM duty during a spin
    static constexpr uint32_t BASELINE_MS = 400;        // resting baseline window
    static constexpr uint32_t SPIN_MS = 1500;           // motor spin window
    static constexpr uint32_t SETTLE_MS = 200;          // stop settle window
    static constexpr uint16_t MOVEMENT_MIN_SPAN = 500;  // minimum movement spread
    static constexpr int64_t RUN_BUDGET_US = 20000000;  // hard runtime budget

    SemaphoreHandle_t m_status_mutex;
    Status m_status;
    std::atomic_bool m_running;
    std::atomic_bool m_abort;
    TaskHandle_t m_task_handle;
    int64_t m_run_start_us;
    uint32_t m_i2c_attempts;
    uint32_t m_i2c_failures;
};

#endif  // _MOTOR_CALIBRATION_H_
