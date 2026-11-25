#ifndef _MOTOR_CALIBRATION_H_
#define _MOTOR_CALIBRATION_H_

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <freertos/semphr.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "TuningMath.h"
#include "WheelController.h"

// Runs one motor calibration at a time. Encoder alignment maps the wheel
// inputs and anchors the angle ranges. Controller tuning measures the wheels
// and validates their speed control.
class MotorCalibration
{
   public:
    enum class Routine : uint8_t
    {
        NONE,
        ENCODER_ALIGNMENT,
        CONTROLLER_TUNING,
    };

    enum class State : uint8_t
    {
        IDLE = 0,
        RUNNING = 1,
        DONE = 2,
        FAILED = 3,
        // Every wheel was visited but at least one was not matched, so the
        // stored wiring map was left untouched.
        PARTIAL = 4,
    };

    enum class EncoderError : uint8_t
    {
        NONE = 0,
        NO_MOVEMENT = 1,
        AMBIGUOUS_ANALOG = 2,
        AMBIGUOUS_I2C = 3,
        BAD_MAPPING = 4,
        ABORTED = 5,
        I2C = 6,
        MAGNET_TOO_CLOSE = 7,
        MAGNET_TOO_FAR = 8,
    };

    struct EncoderStatus
    {
        State state = State::IDLE;
        uint8_t wheel = 4;         // Wheel being calibrated (0..3), 4 when none
        uint8_t progress = 0;      // Progress of the whole routine (0..100)
        uint8_t result_mask = 0;   // Bit i: wheel i matched on both paths
        uint8_t i2c_ready = 0;     // Bit i: wheel i matched on the I2C path
        uint8_t analog_ready = 0;  // Bit i: wheel i matched on the analog path
        std::array<uint8_t, 4> analog_map = {0, 1, 2, 3};
        std::array<uint8_t, 4> mux_map = {0, 1, 2, 3};
        std::array<int16_t, 4> analog_level = {};
        std::array<int16_t, 4> i2c_level = {};
        uint8_t range_mask = 0;
        std::array<EncoderError, 4> wheel_error =
          {};  // Per-wheel failure reason, NONE when matched
        std::array<EncoderError, 4> magnet_warning = {};  // Last warning seen on each mux input
        EncoderError error = EncoderError::NONE;
    };

    enum class ControllerPhase : uint8_t
    {
        IDLE = 0,
        FRICTION = 1,
        IDENTIFY = 2,
        VALIDATE = 3,
        REFINE = 4,
        STORE = 5,
        DONE = 6,
        // Maximum-speed measurement phase (DBG_TUNING_PHASE_MAX_SPEED).
        MAX_SPEED = 7,
    };

    enum class ControllerError : uint8_t
    {
        NONE = 0,
        NO_MOVEMENT = 1,
        UNSTABLE = 2,
        TIMEOUT = 3,
        ABORTED = 4,
        SPEED_MISMATCH = 5,
        SAVE_FAILED = 6,
    };

    struct ControllerStatus
    {
        State state = State::IDLE;
        uint8_t wheel = 4;  // wheel being calibrated (0..3), 4 when none
        ControllerPhase phase = ControllerPhase::IDLE;
        uint8_t progress = 0;  // progress of the whole routine (0..100)
        uint8_t result_mask = 0;
        ControllerError error = ControllerError::NONE;
        // Validated forward (positive target) step metrics.
        std::array<uint16_t, 4> rise_ms = {};
        std::array<uint8_t, 4> overshoot_pct = {};
        std::array<uint8_t, 4> ss_error_pct = {};
        // Validated reverse (negative target) step metrics.
        std::array<uint16_t, 4> rise_ms_rev = {};
        std::array<uint8_t, 4> overshoot_pct_rev = {};
        std::array<uint8_t, 4> ss_error_pct_rev = {};
        // Per-wheel measured maxima in 0.1 RPM units (0 = not measured).
        std::array<uint16_t, 4> wheel_max_rpm = {};
    };

    static MotorCalibration & get_instance();
    MotorCalibration(const MotorCalibration &) = delete;
    MotorCalibration & operator=(const MotorCalibration &) = delete;

    esp_err_t start_encoder_alignment();
    esp_err_t start_controller_tuning();
    bool is_running() const;
    // With no routine specified, requests a stop of whichever routine is active.
    void abort(Routine routine = Routine::NONE);

    EncoderStatus get_encoder_status() const;
    ControllerStatus get_controller_status() const;

   private:
    MotorCalibration();
    bool reserve_routine(Routine routine);
    void release_routine();
    bool abort_requested() const;

    static void encoder_task_wrapper(void * arg);
    void run_encoder_alignment();
    void publish_encoder_status(const EncoderStatus & status);

    static void controller_task_wrapper(void * arg);
    void run_controller_tuning();
    void publish_controller_status();

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

    // ADC-I2C alignment: short open-loop pulses with a standstill sample after
    // each settle. The fit anchors the raw ADC endpoints to the I2C angle
    // reference so the mapping 0/2*pi matches the I2C reading.
    static constexpr uint32_t ALIGN_DUTY = 500;  // PWM duty per pulse
    // Pulse duration varies deterministically per cycle inside this band (still
    // above the stiction breakaway) so the standstill sample positions spread
    // over the whole turn instead of clustering.
    static constexpr uint32_t ALIGN_PULSE_MIN_MS = 180;
    static constexpr uint32_t ALIGN_PULSE_MAX_MS = 280;
    static constexpr uint32_t ALIGN_PULSE_STEP_MS = 37;  // cycle-to-cycle variation
    static constexpr uint32_t ALIGN_SETTLE_MS = 300;     // standstill wait before sampling
    static constexpr size_t ALIGN_MIN_CYCLES = 10;       // minimum samples/fit points
    static constexpr size_t ALIGN_MAX_CYCLES = 16;       // cap while chasing the spread
    // Adaptive kick: when the first cycles barely move the wheel, drive one
    // long pulse so static friction releases before the next standstill sample.
    static constexpr size_t ALIGN_KICK_AFTER_CYCLES = 2;
    static constexpr int32_t ALIGN_KICK_SPREAD = 200;      // counts considered "barely moving"
    static constexpr uint32_t ALIGN_KICK_MS = 450;         // breakaway kick duration
    static constexpr int32_t ALIGN_MIN_I2C_SPREAD = 2000;  // counts covered per wheel
    // Rest wait: the fixed settle window is only a floor; a wheel that is still
    // coasting would make the ADC and I2C samples describe different angles.
    static constexpr float ALIGN_REST_RAD_S = 1.2f;         // rest threshold (wheel rad/s)
    static constexpr uint32_t ALIGN_REST_POLL_MS = 10;      // rest polling period
    static constexpr uint32_t ALIGN_REST_TIMEOUT_MS = 800;  // rest wait cap after the settle
    static constexpr float ALIGN_MIN_SLOPE = 0.70f;
    static constexpr float ALIGN_MAX_SLOPE = 0.90f;
    static constexpr float ALIGN_MAX_RESIDUAL_RMS = 20.0f;  // counts
    // Robust fit: the analog output is nonlinear/hysteretic within a couple of
    // hundred counts of the 0/360 wrap, so those pairs are excluded and a
    // residual-based outlier rejection runs before the final validation.
    static constexpr int32_t ALIGN_WRAP_GUARD = 200;       // counts excluded near 0/4095
    static constexpr int32_t ALIGN_MIN_FIT_SPREAD = 1500;  // counts covered by retained pairs
    static constexpr size_t ALIGN_MIN_FIT_SAMPLES = 6;     // samples required after filtering
    // Acquisition coverage target: keep pulsing until this many mid-range
    // samples cover ALIGN_MIN_FIT_SPREAD unwrapped counts.
    static constexpr size_t ACQUIRE_MIN_MIDRANGE_SAMPLES = 8;
    static constexpr float ALIGN_OUTLIER_COUNTS = 15.0f;  // residual band for rejection

    struct AlignmentSamples
    {
        std::array<uint16_t, ALIGN_MAX_CYCLES> i2c = {};
        std::array<uint16_t, ALIGN_MAX_CYCLES> adc = {};
        size_t count = 0;
        size_t fit_count = 0;
        int32_t spread = 0;
        int32_t fit_spread = 0;
    };

    struct RawRanges
    {
        std::array<uint16_t, 4> minimum;
        std::array<uint16_t, 4> maximum;
    };

    // Least-squares fit of the standstill ADC samples against the I2C angle
    // reference; valid only when slope, residual and endpoints pass the checks.
    struct LinearFit
    {
        bool valid = false;
        float slope = 0.0f;
        float intercept = 0.0f;
        float residual_rms = 0.0f;
        size_t samples_used = 0;
        size_t outliers_dropped = 0;
    };

    void reset_window(WindowStats * window);
    // Samples every analog input and every mux channel once.
    void sample_once(WindowStats * window, std::array<EncoderError, 4> * magnet_warning);
    WindowSpans window_spans(const WindowStats & window) const;
    // Samples for the given time and reports the per-input spreads. Publishes
    // live movement levels when live_status is not null.
    bool run_window(WindowStats * window, uint32_t duration_ms, EncoderStatus * live_status);
    // Waits for the motors to stop. Returns false when the routine must abort.
    bool run_settle(uint32_t duration_ms);
    // Waits until one wheel is at rest after a pulse (settle floor + smoothed
    // velocity polling) so the ADC/I2C pair is sampled at the same angle.
    // Returns false when the routine must abort.
    bool wait_for_rest(uint8_t wheel);
    // Ordinary least squares adc = slope*i2c + intercept; false when the paired
    // data has no usable spread.
    static bool fit_line(const std::array<uint16_t, ALIGN_MAX_CYCLES> & i2c,
                         const std::array<uint16_t, ALIGN_MAX_CYCLES> & adc,
                         size_t count,
                         double * slope,
                         double * intercept,
                         double * rms);
    // Robust least-squares fit of adc = slope*i2c + intercept over the retained
    // standstill pairs: one pass, drop residuals above ALIGN_OUTLIER_COUNTS and
    // refit, then validate the slope range, residual RMS and anchored endpoints.
    static LinearFit fit_adc_vs_i2c(const std::array<uint16_t, ALIGN_MAX_CYCLES> & i2c,
                                    const std::array<uint16_t, ALIGN_MAX_CYCLES> & adc,
                                    size_t count);
    bool pulse_alignment_wheel(uint8_t wheel, uint32_t duration_ms);
    EncoderError collect_alignment_samples(uint8_t wheel,
                                           uint8_t analog_slot,
                                           AlignmentSamples & samples);
    static bool save_alignment(uint8_t wheel, const AlignmentSamples & samples);
    bool align_adc_to_i2c(const std::array<uint8_t, 4> & analog_map, uint8_t & anchored_mask);

    static void log_spans(uint8_t wheel, const char * label, const WindowSpans & spans);
    bool measure_wheel_motion(EncoderStatus & status, WindowSpans & baseline, WindowSpans & spin);
    EncoderError match_wheel_inputs(EncoderStatus & status,
                                    const WindowSpans & baseline,
                                    const WindowSpans & spin);
    EncoderError detect_wiring(EncoderStatus & status);
    EncoderError apply_calibration(EncoderStatus & status, const RawRanges & previous_ranges);
    static void restore_unaligned_ranges(uint8_t anchored_mask, const RawRanges & previous_ranges);
    void publish_encoder_completion(EncoderStatus & status, EncoderError error);
    // Picks the only input whose baseline-corrected score passed the movement
    // test. Returns EncoderError::NONE when exactly one input moved.
    EncoderError pick_moving_input(const std::array<uint16_t, 4> & spans,
                                   const std::array<uint16_t, 4> & baseline,
                                   EncoderError ambiguous_error,
                                   uint8_t * best_input) const;
    bool is_permutation(const std::array<uint8_t, 4> & map) const;
    // True when the abort flag is set or the runtime budget is exhausted.
    bool should_stop() const;

    static constexpr uint32_t CAL_DUTY = 500;           // PWM duty during a spin
    static constexpr uint32_t BASELINE_MS = 400;        // resting baseline window
    static constexpr uint32_t SPIN_MS = 1500;           // motor spin window
    static constexpr uint32_t SETTLE_MS = 200;          // stop settle window
    static constexpr uint16_t MOVEMENT_MIN_SPAN = 500;  // minimum movement spread
    static constexpr int64_t ENCODER_RUN_BUDGET_US =
      60000000;  // hard runtime budget incl. alignment

    // Sets the current wheel/phase and the whole-routine progress derived from
    // the per-wheel progress (0..100).
    void set_controller_progress(uint8_t wheel, ControllerPhase phase, uint8_t wheel_progress);

    // 0 = keep going, 1 = aborted, 2 = runtime budget exhausted.
    int check_stop(uint32_t start_us);
    // Sleeps for the duration, returning false when the routine must stop.
    bool wait_ms(uint32_t duration_ms, uint32_t start_us);
    // Drives one wheel in open loop, holding the other three at zero duty.
    void drive_wheel(int wheel, int duty);
    // Mean raw-encoder velocity of one wheel over a sampling window.
    bool sample_velocity(int wheel, uint32_t duration_ms, uint32_t start_us, float * velocity);
    // Records a step response in the task's sample buffers.
    size_t sample_step(int wheel, uint32_t duration_ms, uint32_t start_us);

    // Measures the breakaway duty magnitude of one wheel in open loop for the
    // requested rotation direction (+1 forward, -1 reverse). The forward run
    // also learns the sign of the raw encoder angle for a positive rotation.
    bool measure_breakaway(
      int wheel, int direction, uint32_t start_us, int * breakaway_duty, float * angle_sign);
    struct PlantMeasurement
    {
        int duty_low;
        int duty_high;
        float velocity_low = 0.0f;
        float velocity_high = 0.0f;
        float tau_s = 0.0f;
        float dead_time_s = 0.0f;
        int floor_duty = 0;
        float floor_speed = 0.0f;
    };

    bool capture_plant_response(int wheel,
                                int direction,
                                uint32_t start_us,
                                PlantMeasurement & measurement);
    void measure_continuous_floor(int wheel,
                                  int direction,
                                  uint32_t start_us,
                                  const tuning::SettledSpeed & high,
                                  PlantMeasurement & measurement);
    static bool fit_plant_model(int wheel,
                                int attempt,
                                int direction,
                                int breakaway_duty,
                                const PlantMeasurement & measurement,
                                tuning::PlantModel & plant);
    // Identifies the plant model with two signed open-loop duty steps in the
    // requested direction (+1 forward, -1 reverse).
    bool identify_plant(
      int wheel, int breakaway_duty, int direction, uint32_t start_us, tuning::PlantModel * plant);
    tuning::SettledSpeed measure_open_loop_speed(int wheel, int duty, uint32_t start_us);
    tuning::StepMetrics validate_step(int wheel, float target_rad_s, uint32_t start_us);
    bool validate_gains(int wheel,
                        float target_rad_s,
                        uint32_t start_us,
                        tuning::Gains gains,
                        WheelController::Parameters & parameters);
    void select_pulse_duties(int wheel,
                             uint32_t start_us,
                             WheelController::Parameters & parameters);
    std::array<tuning::SettledSpeed, 4> measure_tracking(const std::array<float, 4> & targets,
                                                         uint32_t start_us,
                                                         int wheel);
    bool wait_for_stop(int wheel, uint32_t start_us);
    // Measures the maximum wheel speed in both open-loop directions at full
    // duty. The values are signed in the commanded direction in rad/s.
    bool measure_max_speed(int wheel,
                           uint32_t start_us,
                           float * forward_rad_s,
                           float * reverse_rad_s);
    // Saves the minimum of the four measured maxima, when all are available.
    bool update_max_speed_limit(uint32_t start_us);
    bool verify_shared_speeds(uint32_t start_us);
    ControllerError tune_wheel(int wheel,
                               uint32_t start_us,
                               WheelController::Parameters & parameters);

    // Bound the four-wheel directional search, long range checks and retries.
    static constexpr uint32_t CONTROLLER_RUN_BUDGET_US = 30 * 60 * 1000000U;
    static constexpr uint32_t SAMPLE_PERIOD_MS = 10;     // 100 Hz sampling
    static constexpr size_t STEP_SAMPLE_CAPACITY = 320;  // 3.2 s

    static constexpr int BREAKAWAY_START_DUTY = 120;
    static constexpr int BREAKAWAY_STEP_DUTY = 40;
    static constexpr int BREAKAWAY_MAX_DUTY = 600;
    static constexpr uint32_t BREAKAWAY_HOLD_MS = 220;
    static constexpr uint32_t ANGLE_SIGN_SAMPLE_MS = 700;
    static constexpr float MOVEMENT_THRESHOLD_RAD_S = 2.0f;

    // The EKF velocity estimate settles slowly after the acceleration
    // transient: hold each duty step 2 s and average the last 0.5 s.
    static constexpr uint32_t IDENTIFY_HOLD_MS = 2000;
    static constexpr size_t IDENTIFY_STEADY_SAMPLES = 50;

    // Ramp down from the high identification point to find the lowest
    // continuous speed that remains steady.
    static constexpr int RAMP_STEP_DUTY = 40;
    static constexpr int RAMP_MIN_DUTY = 80;
    static constexpr uint32_t RAMP_STEP_MS = 300;
    static constexpr uint32_t RAMP_SAMPLE_MS = 600;
    static constexpr float RAMP_STOP_RAD_S = 1.0f;

    static constexpr uint32_t VALIDATE_HOLD_MS = 2000;
    static constexpr uint8_t MAX_REFINEMENTS = 2;
    static constexpr uint32_t STOP_TIMEOUT_MS = 3000;

    // Let the motor settle at full duty before measuring its maximum speed.
    static constexpr int MAX_SPEED_FULL_DUTY = 1023;
    static constexpr uint32_t MAX_SPEED_SETTLE_MS = 1200;
    static constexpr uint32_t MAX_SPEED_SAMPLE_MS = 800;
    static constexpr uint32_t TRACKING_SETTLE_MS = 2000;
    static constexpr uint32_t TRACKING_SAMPLE_MS = 2000;

    mutable portMUX_TYPE m_run_mux = portMUX_INITIALIZER_UNLOCKED;
    Routine m_routine = Routine::NONE;
    bool m_abort_requested = false;

    SemaphoreHandle_t m_status_mutex;
    EncoderStatus m_encoder_status;
    ControllerStatus m_controller_status;
    ControllerStatus m_controller_working;

    int64_t m_encoder_start_us = 0;
    uint32_t m_i2c_attempts = 0;
    uint32_t m_i2c_failures = 0;

    std::array<float, STEP_SAMPLE_CAPACITY> m_step_times_s = {};
    std::array<float, STEP_SAMPLE_CAPACITY> m_step_velocities = {};
    float m_previous_max_velocity_rad_s = 0.0f;
};

#endif  // _MOTOR_CALIBRATION_H_
