#ifndef _WHEEL_CONTROLLER_H_
#define _WHEEL_CONTROLLER_H_

#include <driver/gptimer.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <array>

#include "FastPID.h"
#include "NVSManager.h"
#include "constants.h"
#include "wheel_state_estimator.h"

// Velocity control of the four wheels with per-wheel feedforward and PI(D)
// gains, plus a wheel position mode whose PD outer loop drives the velocity
// loop. Static friction is broken by a short start-up kick instead of a
// permanent feedforward floor, and a controller-managed integral action (the
// FastPID stages run with ki = 0 so their internal sums cannot wind up) closes
// the remaining speed error. The calibrated velocity loop keeps integral
// correction through the common maximum.
// Below the continuous-law gate the slower wheels have no stable operating
// point, so those targets use a pulsed low-speed mode. Encoder travel supplies
// the fast speed correction and motor feedback counts trim the mean speed.
// The pulse duty and initial drive fraction use each wheel's directional fit.
// The parameters are persisted per wheel in NVS and can be replaced at runtime
// by the control calibration.
class WheelController
{
   public:
    static WheelController & get_instance();

    esp_err_t init();
    void deinit();

    // Per-wheel velocity-control parameters. Each rotation direction has its
    // own feedforward branch (positive target = forward, negative = reverse);
    // the PID gains are common.
    struct Parameters
    {
        float kp = 1.0f;                 // duty per rad/s of speed error
        float ki = 0.5f;                 // integral gain, duty per rad/s per second
                                         // (applied by the controller, not FastPID)
        float kd = 0.01f;                // derivative gain (FastPID units)
        float feedforward = 0.0f;        // forward kinetic-friction intercept (signed)
        float velocity_ff = 0.0f;        // forward duty per rad/s of target speed
        float static_friction = 175.0f;  // forward minimum duty that starts the wheel
        // Reverse parameters default to the mirrored forward ones, so robots
        // calibrated before the reverse NVS keys existed already compensate
        // the friction of the negative direction.
        float feedforward_rev = 0.0f;        // reverse kinetic-friction intercept (signed)
        float velocity_ff_rev = 0.0f;        // reverse duty per rad/s of target speed
        float static_friction_rev = 175.0f;  // reverse minimum duty that starts the wheel
        float angle_sign = -1.0f;            // raw encoder angle delta per positive rotation
        // Measured continuous-speed boundary and pulse duty in each direction.
        std::array<float, 2> continuous_min_rad_s = {};
        std::array<float, 2> pulse_duty = {};
        std::array<float, 2> pulse_off_duty = {};
        std::array<float, 2> pulse_floor_duty = {};
        std::array<float, 2> pulse_response_s = {};
        std::array<float, 2> pulse_coast_delay_s = {};
        std::array<uint8_t, 2> pulse_interleaved = {};
    };

    static constexpr float MIN_SMOOTH_TARGET_RPM = 50.0f;
    static constexpr float PULSE_HANDOFF_HYSTERESIS_FRACTION = 0.02f;
    static constexpr uint32_t CONTROL_PERIOD_US = 500;
    static constexpr float MAX_PULSE_DUTY = 700.0f;

    // Position-control failure reasons, matching DBG_POSITION_ERROR_*.
    enum class PositionError : uint8_t
    {
        NONE = 0,
        TIMEOUT = 1,
        ABORTED = 2,
        ENCODER = 3,
    };

    void set_target_velocities(const std::array<float, 4> & velocities);

    // Open-loop mode: the control task stops writing the motor outputs so the
    // caller can drive them directly (calibration routines). Entering also
    // cancels position moves and zeroes the velocity targets.
    void set_open_loop(bool enabled);
    // Disables the low-speed pulse/chopper mode while a calibration routine
    // drives the wheels through its own closed-loop steps.
    void set_low_speed_mode_enabled(bool enabled);
    bool is_open_loop() const;

    // Position mode: move the masked wheels by (relative) or to (absolute) an
    // angle in the motor frame. The target is held when reached; the move is
    // abandoned after POSITION_TIMEOUT_US. Velocity targets cancel it.
    esp_err_t start_position_move(uint8_t wheel_mask,
                                  const std::array<float, 4> & angles_rad,
                                  bool relative,
                                  float max_velocity_rad_s);
    void stop_position_moves();
    uint8_t get_position_active_mask() const;
    uint8_t get_position_done_mask() const;
    PositionError get_position_error() const;
    std::array<float, 4> get_position_targets_rad() const;
    // Continuous wheel-shaft angle in the motor frame (positive speed grows it).
    std::array<float, 4> get_position_angles_rad() const;

    // Parameters in effect and persistence helpers used by the calibration.
    std::array<Parameters, 4> get_parameters() const;
    void apply_parameters(int wheel, const Parameters & parameters);

    enum class VelocityMode : uint8_t
    {
        STOPPED,
        CONTINUOUS,
        PULSED,
        PULSE_BLOCKED,
    };
    using TuningValues = std::array<float, 23>;
    VelocityMode get_live_tuning(int wheel, TuningValues & values) const;
    esp_err_t apply_live_tuning(int wheel, uint32_t fields, const TuningValues & values);

    esp_err_t save_parameters(int wheel);
    void save_all_parameters();

    std::array<float, 4> get_target_velocities() const;
    // Returns the signed encoder window mean in the motor frame while the
    // pulse mode owns the wheel, so bench telemetry can show speed ripple.
    uint8_t get_low_speed_velocities_rad_s(std::array<float, 4> & velocities) const;
    std::array<float, 4> get_current_velocities() const;
    std::array<float, 4> get_pid_outputs() const;
    void log_diagnostics();

    // Common calibrated speed limit (rad/s) enforced on every target source.
    // Defaults to the drivetrain maximum until the calibration stores one.
    float get_max_velocity_rad_s() const;
    void set_max_velocity_rad_s(float limit);
    // Persists the current speed limit in NVS.
    esp_err_t save_max_velocity();

   private:
    WheelController();
    ~WheelController();
    WheelController(const WheelController &) = delete;
    WheelController & operator=(const WheelController &) = delete;

    static void task_wrapper(void * arg);
    static bool on_control_alarm(gptimer_handle_t timer,
                                 const gptimer_alarm_event_data_t * event,
                                 void * context);
    void control_task();
    void load_parameters_from_nvs();
    void cancel_position_moves_locked();
    // Drops the controller-managed integral, the start-up kick and the
    // low-speed pulse state of one wheel; call with m_mutex held next to a PID
    // clear.
    void reset_control_state_locked(int wheel);
    // Drops one wheel's low-speed pulse state (the feedback-fault latch is
    // cleared separately because it must survive a mode re-entry).
    void reset_low_speed_locked(int wheel);

    TaskHandle_t m_task_handle;
    gptimer_handle_t m_control_timer = nullptr;
    bool m_initialized;
    bool m_open_loop;
    bool m_low_speed_mode_enabled = true;

    std::array<float, 4> m_target_velocity_rad_s;
    std::array<float, 4> m_current_velocity_rad_s;
    std::array<float, 4> m_pid_output_duty;  // signed PWM duty (±1023)
    // Common calibrated speed limit, applied before the control loop.
    float m_max_velocity_rad_s;

    std::array<FastPID, 4> m_pids;
    std::array<Parameters, 4> m_parameters;

    // Position mode state. The angle is accumulated from the raw encoder
    // angle deltas: the estimator velocity is signed by the commanded motor
    // direction, which is unreliable while the wheel is back-driven.
    std::array<float, 4> m_position_angle_rad;
    std::array<float, 4> m_last_raw_angle_rad;
    std::array<int64_t, 4> m_last_raw_snapshot_us = {};
    std::array<int64_t, 4> m_last_checked_sample_us = {};
    // Per-wheel seeding of the raw-angle accumulator. A channel whose analog
    // input is clamped (dead or intermittent harness) is skipped and must be
    // re-seeded before its delta is accumulated again.
    std::array<bool, 4> m_raw_angle_seeded = {};
    std::array<float, 4> m_position_angle_start_rad;
    std::array<float, 4> m_position_target_rad_s;
    uint8_t m_position_active_mask;
    uint8_t m_position_done_mask;
    PositionError m_position_error;
    uint32_t m_position_start_us;
    float m_position_max_velocity_rad_s;

    // Last output request, held between continuous control updates.
    struct WheelState
    {
        int8_t direction = 1;
        uint32_t duty = 0;
        // Keep a newly commanded direction until the driver has latched it.
        bool target_direction_pending = false;
    };
    std::array<WheelState, 4> m_wheel_state;

    // Start-up kick state: the breakaway duty is a short pulse that breaks the
    // wheel free from rest, not a steady feedforward floor. The kick fires once
    // per target engagement, plus a single rescue pulse when a wheel with a
    // target stays genuinely stuck for a while (the integral cannot restart it
    // alone once its duty is below the breakaway).
    struct KickState
    {
        bool armed = false;
        bool active = false;
        int8_t direction = 0;
        uint32_t start_ms = 0;
        uint32_t cooldown_until_ms = 0;
        // Continuous time the wheel has been stuck at (nearly) zero speed.
        uint32_t stall_since_ms = 0;
    };
    std::array<KickState, 4> m_kick_state = {};
    // Controller-managed integral action (FastPID runs with ki = 0).
    std::array<float, 4> m_integral_duty = {};
    // Sign of the last nonzero target, used to reset the integral and kick.
    std::array<int8_t, 4> m_last_target_sign = {};
    // Last requested target per wheel: a large magnitude change starts a new
    // step, so the integrator is cleared before the new operating point.
    std::array<float, 4> m_last_target_rad_s = {};

    // Per-wheel pulse modulation and travel feedback.
    struct LowSpeedState
    {
        bool selected = false;  // keep the speed gate through feedback dropout
        bool active = false;    // the chopper owns the wheel output
        bool starting = true;   // full drive until the wheel moves
        uint32_t cycle_start_us = 0;
        uint8_t drive_ticks = 0;            // whole drive ticks in the current cycle
        float drive_tick_remainder = 0.0f;  // fractional drive left for later cycles
        float ton_ticks = 0.0f;             // requested drive ticks per cycle
        float boost_duty = 0.0f;            // extra duty for a stalled wheel
        uint32_t stall_since_us = 0;        // start of the current stall check
        float stall_angle_rad = 0.0f;       // wheel angle at that start
        int32_t stall_pulse_count = 0;
        // Position-based 20 ms ripple telemetry.
        float window_start_angle_rad = 0.0f;
        uint32_t window_start_us = 0;
        float last_window_delta_rad = 0.0f;  // diagnostics of the last window
        float last_window_s = 0.0f;
        float last_error_rpm = 0.0f;   // last chopper speed error
        float last_ff_ticks = 0.0f;    // last feed-forward drive ticks
        float integral_ticks = 0.0f;   // window-speed integral (drive ticks)
        float last_window_rpm = 0.0f;  // measured mean of the last window
        float filtered_rpm = 0.0f;     // signed encoder speed along the target direction
        uint32_t feedback_us = 0;
        float feedback_angle_rad = 0.0f;
        int64_t fast_sample_us = 0;
        float previous_fast_rpm = 0.0f;
        float fast_acceleration_rpm_s = 0.0f;
        uint32_t last_warn_ms = 0;  // rate limit for the speed-guard warning
        // Angle-vs-speed consistency check: a wheel whose measured speed does
        // not match the tracked angle motion has a broken angle feedback (dead
        // or misplaced analog channel); the pulse drive must not chase it.
        float guard_angle_rad = 0.0f;  // wheel angle at the window start
        float guard_speed_sum = 0.0f;  // sum of the measured speed samples
        uint32_t guard_start_us = 0;   // window start time
        uint16_t guard_samples = 0;    // speed samples in the window
        uint8_t guard_strikes = 0;     // consecutive inconsistent windows
        uint32_t glitch_first_us = 0;  // first rejected sample since the last good angle
    };
    std::array<LowSpeedState, 4> m_low_speed = {};
    // Timestamp of the low-speed feedback fault (0 = none). A fault blocks the
    // pulse mode below the gate until the target is released or the retry
    // cooldown expires with the analog input recovered.
    std::array<uint32_t, 4> m_ls_fault_ms = {};
    // Rate limit of the "low-speed blocked" warning, per wheel.
    std::array<uint32_t, 4> m_ls_blocked_warn_ms = {};
    // Feedback faults detected from the angle/speed inconsistency are sticky:
    // they only clear when the target is released (the analog calibration is
    // not going to fix itself between two commands).
    std::array<bool, 4> m_ls_fault_sticky = {};

    // PID loop timing
    uint32_t m_last_control_us;
    uint32_t m_last_slow_us = 0;
    // Latest encoder publications with their individual timestamps.
    WheelStateEstimator::ControlSnapshot m_control_snapshot = {};
    // Ripple diagnostics for the low-speed mode: 20 ms windows of the fast
    // velocity, min/max over the last second, logged once per second.
    static constexpr uint32_t RIP_WINDOW_US = 20000;
    static constexpr uint32_t RIP_REPORT_US = 1000000;
    std::array<float, 4> m_rip_min = {};
    std::array<float, 4> m_rip_max = {};
    std::array<float, 4> m_rip_last_mean = {};
    std::array<uint32_t, 4> m_rip_windows = {};
    uint32_t m_last_rip_report_us = 0;
    struct RippleReport
    {
        float target_rpm = 0.0f;
        float mean_rpm = 0.0f;
        float min_rpm = 0.0f;
        float max_rpm = 0.0f;
        float angle_delta = 0.0f;
        float window_s = 0.0f;
        float on_ticks = 0.0f;
        float feedforward_ticks = 0.0f;
        float error_rpm = 0.0f;
        float integral_ticks = 0.0f;
        float boost_duty = 0.0f;
    };
    struct TimingReport
    {
        uint32_t steps = 0;
        uint32_t missed = 0;
        uint32_t max_gap_us = 0;
        uint32_t max_age_us = 0;
        uint32_t max_work_us = 0;
    };
    portMUX_TYPE m_diagnostic_mux = portMUX_INITIALIZER_UNLOCKED;
    std::array<RippleReport, 4> m_ripple_reports = {};
    TimingReport m_timing_report = {};

    // ── Motor model constants (from the Arduino reference) ─────────────────
    static constexpr float STATIC_FRICTION_THRESHOLD = 175.0f;
    static constexpr float PWM_SLEW_RATE_PER_MS = 10.0f;

    // ── Start-up kick and integral action ─────────────────────────────────
    // The kick fires once per target engagement while the wheel is (nearly)
    // stopped, releases as soon as it starts turning or after a short timeout,
    // and is disarmed until the target goes to zero (wheel stopped) or the
    // stall rescue re-arms it. The pulse is short and only carries the
    // breakaway duty: the integral action supplies the sustain duty.
    static constexpr float KICK_START_RAD_S = 1.5f;
    static constexpr float KICK_STOP_RAD_S = 0.3f;
    static constexpr uint32_t KICK_TIMEOUT_MS = 60;
    static constexpr uint32_t KICK_COOLDOWN_MS = 300;
    // The kick re-arms with a zero target and the wheel stopped below this
    // speed, so the next engagement gets exactly one pulse.
    static constexpr float KICK_REARM_RAD_S = 0.3f;
    // Stall rescue: one more pulse after the wheel with a target has stayed
    // below KICK_STALL_RAD_S for KICK_STALL_RESCUE_MS without a pulse.
    static constexpr float KICK_STALL_RAD_S = 0.2f;
    static constexpr uint32_t KICK_STALL_RESCUE_MS = 400;
    // Integral duty clamp: the model feedforward plus this margin must pass the
    // 400 duty breakaway of the slowest wheels.
    static constexpr float INTEGRAL_LIMIT_DUTY = 400.0f;
    // Integral schedule for position moves and wheels without a measured
    // continuous-speed boundary.
    static constexpr float LOW_KI_FULL_RAD_S = 20.0f;
    static constexpr float LOW_KI_ZERO_RAD_S = 45.0f;
    // Largest integral gain the calibration may store.
    static constexpr float KI_MAX_CAL = 60.0f;
    // Parameters saved by the pre-kick firmware have ki = 0. Below this
    // threshold the wheel keeps the legacy static-friction feedforward law
    // (no kick, no integral) until the new law is calibrated for it.
    static constexpr float LEGACY_KI_THRESHOLD = 1.0f;

    // ── Low-speed velocity servo ────────────────────────────────────────
    static constexpr bool LS_MODE_ENABLED = true;
    // Reference speed for an unfitted drive model.
    static constexpr float LS_PULSE_REFERENCE_SPEED_RAD_S = 8.4f;
    // Used until a calibration measures the continuous boundary.
    static constexpr float LS_FALLBACK_CONTINUOUS_GATE_RAD_S = 12.0f;
    static constexpr uint32_t LS_CHOPPER_PERIOD_US = 5000;
    static constexpr uint32_t LS_CHOPPER_PERIOD_TICKS = LS_CHOPPER_PERIOD_US / CONTROL_PERIOD_US;
    // Increase drive after a full-drive interval without motion, up to the
    // pulse duty limit.
    static constexpr uint32_t LS_STALL_TIMEOUT_US = 300000;
    // A small margin over the calibrated breakaway so a wheel whose threshold
    // sits exactly at the calibration value still breaks away from rest.
    static constexpr float LS_BREAKAWAY_MARGIN_DUTY = 20.0f;
    // End the full-drive start once encoder travel shows forward motion.
    static constexpr float LS_MOVING_RPM = 3.0f;
    static constexpr float LS_BOOST_STEP_DUTY = 60.0f;
    // Angle progress that counts as "the wheel is moving" for the stall check.
    static constexpr float LS_STALL_ANGLE_RAD = 0.015f;
    // Speed guard: coast above the pulse operating range.
    static constexpr float LS_RUNAWAY_FACTOR = 3.0f;
    static constexpr float LS_RUNAWAY_FLOOR_RAD_S = 8.5f;
    static constexpr uint32_t LS_FAULT_RETRY_MS = 5000;
    static constexpr uint32_t LS_SNAPSHOT_MAX_AGE_US = 40000;
    // A wheel cannot exceed the calibrated maximum on the bench; an angle delta
    // that implies a much higher wheel rate is a sensor glitch.
    static constexpr float LS_MAX_WHEEL_RATE_RAD_S = 25.0f;
    // Successive controller snapshots can fall on either side of an encoder
    // publication. Allow for one publication interval at the maximum rate.
    static constexpr float ENCODER_DELTA_MARGIN_RAD = 0.011f;
    static constexpr uint32_t LS_GLITCH_CONFIRM_US = 8000;

    // The continuous law keeps its calibrated 100 Hz timing.
    static constexpr float PID_HZ = 100.0f;
    static constexpr uint32_t SLOW_CONTROL_PERIOD_US = 10000;

    // ── Position mode constants ───────────────────────────────────────────
    // Empirically tuned on the bench: the research-sim values (6.9 / 0.66)
    // overshoot with the measured velocity lag of the real plant.
    static constexpr float POSITION_KP = 5.0f;  // 1/s, error -> velocity command
    static constexpr float POSITION_KD = 1.2f;  // velocity damping of the outer loop
    static constexpr float POSITION_TOLERANCE_RAD = 1.5f * M_PI / 180.0f;
    static constexpr float POSITION_VELOCITY_THRESHOLD_RAD_S = 2.0f;
    // Minimum command while outside the tolerance: without it static friction
    // leaves the wheel stalled a few degrees short of the target.
    static constexpr float POSITION_CREEP_RAD_S = 1.5f;
    static constexpr float POSITION_DEFAULT_MAX_RAD_S = 300.0f * RPM_TO_RAD_S;
    static constexpr uint32_t POSITION_TIMEOUT_US = 8000000;
    // Sanity bound: a move stops when it travels more than the requested angle
    // plus one extra wheel turn (a direction/feedback disagreement must not
    // spin the wheel indefinitely).
    static constexpr float POSITION_OVERTRAVEL_RAD = 2.0f * M_PI;
    // The AS5600 reports one turn of the motor shaft (2*pi); the 3:1 reduction
    // makes the wheel-shaft angle one third of the motor-shaft angle.
    static constexpr float RAW_ANGLE_PERIOD_RAD = 2.0f * M_PI;
    static constexpr float RAW_ANGLE_TO_WHEEL = 1.0f / 3.0f;

    // ── NVS keys (private — implementation detail) ─────────────────────────
    static constexpr const char * MOTOR_KEYS[4] = {"fl", "bl", "br", "fr"};
    static constexpr const char * PARAMETERS_SUFFIX = "_tune1";
    static constexpr const char * PID_SUFFIXES[3] = {"_kp", "_ki", "_kd"};
    static constexpr const char * FEEDFORWARD_SUFFIX = "_ff";
    static constexpr const char * VELOCITY_FF_SUFFIX = "_vff";
    static constexpr const char * STATIC_FRICTION_SUFFIX = "_sf";
    // Reverse-direction parameters (negative target).
    static constexpr const char * FEEDFORWARD_REV_SUFFIX = "_ffn";
    static constexpr const char * VELOCITY_FF_REV_SUFFIX = "_vffn";
    static constexpr const char * STATIC_FRICTION_REV_SUFFIX = "_sfn";
    static constexpr const char * ANGLE_SIGN_SUFFIX = "_asign";
    static constexpr const char * CONTINUOUS_MIN_SUFFIX = "_cmin";
    static constexpr const char * PULSE_DUTY_SUFFIX = "_pulse";
    static constexpr const char * PULSE_OFF_DUTY_SUFFIX = "_pulse_off";
    static constexpr const char * PULSE_FLOOR_SUFFIX = "_pulse_floor";
    static constexpr const char * PULSE_RESPONSE_SUFFIX = "_pulse_tau";
    static constexpr const char * PULSE_COAST_SUFFIX = "_pulse_coast";
    static constexpr const char * PULSE_PHASE_SUFFIX = "_pulse_phase";
    // Common calibrated maximum speed, one value for all wheels ("pid" namespace).
    static constexpr const char * MAX_VELOCITY_KEY = "vmax";
    static constexpr float PID_DEFAULT_GAINS[3] = {1.0f, 0.5f, 0.01f};

    SemaphoreHandle_t m_mutex;
};

#endif  // _WHEEL_CONTROLLER_H_
