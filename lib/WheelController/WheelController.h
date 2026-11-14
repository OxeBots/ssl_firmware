#ifndef _WHEEL_CONTROLLER_H_
#define _WHEEL_CONTROLLER_H_

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
// loop. The parameters are persisted per wheel in NVS and can be replaced at
// runtime by the control calibration.
class WheelController
{
   public:
    static WheelController & get_instance();

    esp_err_t init();
    void deinit();

    // Per-wheel velocity-control parameters.
    struct Parameters
    {
        float kp = 1.0f;                 // duty per rad/s of speed error
        float ki = 0.5f;                 // duty per rad/s per second
        float kd = 0.01f;                // derivative gain (FastPID units)
        float feedforward = 0.0f;        // kinetic-friction intercept of the static curve (signed)
        float velocity_ff = 0.0f;        // duty per rad/s of target speed
        float static_friction = 175.0f;  // minimum duty that starts the wheel moving
        float angle_sign = -1.0f;        // raw encoder angle delta per positive rotation
    };

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
    void save_parameters(int wheel);
    void save_all_parameters();

    std::array<float, 4> get_target_velocities() const;
    std::array<float, 4> get_current_velocities() const;
    std::array<float, 4> get_pid_outputs() const;

   private:
    WheelController();
    ~WheelController();
    WheelController(const WheelController &) = delete;
    WheelController & operator=(const WheelController &) = delete;

    static void task_wrapper(void * arg);
    void control_task();
    void load_parameters_from_nvs();
    void cancel_position_moves_locked();
    // Position-mode velocity command for one wheel, or the plain velocity target.
    float position_velocity_target(size_t wheel, float measured_rad_s, uint32_t now_us);

    TaskHandle_t m_task_handle;
    bool m_initialized;
    bool m_open_loop;

    std::array<float, 4> m_target_velocity_rad_s;
    std::array<float, 4> m_current_velocity_rad_s;
    std::array<float, 4> m_pid_output_duty;  // signed PWM duty (±1023)

    std::array<FastPID, 4> m_pids;
    std::array<Parameters, 4> m_parameters;

    // Position mode state. The angle is accumulated from the raw encoder
    // angle deltas: the estimator velocity is signed by the commanded motor
    // direction, which is unreliable while the wheel is back-driven.
    std::array<float, 4> m_position_angle_rad;
    std::array<float, 4> m_last_raw_angle_rad;
    bool m_raw_angle_seeded;
    std::array<float, 4> m_position_angle_start_rad;
    std::array<float, 4> m_position_target_rad_s;
    uint8_t m_position_active_mask;
    uint8_t m_position_done_mask;
    PositionError m_position_error;
    uint32_t m_position_start_us;
    float m_position_max_velocity_rad_s;

    // Per-wheel state: current signed PWM, used by the slew-rate limiter.
    struct WheelState
    {
        int8_t direction = 1;
        uint32_t duty = 0;
    };
    std::array<WheelState, 4> m_wheel_state;

    // PID loop timing
    uint32_t m_last_control_us;

    // ── Motor model constants (from the Arduino reference) ─────────────────
    static constexpr float STATIC_FRICTION_THRESHOLD = 175.0f;
    static constexpr float PWM_SLEW_RATE_PER_MS = 10.0f;
    static constexpr float ZERO_VELOCITY_THRESHOLD_RAD_S = 0.5f;

    // PID update rate: 100 Hz (matches CONFIG_FREERTOS_HZ=100)
    static constexpr float PID_HZ = 100.0f;

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
    static constexpr const char * PID_SUFFIXES[3] = {"_kp", "_ki", "_kd"};
    static constexpr const char * FEEDFORWARD_SUFFIX = "_ff";
    static constexpr const char * VELOCITY_FF_SUFFIX = "_vff";
    static constexpr const char * STATIC_FRICTION_SUFFIX = "_sf";
    static constexpr const char * ANGLE_SIGN_SUFFIX = "_asign";
    static constexpr float PID_DEFAULT_GAINS[3] = {1.0f, 0.5f, 0.01f};

    SemaphoreHandle_t m_mutex;
};

#endif  // _WHEEL_CONTROLLER_H_
