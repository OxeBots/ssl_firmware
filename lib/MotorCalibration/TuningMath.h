#ifndef _TUNING_MATH_H_
#define _TUNING_MATH_H_

#include <array>
#include <cstddef>
#include <cstdint>

// Hardware-free math for the wheel control calibration: plant identification
// from open-loop duty steps, closed-loop step metrics, gain synthesis and the
// refinement policy. Keeping it free of ESP-IDF dependencies makes it
// unit-testable without motors.
namespace tuning
{

// First-order-plus-dead-time model of one wheel in the motor frame.
struct PlantModel
{
    float gain_rad_s_per_duty = 0.0f;  // static gain of velocity over duty
    float friction_duty = 0.0f;        // duty extrapolated to zero velocity
    float tau_s = 0.0f;                // time constant of the velocity step
    float dead_time_s = 0.0f;          // delay before the response starts
    float breakaway_duty = 0.0f;       // duty needed to start moving from rest
    // Highest and lowest wheel-speed magnitudes reached by the identification
    // steps: validation must stay inside the identified range instead of
    // extrapolating the model, and a saturated curve (poor separation between
    // the two steps) must be validated near the lower operating point.
    float max_velocity_rad_s = 0.0f;
    float min_velocity_rad_s = 0.0f;
    // Lowest continuous operating point that passed the settled-speed check.
    float continuous_min_rad_s = 0.0f;
    float continuous_min_duty = 0.0f;
    bool valid = false;
};

// Metrics of one closed-loop speed step.
struct StepMetrics
{
    float rise_ms = 0.0f;        // 10-90 % rise time
    float overshoot_pct = 0.0f;  // peak above the target
    float ss_error_pct = 0.0f;   // final velocity error relative to the target
    float final_rad_s = 0.0f;
    float target_rad_s = 0.0f;
    float settled_ripple_pct = 0.0f;
    bool valid = false;
};

struct SettledSpeed
{
    float mean_rad_s = 0.0f;
    float ripple_rad_s = 0.0f;
    float min_rad_s = 0.0f;
    float max_rad_s = 0.0f;
    bool valid = false;
};

// Mean and RMS speed ripple from successive 100 ms windows at 100 Hz.
SettledSpeed measure_settled_speed(const float * velocities, size_t count);
// Sliding speed windows from measured wheel travel and acquisition times.
SettledSpeed measure_settled_travel(const float * times_s,
                                    const float * angles_rad,
                                    size_t count,
                                    float window_s);
bool tracks_speed(const SettledSpeed & steady, float target_rad_s, float mean_error_fraction);
float peak_speed_error(const SettledSpeed & steady, float target_rad_s);
// Each bit identifies a forward or reverse mean that prevents a shared match.
uint8_t unmatched_speed_mask(const std::array<float, 8> & mean_speeds, float target_rad_s);

// Per-wheel control parameters, matching WheelController::Parameters. Each
// rotation direction has its own feedforward branch; the PID gains are common.
struct Gains
{
    float kp = 1.0f;
    // Integral gain in duty per rad/s per second (controller-managed; the
    // FastPID stage runs with ki = 0). synthesize() overwrites this.
    float ki = 6.0f;
    float kd = 0.01f;
    // Forward direction (positive target): signed static-curve intercept, duty
    // per rad/s and breakaway duty.
    float feedforward = 0.0f;
    float velocity_ff = 0.0f;
    float static_friction = 175.0f;
    // Reverse direction (negative target), same three parameters.
    float feedforward_rev = 0.0f;
    float velocity_ff_rev = 0.0f;
    float static_friction_rev = 175.0f;
};

// Acceptance limits of a validated speed step.
constexpr float RISE_MS_LIMIT = 800.0f;
// The velocity estimate carries encoder quantization ripple, so the measured
// overshoot includes a few percent of measurement noise.
constexpr float OVERSHOOT_PCT_LIMIT = 8.0f;
constexpr float SS_ERROR_PCT_LIMIT = 6.0f;
// Best-effort acceptance when no candidate meets the strict limits.
constexpr float FALLBACK_OVERSHOOT_PCT_LIMIT = 18.0f;
constexpr float FALLBACK_SS_ERROR_PCT_LIMIT = 12.0f;
constexpr float FALLBACK_RISE_MS_LIMIT = 2000.0f;
constexpr float SETTLED_RIPPLE_PCT_LIMIT = 12.0f;

// FastPID accepts kp up to 255.99 and kd*hz up to 255.99 at the 100 Hz loop.
constexpr float KP_MAX = 200.0f;
constexpr float KD_MAX = 0.5f;
constexpr float VELOCITY_FF_MAX = 400.0f;
constexpr float FEEDFORWARD_MAX = 600.0f;

// Integral gain of the control calibration (duty per rad/s per second). The
// initial candidate derives it from the P gain (large: the low-speed integral
// must build the missing friction duty quickly) and the low-speed validation
// adjusts it within these bounds.
constexpr float KI_RATIO = 5.0f;
constexpr float KI_MIN_CAL = 20.0f;
constexpr float KI_MAX_CAL = 60.0f;

// Limits for low-speed diagnostics.
constexpr float LOW_SPEED_RISE_MS_LIMIT = 2000.0f;
constexpr float LOW_SPEED_OVERSHOOT_PCT_LIMIT = 35.0f;
constexpr float LOW_SPEED_SS_ERROR_PCT_LIMIT = 8.0f;

// Allowed tracking error and spread of the four mean speeds, as a fraction
// of the same commanded speed at each shared checkpoint.
constexpr float MATCHED_SPEED_FRACTION = 0.05f;

// Leasts squares fit of velocity = gain * duty + offset from operating points
// above the movement threshold. Fills the gain and the duty at zero velocity.
bool fit_static_curve(const float * duties,
                      const float * velocities,
                      size_t count,
                      float * gain,
                      float * friction_duty);

// Plant line through one high-speed and one low-speed operating point (signed
// duty/velocity in the direction frame). The feedforward then passes through
// both points. Returns false for a degenerate span or a non-physical slope, so
// the caller keeps the high-speed fit.
bool fit_plant_two_points(float velocity_high,
                          float duty_high,
                          float velocity_low,
                          float duty_low,
                          float * gain,
                          float * friction_duty);

// Three-sample median filter for a captured step response.
void median_filter_step(const float * input, float * output, size_t count);

// Time constant (63 %) and dead time (2 %) of one open-loop duty step. The
// times are relative to the step start.
bool fit_step_dynamics(const float * times_s,
                       const float * velocities,
                       size_t count,
                       float * tau_s,
                       float * dead_time_s);

// Metrics of a closed-loop speed step from rest to target_rad_s. The target
// may be positive or negative; the metrics are measured in the direction frame,
// so a mirrored response produces the same rise, overshoot and error. The final
// velocity stays signed.
StepMetrics analyze_step(const float * times_s,
                         const float * velocities,
                         size_t count,
                         float target_rad_s);

// Initial candidate for both directions: the velocity feedforward carries the
// setpoint, P trims the residual error (dimensionless loop gain) and the
// integral gain starts proportional to P. The common P gain is capped so
// neither step saturates the actuator; validation_target_rad_s is the positive
// magnitude used in both directions.
Gains synthesize(const PlantModel & forward_plant,
                 const PlantModel & reverse_plant,
                 float validation_target_rad_s);

// Validation target magnitude that keeps the duty inside the actuator range.
float choose_validation_target(const PlantModel & plant);

// Adapt a candidate after rejected high-speed validation steps in either
// direction: overshoot lowers the P gain (then the integral gain at its floor),
// steady-state error raises the integral gain (trimming the velocity
// feedforward only once the integral is at its cap) and a slow rise raises P.
Gains refine(const Gains & current,
             const StepMetrics & forward_metrics,
             const StepMetrics & reverse_metrics);

// Strict acceptance and best-effort acceptance of a validated step.
bool accepts(const StepMetrics & metrics);
bool accepts_best_effort(const StepMetrics & metrics);

// Acceptance of the low-speed validation step: relaxed rise, overshoot and
// steady-state limits (the wheel may pulse-start at these speeds).
bool accepts_low_speed(const StepMetrics & metrics);

// Adapts the common integral gain after a rejected low-speed validation pair:
// decrease it on overshoot or a failed capture, increase it on steady-state
// error or a slow approach.
Gains adjust_low_speed_integral(const Gains & current,
                                const StepMetrics & forward_metrics,
                                const StepMetrics & reverse_metrics);

// Paired acceptance: a wheel counts as calibrated only when both directions
// pass the same criteria.
bool accepts_pair(const StepMetrics & forward_metrics, const StepMetrics & reverse_metrics);
bool accepts_best_effort_pair(const StepMetrics & forward_metrics,
                              const StepMetrics & reverse_metrics);

// Cost used to keep the best candidate seen during the refinement loop.
float step_cost(const StepMetrics & metrics);

// Common calibrated limit in 0.1 RPM units: the smallest wheel maximum.
// Returns 0 when any wheel has no measured maximum.
uint16_t calibrated_max_rpm(const uint16_t * wheel_max_rpm, size_t count);

}  // namespace tuning

#endif  // _TUNING_MATH_H_
