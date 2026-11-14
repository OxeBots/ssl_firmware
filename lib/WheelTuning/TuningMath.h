#ifndef _TUNING_MATH_H_
#define _TUNING_MATH_H_

#include <cstddef>

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
    // Highest and lowest wheel speed reached by the identification steps:
    // validation must stay inside the identified range instead of extrapolating
    // the model, and a saturated curve (poor separation between the two steps)
    // must be validated near the lower operating point.
    float max_velocity_rad_s = 0.0f;
    float min_velocity_rad_s = 0.0f;
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
    bool valid = false;
};

// Per-wheel control parameters, matching WheelController::Parameters.
struct Gains
{
    float kp = 1.0f;
    float ki = 0.5f;
    float kd = 0.01f;
    float feedforward = 0.0f;  // kinetic friction intercept of the static curve
    float velocity_ff = 0.0f;
    float static_friction = 175.0f;  // minimum duty that starts the wheel moving
};

// Acceptance limits of a validated speed step.
constexpr float RISE_MS_LIMIT = 800.0f;
// The velocity estimate carries encoder quantization ripple, so the measured
// overshoot includes a few percent of measurement noise.
constexpr float OVERSHOOT_PCT_LIMIT = 8.0f;
constexpr float SS_ERROR_PCT_LIMIT = 6.0f;
// Best-effort acceptance when no candidate meets the strict limits.
constexpr float FALLBACK_OVERSHOOT_PCT_LIMIT = 12.0f;
constexpr float FALLBACK_SS_ERROR_PCT_LIMIT = 12.0f;
constexpr float FALLBACK_RISE_MS_LIMIT = 2000.0f;

// FastPID accepts kp up to 255.99 and kd*hz up to 255.99 at the 100 Hz loop.
constexpr float KP_MAX = 200.0f;
constexpr float KI_MAX = 4000.0f;
constexpr float KD_MAX = 0.5f;
constexpr float VELOCITY_FF_MAX = 400.0f;
constexpr float FEEDFORWARD_MAX = 600.0f;

// Leasts squares fit of velocity = gain * duty + offset from operating points
// above the movement threshold. Fills the gain and the duty at zero velocity.
bool fit_static_curve(const float * duties,
                      const float * velocities,
                      size_t count,
                      float * gain,
                      float * friction_duty);

// Time constant (63 %) and dead time (2 %) of one open-loop duty step. The
// times are relative to the step start.
bool fit_step_dynamics(const float * times_s,
                       const float * velocities,
                       size_t count,
                       float * tau_s,
                       float * dead_time_s);

// Metrics of a closed-loop speed step from rest to target_rad_s.
StepMetrics analyze_step(const float * times_s,
                         const float * velocities,
                         size_t count,
                         float target_rad_s);

// Initial candidate: the velocity feedforward carries the setpoint and P only
// trims the residual error (dimensionless loop gain, no integral term). The P
// gain is capped so the step does not saturate the actuator.
Gains synthesize(const PlantModel & plant, float validation_target_rad_s);

// Validation target that keeps the duty inside the actuator range.
float choose_validation_target(const PlantModel & plant);

// Adapt a candidate after a rejected validation step.
Gains refine(const PlantModel & plant, const Gains & current, const StepMetrics & metrics);

// Strict acceptance and best-effort acceptance of a validated step.
bool accepts(const StepMetrics & metrics);
bool accepts_best_effort(const StepMetrics & metrics);

// Cost used to keep the best candidate seen during the refinement loop.
float step_cost(const StepMetrics & metrics);

}  // namespace tuning

#endif  // _TUNING_MATH_H_
