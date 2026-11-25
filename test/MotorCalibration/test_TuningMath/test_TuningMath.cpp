#include <unity.h>

#include <algorithm>
#include <array>
#include <cmath>

#include "TuningMath.h"

// ============================================================================
// PLANT IDENTIFICATION
// ============================================================================

void test_static_curve_fits_gain_and_friction()
{
    const float duties[2] = {200.0f, 400.0f};
    const float velocities[2] = {3.0f, 7.0f};
    float gain = 0.0f;
    float friction = 0.0f;

    TEST_ASSERT_TRUE(tuning::fit_static_curve(duties, velocities, 2, &gain, &friction));
    TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, 0.02f, gain);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 50.0f, friction);
}

// The reverse branch is identified with negative duty and velocity: the gain
// stays positive and the intercept keeps the branch sign.
void test_reverse_static_curve_fits_signed_intercept()
{
    const float duties[2] = {-200.0f, -400.0f};
    const float velocities[2] = {-3.0f, -7.0f};
    float gain = 0.0f;
    float friction = 0.0f;

    TEST_ASSERT_TRUE(tuning::fit_static_curve(duties, velocities, 2, &gain, &friction));
    TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, 0.02f, gain);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, -50.0f, friction);
}

void test_static_curve_rejects_a_stalled_wheel()
{
    const float duties[2] = {200.0f, 400.0f};
    const float velocities[2] = {0.0f, 0.4f};
    float gain = 0.0f;
    float friction = 0.0f;

    TEST_ASSERT_FALSE(tuning::fit_static_curve(duties, velocities, 2, &gain, &friction));
}

// The low-speed sweep refit passes through both the high-speed step and the
// low-speed ramp point, so the feedforward is correct at the 33 RPM target.
void test_plant_line_through_high_and_low_operating_points()
{
    float gain = 0.0f;
    float friction = 0.0f;
    TEST_ASSERT_TRUE(tuning::fit_plant_two_points(14.0f, 800.0f, 3.5f, 300.0f, &gain, &friction));
    // (14 - 3.5) / (800 - 300) = 0.021.
    TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, 0.021f, gain);
    // Duty at zero velocity: 300 - 3.5 / 0.021 = 133.33.
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 300.0f - 3.5f / 0.021f, friction);

    // Reverse branch: both duty and velocity are negative, same gain and a
    // mirrored intercept.
    TEST_ASSERT_TRUE(
      tuning::fit_plant_two_points(-14.0f, -800.0f, -3.5f, -300.0f, &gain, &friction));
    TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, 0.021f, gain);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, -(300.0f - 3.5f / 0.021f), friction);
}

void test_plant_line_rejects_degenerate_operating_points()
{
    float gain = 0.0f;
    float friction = 0.0f;
    // No duty span: no line.
    TEST_ASSERT_FALSE(tuning::fit_plant_two_points(14.0f, 800.0f, 3.5f, 800.0f, &gain, &friction));
    // The "low" point is faster than the "high" point: non-physical slope.
    TEST_ASSERT_FALSE(tuning::fit_plant_two_points(3.5f, 800.0f, 14.0f, 300.0f, &gain, &friction));
    TEST_ASSERT_FALSE(
      tuning::fit_plant_two_points(14.0f, 800.0f, 3.5f, 300.0f, nullptr, &friction));
    TEST_ASSERT_FALSE(tuning::fit_plant_two_points(14.0f, 800.0f, 3.5f, 300.0f, &gain, nullptr));
}

void test_median_filter_step_rejects_an_outlier_and_keeps_endpoints()
{
    const float input[] = {10.0f, 80.0f, 12.0f, 14.0f};
    const float expected[] = {10.0f, 12.0f, 14.0f, 14.0f};
    float filtered[4] = {};
    tuning::median_filter_step(input, filtered, 4);
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(expected, filtered, 4);
}

void test_step_dynamics_recovers_time_constant_and_dead_time()
{
    constexpr size_t count = 120;
    constexpr float dt = 0.01f;
    constexpr float tau = 0.2f;
    // The dead time is longer than the baseline window (first 10 % of samples,
    // 0.12 s here) so the baseline is a true pre-step value.
    constexpr float dead_time = 0.15f;
    float times[count] = {};
    float velocities[count] = {};

    for (size_t i = 0; i < count; ++i)
    {
        times[i] = static_cast<float>(i) * dt;
        const float t = times[i] - dead_time;
        velocities[i] = (t <= 0.0f) ? 0.0f : 10.0f * (1.0f - std::exp(-t / tau));
    }

    float fitted_tau = 0.0f;
    float fitted_dead = 0.0f;
    TEST_ASSERT_TRUE(
      tuning::fit_step_dynamics(times, velocities, count, &fitted_tau, &fitted_dead));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, tau, fitted_tau);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, dead_time, fitted_dead);
}

// ============================================================================
// STEP METRICS
// ============================================================================

void test_analyze_step_measures_first_order_response()
{
    constexpr size_t count = 200;
    constexpr float dt = 0.01f;
    constexpr float tau = 0.1f;
    constexpr float target = 10.0f;
    float times[count] = {};
    float velocities[count] = {};

    for (size_t i = 0; i < count; ++i)
    {
        times[i] = static_cast<float>(i) * dt;
        velocities[i] = target * (1.0f - std::exp(-times[i] / tau));
    }

    const tuning::StepMetrics metrics = tuning::analyze_step(times, velocities, count, target);
    TEST_ASSERT_TRUE(metrics.valid);
    // 10-90 % of a first-order response spans tau * ln(9) = 0.2197 s.
    TEST_ASSERT_FLOAT_WITHIN(25.0f, 219.7f, metrics.rise_ms);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, metrics.overshoot_pct);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, metrics.ss_error_pct);
    TEST_ASSERT_TRUE(tuning::accepts(metrics));
}

// A negative target produces the mirrored metrics: same rise, overshoot and
// steady-state error, signed final velocity.
void test_analyze_step_measures_negative_first_order_response()
{
    constexpr size_t count = 200;
    constexpr float dt = 0.01f;
    constexpr float tau = 0.1f;
    constexpr float target = -10.0f;
    float times[count] = {};
    float velocities[count] = {};

    for (size_t i = 0; i < count; ++i)
    {
        times[i] = static_cast<float>(i) * dt;
        velocities[i] = target * (1.0f - std::exp(-times[i] / tau));
    }

    const tuning::StepMetrics metrics = tuning::analyze_step(times, velocities, count, target);
    TEST_ASSERT_TRUE(metrics.valid);
    TEST_ASSERT_FLOAT_WITHIN(25.0f, 219.7f, metrics.rise_ms);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, metrics.overshoot_pct);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, metrics.ss_error_pct);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, target, metrics.final_rad_s);
    TEST_ASSERT_TRUE(tuning::accepts(metrics));
}

void test_analyze_step_measures_overshoot_and_error()
{
    constexpr size_t count = 100;
    float times[count] = {};
    float velocities[count] = {};
    for (size_t i = 0; i < count; ++i)
    {
        times[i] = static_cast<float>(i) * 0.01f;
        velocities[i] = (i < 30) ? 10.0f * static_cast<float>(i) / 30.0f : 9.0f;
        // Sustained overshoot: the metrics smooth the samples with a 50 ms
        // moving average before looking for the peak.
        if (i >= 38 && i < 46)
            velocities[i] = 11.0f;
    }

    const tuning::StepMetrics metrics = tuning::analyze_step(times, velocities, count, 10.0f);
    TEST_ASSERT_TRUE(metrics.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, metrics.overshoot_pct);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, metrics.ss_error_pct);
    TEST_ASSERT_FALSE(tuning::accepts(metrics));
}

// The negative step measures the same overshoot and steady-state error when
// the response is mirrored.
void test_analyze_step_negative_overshoot_and_error()
{
    constexpr size_t count = 100;
    float times[count] = {};
    float velocities[count] = {};
    for (size_t i = 0; i < count; ++i)
    {
        times[i] = static_cast<float>(i) * 0.01f;
        velocities[i] = -((i < 30) ? 10.0f * static_cast<float>(i) / 30.0f : 9.0f);
        if (i >= 38 && i < 46)
            velocities[i] = -11.0f;
    }

    const tuning::StepMetrics metrics = tuning::analyze_step(times, velocities, count, -10.0f);
    TEST_ASSERT_TRUE(metrics.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, metrics.overshoot_pct);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, metrics.ss_error_pct);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -9.0f, metrics.final_rad_s);
    TEST_ASSERT_FALSE(tuning::accepts(metrics));
}

void test_settled_speed_detects_stop_and_surge_at_the_correct_mean()
{
    constexpr size_t count = 100;
    float velocities[count] = {};
    for (size_t i = 0; i < count; ++i) velocities[i] = (i / 10) % 2 == 0 ? 0.0f : -20.0f;

    const auto steady = tuning::measure_settled_speed(velocities, count);
    TEST_ASSERT_TRUE(steady.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -10.0f, steady.mean_rad_s);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, steady.ripple_rad_s);
    TEST_ASSERT_FALSE(tuning::tracks_speed(steady, -10.0f, 0.12f));
}

void test_travel_peak_detects_a_surge_between_window_boundaries()
{
    constexpr size_t count = 201;
    constexpr float dt = 0.01f;
    constexpr float target = 10.0f;
    float times[count] = {};
    float angles[count] = {};
    for (size_t i = 1; i < count; ++i)
    {
        times[i] = i * dt;
        const bool surge = i > 95 && i <= 105;
        angles[i] = angles[i - 1] - (surge ? 2.0f : 1.0f) * target * dt;
    }

    const auto measured = tuning::measure_settled_travel(times, angles, count, 0.1f);
    TEST_ASSERT_TRUE(measured.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -10.5f, measured.mean_rad_s);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -20.0f, measured.min_rad_s);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -10.0f, measured.max_rad_s);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, tuning::peak_speed_error(measured, -target));
}

void test_travel_windows_use_elapsed_time_with_irregular_samples()
{
    const float times[] = {0.0f, 0.07f, 0.1f, 0.13f, 0.21f, 0.37f};
    const float angles[] = {0.0f, 0.7f, 1.0f, 1.3f, 2.1f, 3.7f};
    const auto steady = tuning::measure_settled_travel(times, angles, 6, 0.1f);
    TEST_ASSERT_TRUE(steady.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, steady.mean_rad_s);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, tuning::peak_speed_error(steady, 10.0f));
}

// ============================================================================
// SYNTHESIS AND REFINEMENT
// ============================================================================

// The initial candidate carries the setpoint through the velocity feedforward,
// P trims the residual error and the integral gain starts at KI_RATIO * Kp.
void test_synthesize_uses_velocity_feedforward_and_integral()
{
    tuning::PlantModel plant;
    plant.gain_rad_s_per_duty = 0.02f;  // 50 duty per rad/s
    plant.friction_duty = 150.0f;
    plant.tau_s = 0.2f;
    plant.dead_time_s = 0.02f;
    plant.breakaway_duty = 220.0f;
    plant.valid = true;

    const float target = tuning::choose_validation_target(plant);
    TEST_ASSERT_TRUE(target >= 5.0f && target <= 30.0f);

    const tuning::Gains gains = tuning::synthesize(plant, plant, target);
    // Nominal dimensionless loop gain: Kp = 0.18 * velocity_ff = 9.0, bounded
    // by the step headroom so the step does not start saturated.
    TEST_ASSERT_TRUE(gains.kp >= 1.0f && gains.kp <= 9.1f);
    const float duty_at_step = gains.feedforward + gains.velocity_ff * target + gains.kp * target;
    TEST_ASSERT_TRUE(duty_at_step <= 0.90f * 1023.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 50.0f, gains.velocity_ff);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 150.0f, gains.feedforward);
    // The initial integral gain is proportional to Kp, clamped inside bounds.
    const float expected_ki =
      std::clamp(tuning::KI_RATIO * gains.kp, tuning::KI_MIN_CAL, tuning::KI_MAX_CAL);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, expected_ki, gains.ki);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 220.0f, gains.static_friction);
}

// Each direction gets its own feedforward branch; the PID gains are common and
// bounded so neither step starts saturated.
void test_synthesize_fills_both_directions()
{
    tuning::PlantModel forward_plant;
    forward_plant.gain_rad_s_per_duty = 0.02f;  // 50 duty per rad/s
    forward_plant.friction_duty = 150.0f;
    forward_plant.tau_s = 0.2f;
    forward_plant.breakaway_duty = 220.0f;
    forward_plant.valid = true;

    tuning::PlantModel reverse_plant;
    reverse_plant.gain_rad_s_per_duty = 0.025f;  // 40 duty per rad/s
    reverse_plant.friction_duty = -170.0f;
    reverse_plant.tau_s = 0.2f;
    reverse_plant.breakaway_duty = 240.0f;
    reverse_plant.valid = true;

    const float target = std::fmin(tuning::choose_validation_target(forward_plant),
                                   tuning::choose_validation_target(reverse_plant));
    const tuning::Gains gains = tuning::synthesize(forward_plant, reverse_plant, target);

    TEST_ASSERT_FLOAT_WITHIN(0.5f, 50.0f, gains.velocity_ff);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 150.0f, gains.feedforward);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 220.0f, gains.static_friction);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 40.0f, gains.velocity_ff_rev);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, -170.0f, gains.feedforward_rev);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 240.0f, gains.static_friction_rev);
    // Common Kp bounded by duty headroom in both directions.
    TEST_ASSERT_TRUE(gains.kp >= 1.0f && gains.kp <= 9.1f);
    const float expected_ki =
      std::clamp(tuning::KI_RATIO * gains.kp, tuning::KI_MIN_CAL, tuning::KI_MAX_CAL);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, expected_ki, gains.ki);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, 0.0f, gains.kd);

    const float forward_duty = gains.feedforward + gains.velocity_ff * target + gains.kp * target;
    const float reverse_duty =
      gains.feedforward_rev - gains.velocity_ff_rev * target - gains.kp * target;
    TEST_ASSERT_TRUE(std::fabs(forward_duty) <= 0.90f * 1023.0f);
    TEST_ASSERT_TRUE(std::fabs(reverse_duty) <= 0.90f * 1023.0f);
}

void test_synthesize_requires_both_directions()
{
    tuning::PlantModel plant;
    plant.gain_rad_s_per_duty = 0.02f;
    plant.friction_duty = 150.0f;
    plant.breakaway_duty = 220.0f;
    plant.valid = true;

    tuning::PlantModel invalid;
    const tuning::Gains gains = tuning::synthesize(plant, invalid, 10.0f);
    // No candidate without both identified plants.
    TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, 0.0f, gains.velocity_ff);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, 0.0f, gains.velocity_ff_rev);
}

// The synthesized integral gain always stays inside the calibration bounds.
void test_synthesize_clamps_integral_gain()
{
    tuning::PlantModel low_gain_plant;           // low Kp: integral at its lower bound
    low_gain_plant.gain_rad_s_per_duty = 0.05f;  // 20 duty per rad/s -> Kp = 2.8, 5*Kp = 14 < 20
    low_gain_plant.friction_duty = 150.0f;
    low_gain_plant.breakaway_duty = 220.0f;
    low_gain_plant.valid = true;
    tuning::Gains gains = tuning::synthesize(low_gain_plant, low_gain_plant, 10.0f);
    TEST_ASSERT_TRUE(gains.kp * tuning::KI_RATIO < tuning::KI_MIN_CAL);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, tuning::KI_MIN_CAL, gains.ki);

    tuning::PlantModel high_gain_plant;            // high Kp: integral at its upper bound
    high_gain_plant.gain_rad_s_per_duty = 0.007f;  // ~143 duty per rad/s -> Kp = 20, 5*Kp > 60
    high_gain_plant.friction_duty = 150.0f;
    high_gain_plant.breakaway_duty = 220.0f;
    high_gain_plant.valid = true;
    gains = tuning::synthesize(high_gain_plant, high_gain_plant, 10.0f);
    TEST_ASSERT_TRUE(gains.kp * tuning::KI_RATIO > tuning::KI_MAX_CAL);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, tuning::KI_MAX_CAL, gains.ki);
}

void test_signed_static_curve_intercept_is_kept()
{
    const float duties[2] = {200.0f, 400.0f};
    const float velocities[2] = {5.0f, 9.0f};
    float gain = 0.0f;
    float friction = 0.0f;
    TEST_ASSERT_TRUE(tuning::fit_static_curve(duties, velocities, 2, &gain, &friction));
    // v = 0.02 * duty + 1 -> the curve crosses zero at duty -50.
    TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, 0.02f, gain);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, -50.0f, friction);
}

void test_validation_target_stays_inside_the_duty_range()
{
    tuning::PlantModel plant;
    plant.gain_rad_s_per_duty = 0.0178f;  // weak plant: 56 duty per rad/s
    plant.friction_duty = 175.0f;
    plant.valid = true;
    const float target = tuning::choose_validation_target(plant);
    // (0.90 * 1023 - 60 - 175) * 0.0178 = 12.21 rad/s.
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.21f, target);
}

// The signed intercept consumes the same duty budget in both directions.
void test_validation_target_uses_the_intercept_magnitude()
{
    tuning::PlantModel plant;
    plant.gain_rad_s_per_duty = 0.0178f;
    plant.friction_duty = -175.0f;
    plant.valid = true;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.21f, tuning::choose_validation_target(plant));
}

void test_validation_target_stays_inside_the_identified_range()
{
    tuning::PlantModel plant;
    plant.gain_rad_s_per_duty = 0.02f;
    plant.friction_duty = 150.0f;
    plant.tau_s = 0.2f;
    plant.valid = true;
    // The measured range caps the validation target at 0.95 * 8 rad/s.
    plant.max_velocity_rad_s = 8.0f;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 7.6f, tuning::choose_validation_target(plant));

    plant.max_velocity_rad_s = 0.0f;  // no identified range: no cap
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 14.21f, tuning::choose_validation_target(plant));
}

void test_saturated_curve_validates_near_its_lower_point()
{
    tuning::PlantModel plant;
    plant.gain_rad_s_per_duty = 0.02f;  // flat curve, near saturation
    plant.friction_duty = 100.0f;
    plant.tau_s = 0.4f;
    plant.valid = true;
    plant.max_velocity_rad_s = 12.7f;
    plant.min_velocity_rad_s = 11.2f;
    // Duty headroom allows 10.1 rad/s, but the two operating points are too
    // close: the target goes to 0.55 * 11.2 = 6.16 rad/s.
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 6.16f, tuning::choose_validation_target(plant));
}

// Refinement: overshoot damps P, steady-state error raises the integral gain
// (the feedforward is untouched while the integral can still grow) and a slow
// rise raises P.
void test_refine_damps_overshoot_and_raises_integral()
{
    tuning::PlantModel plant;
    plant.gain_rad_s_per_duty = 0.02f;
    plant.tau_s = 0.2f;
    plant.valid = true;

    tuning::StepMetrics good;
    good.valid = true;
    good.rise_ms = 300.0f;
    good.overshoot_pct = 1.0f;
    good.ss_error_pct = 1.0f;

    tuning::Gains gains = tuning::synthesize(plant, plant, 10.0f);
    const float initial_ki = gains.ki;

    tuning::StepMetrics overshoot;
    overshoot.valid = true;
    overshoot.rise_ms = 300.0f;
    overshoot.overshoot_pct = 10.0f;
    overshoot.ss_error_pct = 1.0f;
    const float aggressive_kp = gains.kp;
    gains = tuning::refine(gains, overshoot, good);
    TEST_ASSERT_TRUE(gains.kp < aggressive_kp);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, initial_ki, gains.ki);

    // A forward steady-state error raises the common integral gain; the
    // per-direction velocity feedforwards stay untouched.
    tuning::StepMetrics steady_error;
    steady_error.valid = true;
    steady_error.rise_ms = 300.0f;
    steady_error.overshoot_pct = 1.0f;
    steady_error.ss_error_pct = 10.0f;
    steady_error.final_rad_s = 11.0f;
    steady_error.target_rad_s = 10.0f;
    const float velocity_ff_before = gains.velocity_ff;
    const float velocity_ff_rev_before = gains.velocity_ff_rev;
    const float ki_before = gains.ki;
    gains = tuning::refine(gains, steady_error, good);
    TEST_ASSERT_TRUE(gains.ki > ki_before);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, velocity_ff_before, gains.velocity_ff);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, velocity_ff_rev_before, gains.velocity_ff_rev);

    tuning::StepMetrics slow;
    slow.valid = true;
    slow.rise_ms = 900.0f;
    slow.overshoot_pct = 1.0f;
    slow.ss_error_pct = 1.0f;
    const float slow_kp = gains.kp;
    gains = tuning::refine(gains, slow, good);
    TEST_ASSERT_TRUE(gains.kp > slow_kp);
}

// A reverse steady-state error also raises the common integral gain: the
// integral closes the error in either direction without trimming the branches.
void test_refine_raises_integral_on_reverse_error()
{
    tuning::PlantModel plant;
    plant.gain_rad_s_per_duty = 0.02f;
    plant.tau_s = 0.2f;
    plant.valid = true;

    tuning::StepMetrics good;
    good.valid = true;
    good.rise_ms = 300.0f;
    good.overshoot_pct = 1.0f;
    good.ss_error_pct = 1.0f;

    tuning::StepMetrics reverse_error;
    reverse_error.valid = true;
    reverse_error.rise_ms = 300.0f;
    reverse_error.overshoot_pct = 1.0f;
    reverse_error.ss_error_pct = 20.0f;
    reverse_error.final_rad_s = -8.0f;
    reverse_error.target_rad_s = -10.0f;

    tuning::Gains gains = tuning::synthesize(plant, plant, 10.0f);
    const float velocity_ff_before = gains.velocity_ff;
    const float reverse_before = gains.velocity_ff_rev;
    const float ki_before = gains.ki;
    gains = tuning::refine(gains, good, reverse_error);
    TEST_ASSERT_TRUE(gains.ki > ki_before);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, velocity_ff_before, gains.velocity_ff);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, reverse_before, gains.velocity_ff_rev);
}

// Once the integral gain is at its cap, a steady-state error falls back to
// trimming the velocity feedforward of the direction that misses the target.
void test_refine_trims_feedforward_when_integral_is_at_cap()
{
    tuning::PlantModel plant;
    plant.gain_rad_s_per_duty = 0.02f;
    plant.tau_s = 0.2f;
    plant.valid = true;

    tuning::StepMetrics good;
    good.valid = true;
    good.rise_ms = 300.0f;
    good.overshoot_pct = 1.0f;
    good.ss_error_pct = 1.0f;

    tuning::StepMetrics steady_error;
    steady_error.valid = true;
    steady_error.rise_ms = 300.0f;
    steady_error.overshoot_pct = 1.0f;
    steady_error.ss_error_pct = 10.0f;
    steady_error.final_rad_s = 11.0f;
    steady_error.target_rad_s = 10.0f;

    tuning::Gains gains = tuning::synthesize(plant, plant, 10.0f);
    gains.ki = tuning::KI_MAX_CAL;
    const float velocity_ff_before = gains.velocity_ff;
    gains = tuning::refine(gains, steady_error, good);
    TEST_ASSERT_TRUE(gains.velocity_ff < velocity_ff_before);
}

// An invalid capture in either direction is treated as too aggressive: the
// common Kp comes down.
void test_refine_damps_when_a_direction_capture_fails()
{
    tuning::PlantModel plant;
    plant.gain_rad_s_per_duty = 0.02f;
    plant.tau_s = 0.2f;
    plant.valid = true;

    tuning::Gains gains = tuning::synthesize(plant, plant, 10.0f);
    tuning::StepMetrics good;
    good.valid = true;
    good.rise_ms = 300.0f;
    good.overshoot_pct = 1.0f;
    good.ss_error_pct = 1.0f;

    const float kp_before = gains.kp;
    const tuning::StepMetrics invalid;
    gains = tuning::refine(gains, good, invalid);
    TEST_ASSERT_TRUE(gains.kp < kp_before);
}

// ============================================================================
// LOW-SPEED VALIDATION AND INTEGRAL ADJUSTMENT
// ============================================================================

void test_low_speed_acceptance_limits()
{
    tuning::StepMetrics metrics;
    metrics.valid = true;
    metrics.rise_ms = 1000.0f;
    metrics.overshoot_pct = 4.0f;
    metrics.ss_error_pct = 3.0f;
    TEST_ASSERT_TRUE(tuning::accepts_low_speed(metrics));

    metrics.rise_ms = tuning::LOW_SPEED_RISE_MS_LIMIT + 100.0f;
    TEST_ASSERT_FALSE(tuning::accepts_low_speed(metrics));

    metrics.rise_ms = tuning::LOW_SPEED_RISE_MS_LIMIT;
    // The wheel may pulse-start at low speed: up to 25 % overshoot passes.
    metrics.overshoot_pct = 15.0f;
    TEST_ASSERT_TRUE(tuning::accepts_low_speed(metrics));

    metrics.overshoot_pct = tuning::LOW_SPEED_OVERSHOOT_PCT_LIMIT + 5.0f;
    TEST_ASSERT_FALSE(tuning::accepts_low_speed(metrics));

    metrics.overshoot_pct = 4.0f;
    metrics.ss_error_pct = tuning::LOW_SPEED_SS_ERROR_PCT_LIMIT + 1.0f;
    TEST_ASSERT_FALSE(tuning::accepts_low_speed(metrics));

    metrics.ss_error_pct = tuning::LOW_SPEED_SS_ERROR_PCT_LIMIT;
    TEST_ASSERT_TRUE(tuning::accepts_low_speed(metrics));

    metrics.valid = false;
    TEST_ASSERT_FALSE(tuning::accepts_low_speed(metrics));
}

void test_adjust_low_speed_integral()
{
    tuning::PlantModel plant;
    plant.gain_rad_s_per_duty = 0.02f;
    plant.tau_s = 0.2f;
    plant.valid = true;

    tuning::StepMetrics good;
    good.valid = true;
    good.rise_ms = 300.0f;
    good.overshoot_pct = 1.0f;
    good.ss_error_pct = 1.0f;

    // A low-speed steady-state error needs more integral action.
    tuning::StepMetrics steady_error;
    steady_error.valid = true;
    steady_error.rise_ms = 1000.0f;
    steady_error.overshoot_pct = 1.0f;
    steady_error.ss_error_pct = 10.0f;

    tuning::Gains gains = tuning::synthesize(plant, plant, 10.0f);
    const float initial_ki = gains.ki;
    gains = tuning::adjust_low_speed_integral(gains, steady_error, good);
    TEST_ASSERT_TRUE(gains.ki > initial_ki);

    // A low-speed overshoot backs it off, but never below the calibration floor.
    tuning::StepMetrics overshoot;
    overshoot.valid = true;
    overshoot.rise_ms = 1000.0f;
    overshoot.overshoot_pct = tuning::LOW_SPEED_OVERSHOOT_PCT_LIMIT + 5.0f;
    overshoot.ss_error_pct = 1.0f;

    tuning::Gains over = tuning::synthesize(plant, plant, 10.0f);
    over.ki = tuning::KI_MAX_CAL;
    over = tuning::adjust_low_speed_integral(over, overshoot, good);
    TEST_ASSERT_TRUE(over.ki < tuning::KI_MAX_CAL);

    tuning::Gains floored = tuning::synthesize(plant, plant, 10.0f);
    floored.ki = tuning::KI_MIN_CAL;
    floored = tuning::adjust_low_speed_integral(floored, overshoot, good);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, tuning::KI_MIN_CAL, floored.ki);

    // A failed capture is treated like too much integral action.
    const tuning::StepMetrics invalid;
    tuning::Gains capture = tuning::synthesize(plant, plant, 10.0f);
    const float capture_ki = capture.ki;
    capture = tuning::adjust_low_speed_integral(capture, invalid, good);
    TEST_ASSERT_TRUE(capture.ki < capture_ki);
}

void test_acceptance_limits()
{
    tuning::StepMetrics metrics;
    metrics.valid = true;
    metrics.rise_ms = 300.0f;
    metrics.overshoot_pct = 4.0f;
    metrics.ss_error_pct = 3.0f;
    TEST_ASSERT_TRUE(tuning::accepts(metrics));
    TEST_ASSERT_TRUE(tuning::accepts_best_effort(metrics));

    metrics.overshoot_pct = 9.0f;
    TEST_ASSERT_FALSE(tuning::accepts(metrics));
    TEST_ASSERT_TRUE(tuning::accepts_best_effort(metrics));

    metrics.overshoot_pct = tuning::FALLBACK_OVERSHOOT_PCT_LIMIT + 1.0f;
    TEST_ASSERT_FALSE(tuning::accepts_best_effort(metrics));

    metrics.valid = false;
    TEST_ASSERT_FALSE(tuning::accepts(metrics));
    TEST_ASSERT_TRUE(tuning::step_cost(metrics) > 1.0e5f);
}

// A wheel is calibrated only when both directions pass; the relaxed limits are
// paired the same way.
void test_paired_acceptance()
{
    tuning::StepMetrics forward;
    forward.valid = true;
    forward.rise_ms = 300.0f;
    forward.overshoot_pct = 4.0f;
    forward.ss_error_pct = 3.0f;
    tuning::StepMetrics reverse = forward;
    reverse.target_rad_s = -10.0f;
    reverse.final_rad_s = -9.7f;

    TEST_ASSERT_TRUE(tuning::accepts_pair(forward, reverse));
    TEST_ASSERT_TRUE(tuning::accepts_best_effort_pair(forward, reverse));

    tuning::StepMetrics bad_reverse = reverse;
    bad_reverse.ss_error_pct = 9.0f;
    TEST_ASSERT_FALSE(tuning::accepts_pair(forward, bad_reverse));
    TEST_ASSERT_TRUE(tuning::accepts_best_effort_pair(forward, bad_reverse));

    bad_reverse.overshoot_pct = tuning::FALLBACK_OVERSHOOT_PCT_LIMIT + 1.0f;
    TEST_ASSERT_FALSE(tuning::accepts_best_effort_pair(forward, bad_reverse));

    bad_reverse = reverse;
    bad_reverse.settled_ripple_pct = tuning::SETTLED_RIPPLE_PCT_LIMIT + 1.0f;
    TEST_ASSERT_FALSE(tuning::accepts_pair(forward, bad_reverse));
    TEST_ASSERT_FALSE(tuning::accepts_best_effort_pair(forward, bad_reverse));
}

// ============================================================================
// CALIBRATED MAXIMUM SPEED LIMIT
// ============================================================================

void test_calibrated_max_rpm_equals_the_slowest_wheel()
{
    const uint16_t maxima[4] = {1750, 1700, 1650, 1380};
    TEST_ASSERT_EQUAL_UINT16(1380, tuning::calibrated_max_rpm(maxima, 4));
}

void test_calibrated_max_rpm_keeps_tenths_of_an_rpm()
{
    const uint16_t maxima[4] = {1750, 1700, 1650, 1333};
    TEST_ASSERT_EQUAL_UINT16(1333, tuning::calibrated_max_rpm(maxima, 4));

    const uint16_t exact[1] = {1240};
    TEST_ASSERT_EQUAL_UINT16(1240, tuning::calibrated_max_rpm(exact, 1));
}

// A wheel without a measured maximum prevents deriving the common limit: the
// caller keeps the stored or fallback limit.
void test_calibrated_max_rpm_requires_every_wheel()
{
    const uint16_t missing[4] = {1750, 0, 1650, 1380};
    TEST_ASSERT_EQUAL_UINT16(0, tuning::calibrated_max_rpm(missing, 4));
    TEST_ASSERT_EQUAL_UINT16(0, tuning::calibrated_max_rpm(nullptr, 4));
    TEST_ASSERT_EQUAL_UINT16(0, tuning::calibrated_max_rpm(missing, 0));
}

void test_shared_speed_checks_target_and_spread_in_both_directions()
{
    std::array<float, 8> means = {98.0f, 99.0f, 100.0f, 101.0f, 100.0f, 99.0f, 101.0f, 103.0f};
    TEST_ASSERT_EQUAL_UINT8(0, tuning::unmatched_speed_mask(means, 100.0f));

    means[7] = 104.0f;
    TEST_ASSERT_EQUAL_UINT8(0x80, tuning::unmatched_speed_mask(means, 100.0f));

    means[0] = 90.0f;
    means[4] = 110.0f;
    TEST_ASSERT_EQUAL_UINT8(0x11, tuning::unmatched_speed_mask(means, 100.0f));
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_static_curve_fits_gain_and_friction);
    RUN_TEST(test_reverse_static_curve_fits_signed_intercept);
    RUN_TEST(test_static_curve_rejects_a_stalled_wheel);
    RUN_TEST(test_plant_line_through_high_and_low_operating_points);
    RUN_TEST(test_plant_line_rejects_degenerate_operating_points);
    RUN_TEST(test_median_filter_step_rejects_an_outlier_and_keeps_endpoints);
    RUN_TEST(test_step_dynamics_recovers_time_constant_and_dead_time);
    RUN_TEST(test_analyze_step_measures_first_order_response);
    RUN_TEST(test_analyze_step_measures_negative_first_order_response);
    RUN_TEST(test_analyze_step_measures_overshoot_and_error);
    RUN_TEST(test_analyze_step_negative_overshoot_and_error);
    RUN_TEST(test_settled_speed_detects_stop_and_surge_at_the_correct_mean);
    RUN_TEST(test_travel_peak_detects_a_surge_between_window_boundaries);
    RUN_TEST(test_travel_windows_use_elapsed_time_with_irregular_samples);
    RUN_TEST(test_synthesize_uses_velocity_feedforward_and_integral);
    RUN_TEST(test_synthesize_fills_both_directions);
    RUN_TEST(test_synthesize_requires_both_directions);
    RUN_TEST(test_synthesize_clamps_integral_gain);
    RUN_TEST(test_signed_static_curve_intercept_is_kept);
    RUN_TEST(test_validation_target_stays_inside_the_duty_range);
    RUN_TEST(test_validation_target_uses_the_intercept_magnitude);
    RUN_TEST(test_validation_target_stays_inside_the_identified_range);
    RUN_TEST(test_saturated_curve_validates_near_its_lower_point);
    RUN_TEST(test_refine_damps_overshoot_and_raises_integral);
    RUN_TEST(test_refine_raises_integral_on_reverse_error);
    RUN_TEST(test_refine_trims_feedforward_when_integral_is_at_cap);
    RUN_TEST(test_refine_damps_when_a_direction_capture_fails);
    RUN_TEST(test_low_speed_acceptance_limits);
    RUN_TEST(test_adjust_low_speed_integral);
    RUN_TEST(test_acceptance_limits);
    RUN_TEST(test_paired_acceptance);
    RUN_TEST(test_calibrated_max_rpm_equals_the_slowest_wheel);
    RUN_TEST(test_calibrated_max_rpm_keeps_tenths_of_an_rpm);
    RUN_TEST(test_calibrated_max_rpm_requires_every_wheel);
    RUN_TEST(test_shared_speed_checks_target_and_spread_in_both_directions);
    UNITY_END();
}

extern "C" void app_main(void)
{
    setup();
}
