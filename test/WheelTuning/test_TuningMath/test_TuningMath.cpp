#include <unity.h>

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

void test_static_curve_rejects_a_stalled_wheel()
{
    const float duties[2] = {200.0f, 400.0f};
    const float velocities[2] = {0.0f, 0.4f};
    float gain = 0.0f;
    float friction = 0.0f;

    TEST_ASSERT_FALSE(tuning::fit_static_curve(duties, velocities, 2, &gain, &friction));
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

// ============================================================================
// SYNTHESIS AND REFINEMENT
// ============================================================================

void test_synthesize_uses_velocity_feedforward_and_no_integral()
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

    const tuning::Gains gains = tuning::synthesize(plant, target);
    // Nominal dimensionless loop gain: Kp = 0.06 * velocity_ff = 3.0.
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 3.0f, gains.kp);
    const float duty_at_step = gains.feedforward + gains.velocity_ff * target + gains.kp * target;
    TEST_ASSERT_TRUE(duty_at_step <= 0.75f * 1023.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 50.0f, gains.velocity_ff);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 150.0f, gains.feedforward);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, 0.0f, gains.ki);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 220.0f, gains.static_friction);
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
    // (0.65 * 1023 - 60 - 175) / 56.18 = 7.65 rad/s.
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 7.65f, target);
}

void test_validation_target_stays_inside_the_identified_range()
{
    tuning::PlantModel plant;
    plant.gain_rad_s_per_duty = 0.02f;
    plant.friction_duty = 150.0f;
    plant.tau_s = 0.2f;
    plant.valid = true;
    // Duty headroom allows 9.1 rad/s, but the identification stopped at 8 rad/s:
    // the validation target must stay at 0.80 * 8 = 6.4 rad/s.
    plant.max_velocity_rad_s = 8.0f;
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 6.4f, tuning::choose_validation_target(plant));

    plant.max_velocity_rad_s = 0.0f;  // no identified range: no cap
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 9.1f, tuning::choose_validation_target(plant));
}

void test_saturated_curve_validates_near_its_lower_point()
{
    tuning::PlantModel plant;
    plant.gain_rad_s_per_duty = 0.006f;  // flat curve, near saturation
    plant.friction_duty = -600.0f;
    plant.tau_s = 0.4f;
    plant.valid = true;
    plant.max_velocity_rad_s = 12.7f;
    plant.min_velocity_rad_s = 11.2f;
    // The two operating points are too close: the target goes to 0.55 * 11.2.
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 6.16f, tuning::choose_validation_target(plant));
}

void test_refine_damps_overshoot_and_adds_integral()
{
    tuning::PlantModel plant;
    plant.gain_rad_s_per_duty = 0.02f;
    plant.tau_s = 0.2f;
    plant.valid = true;

    tuning::Gains gains = tuning::synthesize(plant, 10.0f);

    tuning::StepMetrics overshoot;
    overshoot.valid = true;
    overshoot.rise_ms = 300.0f;
    overshoot.overshoot_pct = 10.0f;
    overshoot.ss_error_pct = 1.0f;
    const float aggressive_kp = gains.kp;
    gains = tuning::refine(plant, gains, overshoot);
    TEST_ASSERT_TRUE(gains.kp < aggressive_kp);

    tuning::StepMetrics steady_error;
    steady_error.valid = true;
    steady_error.rise_ms = 300.0f;
    steady_error.overshoot_pct = 1.0f;
    steady_error.ss_error_pct = 10.0f;
    steady_error.final_rad_s = 11.0f;
    steady_error.target_rad_s = 10.0f;
    const float feedforward_before = gains.velocity_ff;
    gains = tuning::refine(plant, gains, steady_error);
    // A positive steady-state error trims the velocity feedforward down.
    TEST_ASSERT_TRUE(gains.velocity_ff < feedforward_before);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, 0.0f, gains.ki);

    tuning::StepMetrics slow;
    slow.valid = true;
    slow.rise_ms = 900.0f;
    slow.overshoot_pct = 1.0f;
    slow.ss_error_pct = 1.0f;
    const float slow_kp = gains.kp;
    gains = tuning::refine(plant, gains, slow);
    TEST_ASSERT_TRUE(gains.kp > slow_kp);
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

    metrics.overshoot_pct = 13.0f;
    TEST_ASSERT_FALSE(tuning::accepts_best_effort(metrics));

    metrics.valid = false;
    TEST_ASSERT_FALSE(tuning::accepts(metrics));
    TEST_ASSERT_TRUE(tuning::step_cost(metrics) > 1.0e5f);
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_static_curve_fits_gain_and_friction);
    RUN_TEST(test_static_curve_rejects_a_stalled_wheel);
    RUN_TEST(test_step_dynamics_recovers_time_constant_and_dead_time);
    RUN_TEST(test_analyze_step_measures_first_order_response);
    RUN_TEST(test_analyze_step_measures_overshoot_and_error);
    RUN_TEST(test_synthesize_uses_velocity_feedforward_and_no_integral);
    RUN_TEST(test_signed_static_curve_intercept_is_kept);
    RUN_TEST(test_validation_target_stays_inside_the_duty_range);
    RUN_TEST(test_validation_target_stays_inside_the_identified_range);
    RUN_TEST(test_saturated_curve_validates_near_its_lower_point);
    RUN_TEST(test_refine_damps_overshoot_and_adds_integral);
    RUN_TEST(test_acceptance_limits);
    UNITY_END();
}

extern "C" void app_main(void)
{
    setup();
}
