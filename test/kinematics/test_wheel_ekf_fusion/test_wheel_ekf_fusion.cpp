#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <unity.h>

#include <cmath>

#include "helper_func.h"
#include "wheel_ekf.h"

// The filter derives dt from the wall clock, so each simulated sample is one
// tick (10 ms at CONFIG_FREERTOS_HZ=100) apart.
static void tick()
{
    vTaskDelay(pdMS_TO_TICKS(10));
}

// The first sample seeds the state instead of correcting it.
void test_first_sample_seeds_angle_and_velocity()
{
    WheelKalmanFilter filter;
    filter.update(1.0, 2.0);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, static_cast<float>(filter.get_angle_rad()));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, static_cast<float>(filter.get_velocity_rad_s()));
}

// Without a velocity row the velocity is inferred from the angle slope only.
void test_angle_update_tracks_angle()
{
    WheelKalmanFilter filter;

    double angle = 0.0;
    for (int i = 0; i < 40; ++i)
    {
        tick();
        angle += 0.02;
        filter.update(angle);
    }

    // The angle-only path trails a constant ramp by (1-K)/K steps.
    TEST_ASSERT_FLOAT_WITHIN(
      0.3f, static_cast<float>(angle), static_cast<float>(filter.get_angle_rad()));
    TEST_ASSERT_TRUE(std::fabs(filter.get_velocity_rad_s()) < 10.0);
}

// The fused call applies both rows in one predict/update cycle: the angle
// follows the encoder slope while the velocity locks onto the measurement.
void test_fused_update_tracks_constant_velocity()
{
    WheelKalmanFilter filter;

    double angle = 0.0;
    for (int i = 0; i < 20; ++i)
    {
        tick();
        angle += 0.02;  // 2 rad/s across the 10 ms tick
        filter.update(angle, 2.0);
    }

    TEST_ASSERT_FLOAT_WITHIN(0.2f, 2.0f, static_cast<float>(filter.get_velocity_rad_s()));
    TEST_ASSERT_FLOAT_WITHIN(
      0.1f, static_cast<float>(angle), static_cast<float>(filter.get_angle_rad()));
}

void test_fused_update_negative_velocity()
{
    WheelKalmanFilter filter;

    double angle = 0.0;
    for (int i = 0; i < 20; ++i)
    {
        tick();
        angle -= 0.03;
        filter.update(angle, -3.0);
    }

    TEST_ASSERT_FLOAT_WITHIN(0.2f, -3.0f, static_cast<float>(filter.get_velocity_rad_s()));
}

void test_filter_reset_clears_the_estimate()
{
    WheelKalmanFilter filter;

    double angle = 0.0;
    for (int i = 0; i < 10; ++i)
    {
        tick();
        angle += 0.05;
        filter.update(angle, 5.0);
    }
    filter.reset();

    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, static_cast<float>(filter.get_velocity_rad_s()));
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, static_cast<float>(filter.get_angle_rad()));
}

// A zero velocity measurement must pull the estimate back to rest instead of
// holding the stale value.
void test_zero_velocity_anchor()
{
    WheelKalmanFilter filter;

    double angle = 0.0;
    for (int i = 0; i < 10; ++i)
    {
        tick();
        angle += 0.05;
        filter.update(angle, 5.0);
    }
    for (int i = 0; i < 30; ++i)
    {
        tick();
        filter.update(angle, 0.0);
    }

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, static_cast<float>(filter.get_velocity_rad_s()));
}

// Angle-only samples also estimate velocity from the shaft slope, but the
// slope path converges much slower than direct velocity measurements. Both
// filters see the same angle samples; one additionally gets the true velocity.
void test_motor_velocity_dominates_encoder_slope()
{
    WheelKalmanFilter slope_only;
    WheelKalmanFilter slope_plus_feedback;

    const double true_velocity = 2.0;
    double angle = 0.0;

    for (int i = 0; i <= 40; ++i)
    {
        if (i == 20)
        {
            // Mid-run: the slope-only estimate must still be behind.
            TEST_ASSERT_TRUE(std::fabs(slope_plus_feedback.get_velocity_rad_s() - true_velocity) <
                             std::fabs(slope_only.get_velocity_rad_s() - true_velocity));
        }
        tick();
        angle += true_velocity * 0.01;
        slope_only.update(angle);
        slope_plus_feedback.update(angle, true_velocity);
    }

    const double slope_error = std::fabs(slope_only.get_velocity_rad_s() - true_velocity);
    const double feedback_error =
      std::fabs(slope_plus_feedback.get_velocity_rad_s() - true_velocity);

    TEST_ASSERT_TRUE(feedback_error <= slope_error);
    TEST_ASSERT_FLOAT_WITHIN(0.2f,
                             static_cast<float>(true_velocity),
                             static_cast<float>(slope_plus_feedback.get_velocity_rad_s()));
}

// The angle path keeps its wrap handling with the two-row measurement model.
void test_angle_update_wraps_over_pi()
{
    WheelKalmanFilter filter;

    for (int i = 0; i < 30; ++i)
    {
        tick();
        filter.update(3.0 + 0.01 * i);
    }
    // Jump past +PI, then keep sampling: the estimate must settle on the new
    // branch through the wrap instead of winding.
    tick();
    filter.update(-3.13);
    for (int i = 0; i < 10; ++i)
    {
        tick();
        filter.update(-3.13 + 0.01 * i);
    }

    const float wrap_error = static_cast<float>(normalize_angle(filter.get_angle_rad() - (-3.13)));
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 0.0f, wrap_error);
}

// The estimator scales the encoder angle down by the gear ratio, so in the
// wheel frame one absolute encoder turn covers 2*pi/3. The filter must wrap
// its angle innovations at that period; wrapping at 2*pi lets the angle slope
// estimate wind up and destroys the velocity estimate.
void test_angle_update_wraps_at_wheel_period()
{
    constexpr double WHEEL_PERIOD = 2.0 * M_PI / 3.0;
    WheelKalmanFilter filter(WHEEL_PERIOD);

    double wheel_angle = 0.0;
    for (int i = 0; i < 100; ++i)
    {
        tick();
        wheel_angle += 0.05;  // 5 rad/s across the 10 ms tick
        const double wrapped = wheel_angle - WHEEL_PERIOD * std::floor(wheel_angle / WHEEL_PERIOD);
        filter.update(wrapped);
    }

    TEST_ASSERT_FLOAT_WITHIN(0.5f, 5.0f, static_cast<float>(filter.get_velocity_rad_s()));
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_first_sample_seeds_angle_and_velocity);
    RUN_TEST(test_angle_update_tracks_angle);
    RUN_TEST(test_fused_update_tracks_constant_velocity);
    RUN_TEST(test_fused_update_negative_velocity);
    RUN_TEST(test_filter_reset_clears_the_estimate);
    RUN_TEST(test_zero_velocity_anchor);
    RUN_TEST(test_motor_velocity_dominates_encoder_slope);
    RUN_TEST(test_angle_update_wraps_over_pi);
    RUN_TEST(test_angle_update_wraps_at_wheel_period);
    UNITY_END();
}

extern "C" void app_main(void)
{
    setup();
}
