#include <unity.h>

#include "RobotStateEstimator.h"
#include "omni_robot.h"

static HealthState healthy_state()
{
    HealthState health = {};
    health.severity = HealthSeverity::HEALTHY;
    health.kick_inhibited = false;
    return health;
}

void test_robot_state_estimator_integrates_odometry_and_accepts_vision_correction()
{
    OmnidirectionalRobot robot(config::kinematic::OMNI_WHEEL_RADIUS,
                               config::kinematic::OMNI_WHEEL_DISTANCE);
    RobotStateEstimator estimator(robot);
    FusionInputFrame input = {};
    input.wheels.valid_mask = 0x0F;
    input.health = healthy_state();
    input.timestamp_us = 1000000;

    const vt::numeric_vector<4> wheel_velocity =
      robot.compute_wheel_velocities(vt::make_numeric_vector<3>({0.0, 1.0, 0.0}));
    for (size_t index = 0; index < ROBOT_WHEEL_COUNT; ++index)
        input.wheels.velocity_rad_s[index] = static_cast<float>(wheel_velocity(index));

    RobotStateEstimate state = estimator.update(input);
    TEST_ASSERT_TRUE(state.valid);
    input.timestamp_us = 1100000;
    state = estimator.update(input);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.1f, state.pose.x_m);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, state.body_velocity.vx_m_s);

    input.base_station_pose.valid = true;
    input.base_station_pose.pose = {2.0f, -1.0f, 0.5f};
    input.timestamp_us = 1200000;
    state = estimator.update(input);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, state.pose.x_m);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, state.pose.y_m);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, state.pose.heading_rad);
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_robot_state_estimator_integrates_odometry_and_accepts_vision_correction);
    UNITY_END();
}

extern "C" void app_main(void)
{
    setup();
}
