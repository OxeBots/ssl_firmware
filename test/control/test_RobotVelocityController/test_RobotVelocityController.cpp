#include <unity.h>

#include <cmath>

#include "RobotVelocityController.h"
#include "omni_robot.h"

static HealthState healthy_state()
{
    HealthState health = {};
    health.severity = HealthSeverity::HEALTHY;
    health.kick_inhibited = false;
    return health;
}

void test_velocity_controller_limits_body_velocity_and_wheels_with_health_warnings()
{
    OmnidirectionalRobot robot(config::kinematic::OMNI_WHEEL_RADIUS,
                               config::kinematic::OMNI_WHEEL_DISTANCE);
    VelocityControlConfig config = {};
    config.max_vx_m_s = 0.5f;
    config.max_wheel_velocity_rad_s = 10.0f;
    RobotVelocityController controller(robot, config);

    GoalPose goal = {};
    goal.kick_velocity = 25;
    goal.kick_inhibited = false;
    goal.command_sequence = 4;
    goal.received_at_us = 50;
    HealthState health = healthy_state();
    health.severity = HealthSeverity::WARNING;
    health.active_faults = ERROR_MOTOR0_FAULT | ERROR_ENCODER_INVALID | ERROR_BATT_OVERVOLT;

    BodyVelocity desired = {};
    desired.vx_m_s = 5.0f;

    const ActuationSetpoint output = controller.compute(desired, goal, health);
    TEST_ASSERT_FALSE(output.motion_inhibited);
    TEST_ASSERT_FALSE(output.kick_inhibited);
    TEST_ASSERT_EQUAL_UINT8(25, output.kick_velocity);
    TEST_ASSERT_EQUAL_UINT32(4, output.command_sequence);
    TEST_ASSERT_TRUE(output.timestamp_us == 50);
    for (float velocity : output.wheel_velocity_rad_s)
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, std::fabs(velocity));
}

void test_velocity_controller_zeroes_output_when_motion_inhibited()
{
    OmnidirectionalRobot robot(config::kinematic::OMNI_WHEEL_RADIUS,
                               config::kinematic::OMNI_WHEEL_DISTANCE);
    RobotVelocityController controller(robot);

    GoalPose goal = {};
    goal.motion_inhibited = true;
    BodyVelocity desired = {};
    desired.vx_m_s = 1.0f;

    const ActuationSetpoint output = controller.compute(desired, goal, healthy_state());
    TEST_ASSERT_TRUE(output.motion_inhibited);
    TEST_ASSERT_TRUE(output.kick_inhibited);
    for (float velocity : output.wheel_velocity_rad_s)
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, velocity);
}

void test_diagonal_velocity_keeps_stationary_wheel_targets_at_zero()
{
    OmnidirectionalRobot robot(config::kinematic::OMNI_WHEEL_RADIUS,
                               config::kinematic::OMNI_WHEEL_DISTANCE);
    RobotVelocityController controller(robot);
    BodyVelocity desired = {};
    desired.vx_m_s = 0.097f;
    desired.vy_m_s = 0.097f;

    const ActuationSetpoint output = controller.compute(desired, {}, healthy_state());
    TEST_ASSERT_TRUE(output.wheel_velocity_rad_s[0] == 0.0f);
    TEST_ASSERT_TRUE(output.wheel_velocity_rad_s[1] < -4.0f);
    TEST_ASSERT_TRUE(output.wheel_velocity_rad_s[2] == 0.0f);
    TEST_ASSERT_TRUE(output.wheel_velocity_rad_s[3] > 4.0f);
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_velocity_controller_limits_body_velocity_and_wheels_with_health_warnings);
    RUN_TEST(test_velocity_controller_zeroes_output_when_motion_inhibited);
    RUN_TEST(test_diagonal_velocity_keeps_stationary_wheel_targets_at_zero);
    UNITY_END();
}

extern "C" void app_main(void)
{
    setup();
}
