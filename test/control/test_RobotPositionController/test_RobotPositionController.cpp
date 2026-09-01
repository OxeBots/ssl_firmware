#include <unity.h>

#include "RobotPositionController.h"

void test_position_controller_converts_pose_error_to_body_velocity()
{
    PositionControlConfig config = {};
    config.position_gain = 2.0f;
    config.heading_gain = 3.0f;
    RobotPositionController controller(config);

    RobotStateEstimate state = {};
    GoalPose goal = {};
    goal.pose.x_m = 1.0f;
    goal.velocity.vx_m_s = 0.5f;

    const BodyVelocity velocity = controller.compute(state, goal);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.5f, velocity.vx_m_s);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, velocity.vy_m_s);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, velocity.omega_rad_s);

    // Target ahead of a robot facing +y must map to forward body velocity and
    // a negative heading correction back to zero yaw.
    state.pose = {1.0f, 1.0f, 1.5707963f};
    goal.pose = {1.0f, 2.0f, 0.0f};
    goal.velocity = {};
    const BodyVelocity rotated = controller.compute(state, goal);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, rotated.vx_m_s);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, rotated.vy_m_s);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -4.712388f, rotated.omega_rad_s);
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_position_controller_converts_pose_error_to_body_velocity);
    UNITY_END();
}

extern "C" void app_main(void)
{
    setup();
}
