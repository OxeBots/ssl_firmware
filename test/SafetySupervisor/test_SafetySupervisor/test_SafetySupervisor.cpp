#include <unity.h>

#include "SafetySupervisor.h"

static RobotCommand make_command(int16_t x_mm, int16_t y_mm, uint8_t kick_velocity = 0)
{
    RobotCommand command = {};
    command.target_pose.x = x_mm;
    command.target_pose.y = y_mm;
    command.target_pose.angle = 9000;
    command.target_pose.x_v = 1000;
    command.target_pose.y_v = 0;
    command.target_pose.angular_vel = 0;
    command.kick_velocity = kick_velocity;
    return command;
}

static HealthState healthy_state()
{
    HealthState health = {};
    health.severity = HealthSeverity::HEALTHY;
    health.kick_inhibited = false;
    return health;
}

void test_safety_supervisor_gates_commands_and_timeouts()
{
    SafetySupervisor supervisor;
    supervisor.mark_self_test_complete(true);

    CommandEnvelope envelope = {};
    envelope.command = make_command(1000, 0, 80);
    envelope.received_at_us = 1000;
    envelope.sequence = 7;

    SafetyDecision decision = supervisor.update(1100, &envelope, healthy_state(), {});
    TEST_ASSERT_EQUAL_INT(static_cast<int>(LifecycleState::ACTIVE),
                          static_cast<int>(decision.lifecycle));
    TEST_ASSERT_TRUE(decision.command_fresh);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, decision.goal.pose.x_m);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1000.0f / 1000.0f, decision.goal.velocity.vx_m_s);
    TEST_ASSERT_FALSE(decision.goal.kick_inhibited);
    TEST_ASSERT_EQUAL_UINT8(80, decision.goal.kick_velocity);

    decision = supervisor.update(300000, &envelope, healthy_state(), {});
    TEST_ASSERT_EQUAL_INT(static_cast<int>(LifecycleState::SAFE_STOP),
                          static_cast<int>(decision.lifecycle));
    TEST_ASSERT_TRUE(decision.goal.motion_inhibited);
    TEST_ASSERT_TRUE(decision.goal.kick_inhibited);

    supervisor.clear_safe_stop();
    decision = supervisor.update(300000, nullptr, healthy_state(), {});
    TEST_ASSERT_EQUAL_INT(static_cast<int>(LifecycleState::DISARMED),
                          static_cast<int>(decision.lifecycle));
}

void test_safety_supervisor_forced_safe_stop_survives_self_test()
{
    SafetySupervisor supervisor;
    supervisor.enter_safe_stop();

    // Boot failure latch: a later self-test completion must not release it.
    supervisor.mark_self_test_complete(true);

    CommandEnvelope envelope = {};
    envelope.command = make_command(1000, 0, 80);
    envelope.received_at_us = 1000;
    envelope.sequence = 7;

    SafetyDecision decision = supervisor.update(1100, &envelope, healthy_state(), {});
    TEST_ASSERT_EQUAL_INT(static_cast<int>(LifecycleState::SAFE_STOP),
                          static_cast<int>(decision.lifecycle));
    TEST_ASSERT_TRUE(decision.goal.motion_inhibited);
    TEST_ASSERT_TRUE(decision.goal.kick_inhibited);
    TEST_ASSERT_FALSE(decision.command_fresh);
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_safety_supervisor_gates_commands_and_timeouts);
    RUN_TEST(test_safety_supervisor_forced_safe_stop_survives_self_test);
    UNITY_END();
}

extern "C" void app_main(void)
{
    setup();
}
