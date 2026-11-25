#include <sdkconfig.h>
#include <unity.h>

#include "SafetySupervisor.h"

#if CONFIG_DISABLE_SAFE_STOP
static constexpr LifecycleState STOPPED_STATE = LifecycleState::DISARMED;
#else
static constexpr LifecycleState STOPPED_STATE = LifecycleState::SAFE_STOP;
#endif

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
    TEST_ASSERT_EQUAL_INT(static_cast<int>(STOPPED_STATE), static_cast<int>(decision.lifecycle));
    TEST_ASSERT_TRUE(decision.goal.motion_inhibited);
    TEST_ASSERT_TRUE(decision.goal.kick_inhibited);

    envelope.received_at_us = 300001;
    decision = supervisor.update(300002, &envelope, healthy_state(), {});
    TEST_ASSERT_EQUAL_INT(static_cast<int>(LifecycleState::ACTIVE),
                          static_cast<int>(decision.lifecycle));
    TEST_ASSERT_TRUE(decision.command_fresh);
    TEST_ASSERT_FALSE(decision.goal.motion_inhibited);
}

void test_safety_supervisor_accepts_motion_with_health_warnings()
{
    SafetySupervisor supervisor;
    supervisor.mark_self_test_complete(true);

    CommandEnvelope envelope = {};
    envelope.command = make_command(1000, 0, 80);
    envelope.received_at_us = 1000;
    envelope.sequence = 7;

    HealthState health = {};
    health.severity = HealthSeverity::WARNING;
    health.active_faults = ERROR_IMU_FAIL | ERROR_ENCODER_INVALID;
    const SafetyDecision decision = supervisor.update(1100, &envelope, health, {});
    TEST_ASSERT_EQUAL_INT(static_cast<int>(LifecycleState::DEGRADED),
                          static_cast<int>(decision.lifecycle));
    TEST_ASSERT_FALSE(decision.goal.motion_inhibited);
    TEST_ASSERT_TRUE(decision.goal.kick_inhibited);
    TEST_ASSERT_TRUE(decision.command_fresh);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, decision.goal.velocity.vx_m_s);
}

void test_safety_supervisor_applies_configured_health_policy()
{
    SafetySupervisor supervisor;
    supervisor.mark_self_test_complete(true);

    CommandEnvelope envelope = {};
    envelope.command = make_command(1000, 0, 80);
    envelope.received_at_us = 1000;

    HealthState health = healthy_state();
    health.severity = HealthSeverity::CRITICAL;
    health.active_faults = ERROR_BATT_UNDERVOLT;
    health.latched_faults = ERROR_BATT_UNDERVOLT;
    SafetyDecision decision = supervisor.update(1100, &envelope, health, {});
#if CONFIG_DISABLE_SAFE_STOP
    TEST_ASSERT_FALSE(SafetySupervisor::health_inhibits_motion(health));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(LifecycleState::DEGRADED),
                          static_cast<int>(decision.lifecycle));
    TEST_ASSERT_FALSE(decision.goal.motion_inhibited);
    TEST_ASSERT_FALSE(decision.goal.kick_inhibited);
#else
    TEST_ASSERT_TRUE(SafetySupervisor::health_inhibits_motion(health));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(STOPPED_STATE), static_cast<int>(decision.lifecycle));
    TEST_ASSERT_TRUE(decision.goal.motion_inhibited);
    TEST_ASSERT_TRUE(decision.goal.kick_inhibited);
#endif

    health.severity = HealthSeverity::HEALTHY;
    health.active_faults = 0;
    envelope.received_at_us = 1200;
    decision = supervisor.update(1300, &envelope, health, {});
#if CONFIG_DISABLE_SAFE_STOP
    TEST_ASSERT_EQUAL_INT(static_cast<int>(LifecycleState::ACTIVE),
                          static_cast<int>(decision.lifecycle));
    TEST_ASSERT_FALSE(decision.goal.motion_inhibited);
#else
    TEST_ASSERT_EQUAL_INT(static_cast<int>(LifecycleState::SAFE_STOP),
                          static_cast<int>(decision.lifecycle));
    TEST_ASSERT_TRUE(decision.goal.motion_inhibited);
#endif
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_safety_supervisor_gates_commands_and_timeouts);
    RUN_TEST(test_safety_supervisor_accepts_motion_with_health_warnings);
    RUN_TEST(test_safety_supervisor_applies_configured_health_policy);
    UNITY_END();
}

extern "C" void app_main(void)
{
    setup();
}
