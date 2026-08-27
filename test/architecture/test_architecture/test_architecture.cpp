#include <unity.h>

#include <cmath>

#include "CascadedMotionController.h"
#include "CommandInbox.h"
#include "HealthMonitor.h"
#include "PowerMonitor.h"
#include "RobotStateEstimator.h"
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

void test_command_inbox_preserves_fifo_and_timestamps()
{
    CommandInbox inbox;
    TEST_ASSERT_TRUE(inbox.init());

    RobotCommand first = make_command(100, 0);
    RobotCommand second = make_command(200, 0);
    TEST_ASSERT_TRUE(inbox.push(first, 1000));
    TEST_ASSERT_TRUE(inbox.push(second, 2000));
    TEST_ASSERT_EQUAL_UINT8(2, inbox.size());

    CommandEnvelope envelope = {};
    TEST_ASSERT_TRUE(inbox.pop(&envelope));
    TEST_ASSERT_EQUAL_INT16(100, envelope.command.target_pose.x);
    TEST_ASSERT_EQUAL_UINT32(1, envelope.sequence);
    TEST_ASSERT_TRUE(envelope.received_at_us == 1000);
    TEST_ASSERT_TRUE(inbox.pop(&envelope));
    TEST_ASSERT_EQUAL_INT16(200, envelope.command.target_pose.x);
    TEST_ASSERT_EQUAL_UINT32(2, envelope.sequence);
    TEST_ASSERT_FALSE(inbox.pop(&envelope));
}

void test_health_monitor_separates_recoverable_and_critical_faults()
{
    HealthMonitor monitor;
    HealthObservationFrame observation = {};
    observation.wheel_valid_mask = 0x0F;
    observation.imu_available = true;
    observation.tof_available = true;
    observation.kicker_ready = true;
    observation.timestamp_us = 10;

    HealthState health = monitor.update(observation);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(HealthSeverity::HEALTHY),
                          static_cast<int>(health.severity));
    TEST_ASSERT_EQUAL_UINT16(0, health.active_faults);
    TEST_ASSERT_FALSE(health.kick_inhibited);

    observation.tof_available = false;
    health = monitor.update(observation);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(HealthSeverity::RECOVERABLE),
                          static_cast<int>(health.severity));
    TEST_ASSERT_TRUE((health.active_faults & ERROR_TOF_FAIL) != 0);
    TEST_ASSERT_FALSE(health.motion_inhibited);
    TEST_ASSERT_TRUE(health.kick_inhibited);

    observation.wheel_valid_mask = 0;
    health = monitor.update(observation);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(HealthSeverity::CRITICAL),
                          static_cast<int>(health.severity));
    TEST_ASSERT_TRUE(health.motion_inhibited);
    TEST_ASSERT_TRUE((health.latched_faults & ERROR_ENCODER_INVALID) != 0);
}

void test_power_monitor_converts_and_reports_limits()
{
    PowerMonitorConfig config = {};
    config.adc_to_supply_ratio = 10.0f;
    config.undervoltage_v = 11.0f;
    config.overvoltage_v = 13.0f;
    PowerMonitor monitor(config);
    HealthObservationFrame observation = {};

    TEST_ASSERT_TRUE(monitor.update_adc_voltage(1.2f, 42));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.0f, monitor.get_supply_voltage_v());
    TEST_ASSERT_FALSE(monitor.is_undervoltage());
    TEST_ASSERT_FALSE(monitor.is_overvoltage());
    monitor.fill_observation(&observation);
    TEST_ASSERT_TRUE(observation.supply_valid);
    TEST_ASSERT_TRUE(observation.timestamp_us == 42);

    TEST_ASSERT_TRUE(monitor.update_supply_voltage(10.0f, 43));
    TEST_ASSERT_TRUE(monitor.is_undervoltage());
    monitor.fill_observation(&observation);
    TEST_ASSERT_TRUE(observation.supply_undervoltage);
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

void test_cascaded_controller_limits_wheels_and_kick()
{
    OmnidirectionalRobot robot(config::kinematic::OMNI_WHEEL_RADIUS,
                               config::kinematic::OMNI_WHEEL_DISTANCE);
    MotionControlConfig config = {};
    config.max_wheel_velocity_rad_s = 10.0f;
    CascadedMotionController controller(robot, config);
    GoalPose goal = {};
    goal.pose.x_m = 1.0f;
    goal.kick_velocity = 25;
    goal.kick_inhibited = false;
    goal.command_sequence = 4;
    goal.received_at_us = 50;
    RobotStateEstimate state = {};
    state.health = healthy_state();

    const ActuationSetpoint output = controller.compute(goal, state);
    TEST_ASSERT_FALSE(output.motion_inhibited);
    TEST_ASSERT_FALSE(output.kick_inhibited);
    TEST_ASSERT_EQUAL_UINT8(25, output.kick_velocity);
    TEST_ASSERT_EQUAL_UINT32(4, output.command_sequence);
    for (float velocity : output.wheel_velocity_rad_s)
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, std::fabs(velocity));
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
    RUN_TEST(test_command_inbox_preserves_fifo_and_timestamps);
    RUN_TEST(test_health_monitor_separates_recoverable_and_critical_faults);
    RUN_TEST(test_power_monitor_converts_and_reports_limits);
    RUN_TEST(test_safety_supervisor_gates_commands_and_timeouts);
    RUN_TEST(test_cascaded_controller_limits_wheels_and_kick);
    RUN_TEST(test_robot_state_estimator_integrates_odometry_and_accepts_vision_correction);
    UNITY_END();
}

extern "C" void app_main(void)
{
    setup();
}
