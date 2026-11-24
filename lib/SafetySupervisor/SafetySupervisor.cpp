#include "SafetySupervisor.h"

static constexpr float PI = 3.14159265359f;

SafetySupervisor::SafetySupervisor(const SafetySupervisorConfig & config) : m_config(config)
{
}

void SafetySupervisor::mark_self_test_complete(bool complete)
{
    m_self_test_complete = complete;
    if (complete && m_state == LifecycleState::BOOT)
        m_state = LifecycleState::DISARMED;
    else if (!complete)
        m_state = LifecycleState::SELF_TEST;
}

void SafetySupervisor::set_calibration_active(bool active)
{
    m_calibration_active = active;
}

void SafetySupervisor::enter_safe_stop()
{
    m_forced_safe_stop = true;
    m_state = LifecycleState::SAFE_STOP;
}

void SafetySupervisor::clear_safe_stop()
{
    if (m_state == LifecycleState::SAFE_STOP && m_self_test_complete)
        m_state = LifecycleState::DISARMED;
}

/**
 * @brief Apply command freshness, lifecycle, health and local-obstacle gates.
 */
SafetyDecision SafetySupervisor::update(uint64_t now_us,
                                        const CommandEnvelope * command,
                                        const HealthState & health,
                                        const LocalPerceptionState & perception)
{
    SafetyDecision decision = {};
    decision.goal = make_safe_goal();

    if (m_forced_safe_stop)
    {
        m_state = LifecycleState::SAFE_STOP;
        decision.lifecycle = m_state;
        return decision;
    }

    if (!m_self_test_complete)
    {
        m_state = LifecycleState::SELF_TEST;
        decision.lifecycle = m_state;
        return decision;
    }

    if (m_calibration_active)
    {
        m_state = LifecycleState::CALIBRATION;
        decision.lifecycle = m_state;
        return decision;
    }

    if (health.severity == HealthSeverity::CRITICAL || health.motion_inhibited)
        m_state = LifecycleState::SAFE_STOP;

    decision.command_fresh = is_command_fresh(now_us, command);

    if (m_state == LifecycleState::SAFE_STOP)
    {
        decision.lifecycle = m_state;
        return decision;
    }

    if (!decision.command_fresh)
    {
        m_state = LifecycleState::DISARMED;
        decision.lifecycle = m_state;
        return decision;
    }

    decision.goal = make_goal(*command, health);
    if (perception.valid && perception.obstacle_close &&
        (perception.nearest_distance_mm < 0 ||
         perception.nearest_distance_mm <= m_config.obstacle_stop_distance_mm))
    {
        decision.goal.motion_inhibited = true;
    }

    m_state = health.severity == HealthSeverity::RECOVERABLE || decision.goal.motion_inhibited
                ? LifecycleState::DEGRADED
                : LifecycleState::ACTIVE;
    decision.lifecycle = m_state;
    return decision;
}

LifecycleState SafetySupervisor::get_state() const
{
    return m_state;
}

GoalPose SafetySupervisor::make_safe_goal()
{
    GoalPose goal = {};
    goal.motion_inhibited = true;
    goal.kick_inhibited = true;
    return goal;
}

bool SafetySupervisor::is_command_fresh(uint64_t now_us, const CommandEnvelope * command) const
{
    return command != nullptr && now_us >= command->received_at_us &&
           now_us - command->received_at_us <= m_config.command_timeout_us;
}

GoalPose SafetySupervisor::make_goal(const CommandEnvelope & envelope,
                                     const HealthState & health) const
{
    const RobotCommand & command = envelope.command;
    GoalPose goal = {};
    goal.pose.x_m = static_cast<float>(command.target_pose.x) / 1000.0f;
    goal.pose.y_m = static_cast<float>(command.target_pose.y) / 1000.0f;
    goal.pose.heading_rad = static_cast<float>(command.target_pose.angle) * 0.01f * PI / 180.0f;
    goal.velocity.vx_m_s = static_cast<float>(command.target_pose.x_v) / 1000.0f;
    goal.velocity.vy_m_s = static_cast<float>(command.target_pose.y_v) / 1000.0f;
    goal.velocity.omega_rad_s = static_cast<float>(command.target_pose.angular_vel) / 100.0f;
    goal.kick_velocity = command.kick_velocity;
    goal.kick_inhibited = health.kick_inhibited;
    goal.command_sequence = envelope.sequence;
    goal.received_at_us = envelope.received_at_us;
    return goal;
}
