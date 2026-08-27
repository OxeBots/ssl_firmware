#include "CascadedMotionController.h"

#include <cmath>

static float clamp_value(float value, float minimum, float maximum)
{
    if (value < minimum)
        return minimum;
    if (value > maximum)
        return maximum;
    return value;
}

static float wrap_angle(float angle_rad)
{
    constexpr float TWO_PI_VALUE = 6.28318530718f;
    while (angle_rad > 3.14159265359f) angle_rad -= TWO_PI_VALUE;
    while (angle_rad < -3.14159265359f) angle_rad += TWO_PI_VALUE;
    return angle_rad;
}

PoseController::PoseController(const MotionControlConfig & config) : m_config(config)
{
}

/**
 * @brief Convert a field-frame pose error and body velocity feed-forward into a body command.
 */
BodyVelocity PoseController::compute(const RobotStateEstimate & state, const GoalPose & goal) const
{
    const float cos_heading = std::cos(state.pose.heading_rad);
    const float sin_heading = std::sin(state.pose.heading_rad);
    const float error_x = goal.pose.x_m - state.pose.x_m;
    const float error_y = goal.pose.y_m - state.pose.y_m;

    BodyVelocity desired = goal.velocity;
    desired.vx_m_s += m_config.position_gain * (cos_heading * error_x + sin_heading * error_y);
    desired.vy_m_s += m_config.position_gain * (-sin_heading * error_x + cos_heading * error_y);
    desired.omega_rad_s +=
      m_config.heading_gain * wrap_angle(goal.pose.heading_rad - state.pose.heading_rad);
    return desired;
}

VelocityController::VelocityController(const MotionControlConfig & config) : m_config(config)
{
}

/**
 * @brief Bound the body velocity target before inverse kinematics.
 */
BodyVelocity VelocityController::limit(const BodyVelocity & desired) const
{
    return {
      clamp_value(desired.vx_m_s, -m_config.max_vx_m_s, m_config.max_vx_m_s),
      clamp_value(desired.vy_m_s, -m_config.max_vy_m_s, m_config.max_vy_m_s),
      clamp_value(desired.omega_rad_s, -m_config.max_omega_rad_s, m_config.max_omega_rad_s),
    };
}

CascadedMotionController::CascadedMotionController() : m_pose_controller(), m_velocity_controller()
{
}

CascadedMotionController::CascadedMotionController(const OmnidirectionalRobot & kinematics,
                                                   const MotionControlConfig & config)
: m_pose_controller(config), m_velocity_controller(config)
{
    init(kinematics, config);
}

void CascadedMotionController::init(const OmnidirectionalRobot & kinematics,
                                    const MotionControlConfig & config)
{
    m_kinematics = &kinematics;
    m_pose_controller = PoseController(config);
    m_velocity_controller = VelocityController(config);
    m_max_wheel_velocity_rad_s = config.max_wheel_velocity_rad_s;
}

/**
 * @brief Apply safety gates, pose control, velocity limits and inverse kinematics.
 */
ActuationSetpoint CascadedMotionController::compute(const GoalPose & goal,
                                                    const RobotStateEstimate & state) const
{
    ActuationSetpoint output = {};
    output.kick_inhibited = goal.kick_inhibited || state.health.kick_inhibited;
    output.motion_inhibited = goal.motion_inhibited || state.health.motion_inhibited;
    output.command_sequence = goal.command_sequence;
    output.timestamp_us = goal.received_at_us;

    if (output.motion_inhibited || m_kinematics == nullptr)
        return output;

    const BodyVelocity body_target =
      m_velocity_controller.limit(m_pose_controller.compute(state, goal));
    vt::numeric_vector<3> body_vector;
    body_vector(0) = body_target.omega_rad_s;
    body_vector(1) = body_target.vx_m_s;
    body_vector(2) = body_target.vy_m_s;
    const vt::numeric_vector<4> wheel_target = m_kinematics->compute_wheel_velocities(body_vector);

    for (size_t index = 0; index < ROBOT_WHEEL_COUNT; ++index)
    {
        output.wheel_velocity_rad_s[index] = clamp_value(static_cast<float>(wheel_target(index)),
                                                         -m_max_wheel_velocity_rad_s,
                                                         m_max_wheel_velocity_rad_s);
    }

    if (!output.kick_inhibited)
        output.kick_velocity = goal.kick_velocity;
    return output;
}
