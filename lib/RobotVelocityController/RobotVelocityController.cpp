#include "RobotVelocityController.h"

#include <algorithm>

/**
 * @brief Construct without a kinematic model. Call init() before compute().
 */
RobotVelocityController::RobotVelocityController() : m_config()
{
}

/**
 * @brief Construct with the kinematic model and velocity limits.
 */
RobotVelocityController::RobotVelocityController(const OmnidirectionalRobot & kinematics,
                                                 const VelocityControlConfig & config)
: m_config(config)
{
    init(kinematics, config);
}

/**
 * @brief Bind the kinematic model used to convert body velocity to wheel speed.
 */
void RobotVelocityController::init(const OmnidirectionalRobot & kinematics,
                                   const VelocityControlConfig & config)
{
    m_kinematics = &kinematics;
    m_config = config;
}

/**
 * @brief Apply safety gates, body velocity limits and inverse kinematics.
 */
ActuationSetpoint RobotVelocityController::compute(const BodyVelocity & desired_velocity,
                                                   const GoalPose & goal,
                                                   const HealthState & health) const
{
    ActuationSetpoint output = {};
    output.kick_inhibited = goal.kick_inhibited || health.kick_inhibited;
    output.motion_inhibited = goal.motion_inhibited || health.motion_inhibited;
    output.command_sequence = goal.command_sequence;
    output.timestamp_us = goal.received_at_us;

    if (output.motion_inhibited || m_kinematics == nullptr)
        return output;

    const BodyVelocity limited = {
      std::clamp(desired_velocity.vx_m_s, -m_config.max_vx_m_s, m_config.max_vx_m_s),
      std::clamp(desired_velocity.vy_m_s, -m_config.max_vy_m_s, m_config.max_vy_m_s),
      std::clamp(desired_velocity.omega_rad_s, -m_config.max_omega_rad_s, m_config.max_omega_rad_s),
    };

    vt::numeric_vector<3> body_vector;
    body_vector(0) = limited.omega_rad_s;
    body_vector(1) = limited.vx_m_s;
    body_vector(2) = limited.vy_m_s;
    const vt::numeric_vector<4> wheel_target = m_kinematics->compute_wheel_velocities(body_vector);

    for (size_t index = 0; index < ROBOT_WHEEL_COUNT; ++index)
    {
        output.wheel_velocity_rad_s[index] = std::clamp(static_cast<float>(wheel_target(index)),
                                                        -m_config.max_wheel_velocity_rad_s,
                                                        m_config.max_wheel_velocity_rad_s);
    }

    if (!output.kick_inhibited)
        output.kick_velocity = goal.kick_velocity;
    return output;
}
