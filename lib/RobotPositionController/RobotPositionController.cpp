#include "RobotPositionController.h"

#include <cmath>

static float wrap_angle(float angle_rad)
{
    constexpr float TWO_PI_VALUE = 6.28318530718f;
    while (angle_rad > 3.14159265359f) angle_rad -= TWO_PI_VALUE;
    while (angle_rad < -3.14159265359f) angle_rad += TWO_PI_VALUE;
    return angle_rad;
}

/**
 * @brief Construct the position controller with its proportional gains.
 */
RobotPositionController::RobotPositionController(const PositionControlConfig & config)
: m_config(config)
{
}

/**
 * @brief Convert a field-frame pose error and body velocity feed-forward into a
 * body-frame velocity command.
 */
BodyVelocity RobotPositionController::compute(const RobotStateEstimate & state,
                                              const GoalPose & goal) const
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
