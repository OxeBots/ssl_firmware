#include "omni_robot.h"

using namespace config::kinematic;

/**
 * @brief Constructor for OmnidirectionalRobot
 *
 * @param r Wheel radius in meters (must be > 0)
 * @param d Distance from center to wheels in meters (must be > 0)
 */
OmnidirectionalRobot::OmnidirectionalRobot(double const r, double const d)
: m_wheel_radius(abs(r)), m_wheel_distance(abs(d))
{
    // Construct H matrix. Body frame: x forward, y left. Each wheel is mounted
    // at its angle offset and rolls along the tangent, so its speed is the body
    // velocity projected on (-sin angle, cos angle).
    for (size_t i = 0; i < 4; ++i)
    {
        m_h(i, 0) = -m_wheel_distance;
        m_h(i, 1) = -std::sin(WHEELS_ANGLE_OFFSET[i]);
        m_h(i, 2) = std::cos(WHEELS_ANGLE_OFFSET[i]);
    }

    // Compute pseudoinverse: H⁺ = (HᵀH)⁻¹Hᵀ
    m_h_pinv = (m_h.transpose() * m_h).inverse() * m_h.transpose();
}

/**
 * @brief Compute wheel angular velocities from body velocities (inverse kinematics)
 *
 * @param body_velocities [ω_z (rad/s), v_x (m/s), v_y (m/s)]
 * @return vt::numeric_vector<4> Wheel angular velocities [rad/s]
 */
vt::numeric_vector<4> OmnidirectionalRobot::compute_wheel_velocities(
  const vt::numeric_vector<3> & body_velocities) const
{
    return (m_h * body_velocities) / m_wheel_radius;
}

/**
 * @brief Compute body velocities from wheel angular velocities (forward kinematics)
 *
 * @param wheel_velocities Wheel angular velocities [rad/s]
 * @return vt::numeric_vector<3> [ω_z (rad/s), v_x (m/s), v_y (m/s)]
 */
vt::numeric_vector<3> OmnidirectionalRobot::compute_body_velocities(
  const vt::numeric_vector<4> & wheel_velocities) const
{
    return m_h_pinv * wheel_velocities * m_wheel_radius;
}
