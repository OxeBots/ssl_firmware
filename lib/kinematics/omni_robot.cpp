#include "omni_robot.h"

using namespace config::kinematic;

/**
 * @brief Constructor for OmnidirectionalRobot
 *
 * @param r Wheel radius in meters (must be > 0)
 * @param d Distance from center to wheels in meters (must be > 0)
 */
OmnidirectionalRobot::OmnidirectionalRobot(double const r = OMNI_WHEEL_RADIUS,
                                           double const d = OMNI_WHEEL_DISTANCE)
: m_wheel_radius(abs(r)), m_wheel_distance(abs(d))
{
    // Construct H matrix
    for (size_t i = 0; i < 4; ++i)
    {
        m_H(i, 0) = -m_wheel_distance;
        m_H(i, 1) = std::cos(WHEELS_ANGLE_OFFSET[i]);
        m_H(i, 2) = std::sin(WHEELS_ANGLE_OFFSET[i]);
    }

    // Compute pseudoinverse: H⁺ = (HᵀH)⁻¹Hᵀ
    m_H_pinv = (m_H.transpose() * m_H).inverse() * m_H.transpose();
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
    return (m_H * body_velocities) / m_wheel_radius;
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
    return m_H_pinv * wheel_velocities * m_wheel_radius;
}
