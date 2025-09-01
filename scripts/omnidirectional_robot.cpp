#include "kinematics/omnidirectional_robot.h"

using namespace config::kinematic;

OmnidirectionalRobot::OmnidirectionalRobot(
  double r = OMNI_WHEEL_RADIUS,
  double d = OMNI_WHEEL_DISTANCE)
: wheel_radius(r), wheel_distance(d)
{
    if (r <= 0 or d <= 0)
        throw std::invalid_argument("Both parameters must be positive");

    // Construct H matrix
    for (size_t i = 0; i < 4; ++i)
    {
        H(i, 0) = -wheel_distance;
        H(i, 1) = std::cos(WHEELS_ANGLE_OFFSET[i]);
        H(i, 2) = std::sin(WHEELS_ANGLE_OFFSET[i]);
    }

    // Compute pseudoinverse: H⁺ = (HᵀH)⁻¹Hᵀ
    H_pinv = (H.transpose() * H).inverse() * H.transpose();
}

Eigen::Vector4d OmnidirectionalRobot::computeWheelVelocities(double omega_z,
                                                             double vx,
                                                             double vy) const
{
    return (H * Eigen::Vector3d(omega_z, vx, vy)) / wheel_radius;
}

Eigen::Vector3d OmnidirectionalRobot::computeBodyVelocities(
  const Eigen::Vector4d & wheel_velocities) const
{
    return H_pinv * wheel_velocities * wheel_radius;
}
