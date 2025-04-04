#include "kinematics/omnidirectional_robot.h"

void OmnidirectionalRobot::initializeKinematicMatrices()
{
    constexpr std::array<double, 4> angles = {
      M_PI / 4,      // 45° - 1st wheel angle with respect to robot frame
      3 * M_PI / 4,  // 135° - 2nd wheel angle
      5 * M_PI / 4,  // 225° - 3rd wheel angle
      7 * M_PI / 4   // 315° - 4th wheel angle
    };

    // Construct H matrix
    for (size_t i = 0; i < 4; ++i)
    {
        H(i, 0) = -wheel_distance;
        H(i, 1) = std::cos(angles[i]);
        H(i, 2) = std::sin(angles[i]);
    }

    // Compute pseudoinverse: H⁺ = (HᵀH)⁻¹Hᵀ
    H_pinv = (H.transpose() * H).inverse() * H.transpose();
}

OmnidirectionalRobot::OmnidirectionalRobot(double r, double d)
: wheel_radius(r), wheel_distance(d)
{
    if (r <= 0 or d <= 0)
        throw std::invalid_argument("Both parameters must be positive");
    initializeKinematicMatrices();
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
