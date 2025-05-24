#ifndef KINEMATICS_OMNIDIRECTIONAL_ROBOT_H
#define KINEMATICS_OMNIDIRECTIONAL_ROBOT_H

#include <ArduinoEigenDense.h>

#include <cmath>
#include <stdexcept>

namespace config
{
namespace kinematic
{
constexpr double OMNI_WHEEL_RADIUS =
  0.0325;  // Wheel radius in meters (32.5 mm)
constexpr double OMNI_WHEEL_DISTANCE =
  0.096;  // Distance from center to wheels in meters (96 mm)

constexpr std::array<double, 4> WHEELS_ANGLE_OFFSET = {
  M_PI / 4,      // 45° - 1st wheel angle with respect to robot frame
  3 * M_PI / 4,  // 135° - 2nd wheel angle
  5 * M_PI / 4,  // 225° - 3rd wheel angle
  7 * M_PI / 4   // 315° - 4th wheel angle
};

}  // namespace kinematic
}  // namespace config

/**
 * @brief See
 * https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html
 * for mathematical details
 *
 */
class OmnidirectionalRobot
{
   private:
    const double wheel_radius;    // Wheel radius (m)
    const double wheel_distance;  // Distance from center to wheels (m)

    // Kinematic matrices (precomputed during construction)
    Eigen::Matrix<double, 4, 3> H;  // Inverse Kinematics matrix
    Eigen::Matrix<double, 3, 4>
      H_pinv;  // Pseudoinverse of H for Forward Kinematics

   public:
    /**
     * @brief Constructor for OmnidirectionalRobot
     *
     * @param r Wheel radius in meters (must be > 0)
     * @param d Distance from center to wheels in meters (must be > 0)
     */
    OmnidirectionalRobot(double r, double d);

    /**
     * @brief Compute wheel angular velocities from body velocities (inverse
     * kinematics)
     *
     * @param omega_z  Angular velocity about Z-axis [rad/s]
     * @param vx       Linear velocity in X-direction [m/s]
     * @param vy       Linear velocity in Y-direction [m/s]
     * @return Eigen::Vector4d Wheel angular velocities [rad/s]
     */
    Eigen::Vector4d computeWheelVelocities(double omega_z, double vx,
                                           double vy) const;

    /**
     * @brief Compute body velocities from wheel angular velocities (forward
     * kinematics)
     *
     * @param wheel_velocities Wheel angular velocities [rad/s]
     * @return Eigen::Vector3d [ω_z (rad/s), v_x (m/s), v_y (m/s)]
     */
    Eigen::Vector3d computeBodyVelocities(
      const Eigen::Vector4d & wheel_velocities) const;

    OmnidirectionalRobot(const OmnidirectionalRobot &) = delete;
    OmnidirectionalRobot & operator=(const OmnidirectionalRobot &) = delete;
};

#endif  // KINEMATICS_OMNIDIRECTIONAL_ROBOT_H
