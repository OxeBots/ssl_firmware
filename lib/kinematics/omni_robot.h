/**
 * @file omni_robot.h
 * @brief Defines the kinematics for an omnidirectional robot with four omnidirectional wheels.
 * This class is pure math and has no hardware dependencies.
 */

#ifndef KINEMATICS_OMNI_ROBOT_H
#define KINEMATICS_OMNI_ROBOT_H

#include <numeric_matrix.h>
#include <numeric_vector.h>

#include <array>
#include <cmath>
#include <stdexcept>

namespace config
{
namespace kinematic
{
constexpr double OMNI_WHEEL_RADIUS = 0.0325;   // Wheel radius in meters (32.5 mm)
constexpr double OMNI_WHEEL_DISTANCE = 0.096;  // Distance from center to wheels in meters (96 mm)

constexpr std::array<double, 4> WHEELS_ANGLE_OFFSET = {
  M_PI / 4,      // 45° - 1st wheel angle with respect to robot frame
  3 * M_PI / 4,  // 135° - 2nd wheel angle
  5 * M_PI / 4,  // 225° - 3rd wheel angle
  7 * M_PI / 4   // 315° - 4th wheel angle
};

}  // namespace kinematic
}  // namespace config

/**
 * @brief See https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html
 * for mathematical details
 *
 */
class OmnidirectionalRobot
{
   private:
    const double wheel_radius;    // Wheel radius (m)
    const double wheel_distance;  // Distance from center to wheels (m)

    vt::numeric_matrix<4, 3> H;       // Inverse Kinematics matrix
    vt::numeric_matrix<3, 4> H_pinv;  // Pseudoinverse of H for Forward Kinematics

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
     * @param body_velocities [ω_z (rad/s), v_x (m/s), v_y (m/s)]
     * @return vt::numeric_vector<4> Wheel angular velocities [rad/s]
     */
    vt::numeric_vector<4> computeWheelVelocities(const vt::numeric_vector<3> & body_velocities) const;

    /**
     * @brief Compute body velocities from wheel angular velocities (forward
     * kinematics)
     *
     * @param wheel_velocities Wheel angular velocities [rad/s]
     * @return vt::numeric_vector<3> [ω_z (rad/s), v_x (m/s), v_y (m/s)]
     */
    vt::numeric_vector<3> computeBodyVelocities(const vt::numeric_vector<4> & wheel_velocities) const;

    OmnidirectionalRobot(const OmnidirectionalRobot &) = delete;
    OmnidirectionalRobot & operator=(const OmnidirectionalRobot &) = delete;
};

#endif  // KINEMATICS_OMNI_ROBOT_H
