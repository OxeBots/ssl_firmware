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
    const double m_wheel_radius;
    const double m_wheel_distance;

    vt::numeric_matrix<4, 3> m_H;
    vt::numeric_matrix<3, 4> m_H_pinv;

   public:
    OmnidirectionalRobot(double r, double d);

    vt::numeric_vector<4> compute_wheel_velocities(
      const vt::numeric_vector<3> & body_velocities) const;

    vt::numeric_vector<3> compute_body_velocities(
      const vt::numeric_vector<4> & wheel_velocities) const;

    OmnidirectionalRobot(const OmnidirectionalRobot &) = delete;
    OmnidirectionalRobot & operator=(const OmnidirectionalRobot &) = delete;
};

#endif  // KINEMATICS_OMNI_ROBOT_H
