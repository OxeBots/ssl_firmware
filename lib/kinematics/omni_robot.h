#ifndef _OMNI_ROBOT_H_
#define _OMNI_ROBOT_H_

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

// Counterclockwise from forward, in motor channel order.
constexpr std::array<double, 4> WHEELS_ANGLE_OFFSET = {
  M_PI / 4,      // M1, front-left
  3 * M_PI / 4,  // M2, rear-left
  5 * M_PI / 4,  // M3, rear-right
  7 * M_PI / 4   // M4, front-right
};

}  // namespace kinematic
}  // namespace config

// See https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html
// for mathematical details. Body velocities use x forward and y left.
class OmnidirectionalRobot
{
   private:
    const double m_wheel_radius;
    const double m_wheel_distance;

    vt::numeric_matrix<4, 3> m_h;
    vt::numeric_matrix<3, 4> m_h_pinv;

   public:
    OmnidirectionalRobot(double r = config::kinematic::OMNI_WHEEL_RADIUS,
                         double d = config::kinematic::OMNI_WHEEL_DISTANCE);

    vt::numeric_vector<4> compute_wheel_velocities(
      const vt::numeric_vector<3> & body_velocities) const;

    vt::numeric_vector<3> compute_body_velocities(
      const vt::numeric_vector<4> & wheel_velocities) const;

    OmnidirectionalRobot(const OmnidirectionalRobot &) = delete;
    OmnidirectionalRobot & operator=(const OmnidirectionalRobot &) = delete;
};

#endif  // _OMNI_ROBOT_H_
