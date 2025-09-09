#include "kinematics/omnidirectional_robot.hpp"

using namespace config::kinematic;

OmnidirectionalRobot::OmnidirectionalRobot(double const r = OMNI_WHEEL_RADIUS, double const d = OMNI_WHEEL_DISTANCE)
: wheel_radius(abs(r)), wheel_distance(abs(d))
{
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

vt::numeric_vector<4> OmnidirectionalRobot::computeWheelVelocities(const vt::numeric_vector<3> & body_velocities) const
{
    return (H * body_velocities) / wheel_radius;
}

vt::numeric_vector<3> OmnidirectionalRobot::computeBodyVelocities(const vt::numeric_vector<4> & wheel_velocities) const
{
    return H_pinv * wheel_velocities * wheel_radius;
}
