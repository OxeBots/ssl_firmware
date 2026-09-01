#ifndef ROBOT_VELOCITY_CONTROLLER_H
#define ROBOT_VELOCITY_CONTROLLER_H

#include "RuntimeTypes.h"
#include "omni_robot.h"

struct VelocityControlConfig
{
    float max_vx_m_s = 2.0f;
    float max_vy_m_s = 2.0f;
    float max_omega_rad_s = 8.0f;
    float max_wheel_velocity_rad_s = 80.0f;
};

// Middle motion layer: bounds a body-velocity setpoint, converts it to wheel
// speed targets through inverse kinematics and applies the safety gates.
// WheelController runs below it, tracking each wheel speed with PID.
class RobotVelocityController
{
   public:
    RobotVelocityController();
    explicit RobotVelocityController(
      const OmnidirectionalRobot & kinematics,
      const VelocityControlConfig & config = VelocityControlConfig());

    void init(const OmnidirectionalRobot & kinematics,
              const VelocityControlConfig & config = VelocityControlConfig());

    ActuationSetpoint compute(const BodyVelocity & desired_velocity,
                              const GoalPose & goal,
                              const HealthState & health) const;

   private:
    const OmnidirectionalRobot * m_kinematics = nullptr;
    VelocityControlConfig m_config;
};

#endif  // ROBOT_VELOCITY_CONTROLLER_H
