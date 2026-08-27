#ifndef CASCADED_MOTION_CONTROLLER_H
#define CASCADED_MOTION_CONTROLLER_H

#include "RuntimeTypes.h"
#include "omni_robot.h"

struct MotionControlConfig
{
    float position_gain = 2.0f;
    float heading_gain = 3.0f;
    float max_vx_m_s = 2.0f;
    float max_vy_m_s = 2.0f;
    float max_omega_rad_s = 8.0f;
    float max_wheel_velocity_rad_s = 80.0f;
};

class PoseController
{
   public:
    explicit PoseController(const MotionControlConfig & config = MotionControlConfig());

    BodyVelocity compute(const RobotStateEstimate & state, const GoalPose & goal) const;

   private:
    MotionControlConfig m_config;
};

class VelocityController
{
   public:
    explicit VelocityController(const MotionControlConfig & config = MotionControlConfig());

    BodyVelocity limit(const BodyVelocity & desired) const;

   private:
    MotionControlConfig m_config;
};

class CascadedMotionController
{
   public:
    CascadedMotionController();
    explicit CascadedMotionController(const OmnidirectionalRobot & kinematics,
                                      const MotionControlConfig & config = MotionControlConfig());

    void init(const OmnidirectionalRobot & kinematics,
              const MotionControlConfig & config = MotionControlConfig());
    ActuationSetpoint compute(const GoalPose & goal, const RobotStateEstimate & state) const;

   private:
    const OmnidirectionalRobot * m_kinematics = nullptr;
    PoseController m_pose_controller;
    VelocityController m_velocity_controller;
    float m_max_wheel_velocity_rad_s = MotionControlConfig().max_wheel_velocity_rad_s;
};

#endif  // CASCADED_MOTION_CONTROLLER_H
