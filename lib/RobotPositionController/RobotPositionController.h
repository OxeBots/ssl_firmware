#ifndef ROBOT_POSITION_CONTROLLER_H
#define ROBOT_POSITION_CONTROLLER_H

#include "RuntimeTypes.h"

struct PositionControlConfig
{
    float position_gain = 2.0f;
    float heading_gain = 3.0f;
};

// Top motion layer: turns a goal pose plus the estimated robot pose into a
// body-frame velocity setpoint for RobotVelocityController.
class RobotPositionController
{
   public:
    explicit RobotPositionController(
      const PositionControlConfig & config = PositionControlConfig());

    BodyVelocity compute(const RobotStateEstimate & state, const GoalPose & goal) const;

   private:
    PositionControlConfig m_config;
};

#endif  // ROBOT_POSITION_CONTROLLER_H
