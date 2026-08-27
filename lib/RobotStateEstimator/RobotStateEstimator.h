#ifndef ROBOT_STATE_ESTIMATOR_H
#define ROBOT_STATE_ESTIMATOR_H

#include "RuntimeTypes.h"
#include "omni_robot.h"

class RobotStateEstimator
{
   public:
    explicit RobotStateEstimator(const OmnidirectionalRobot & kinematics);

    void reset();
    RobotStateEstimate update(const FusionInputFrame & input);
    const RobotStateEstimate & get_latest() const;

   private:
    const OmnidirectionalRobot & m_kinematics;
    RobotStateEstimate m_latest;
    uint64_t m_last_timestamp_us = 0;
    bool m_has_timestamp = false;
};

#endif  // ROBOT_STATE_ESTIMATOR_H
