#ifndef KINEMATICS_WHEEL_EKF_H
#define KINEMATICS_WHEEL_EKF_H

#include <esp_timer.h>

#include <cstdint>
#include <memory>

#include "helper_func.h"

class WheelKalmanFilter
{
   public:
    WheelKalmanFilter();
    ~WheelKalmanFilter();

    void update(float measured_angle);

    float get_angle_rad() const;
    float get_velocity_rad_s() const;
    float get_acceleration_rad_s2() const;

   private:
    struct KalmanState;
    std::unique_ptr<KalmanState> m_state;
};

#endif  // KINEMATICS_WHEEL_EKF_H
