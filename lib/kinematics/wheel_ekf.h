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

    void update(double measured_angle);

    double get_angle_rad() const;
    double get_velocity_rad_s() const;
    double get_acceleration_rad_s2() const;

   private:
    struct KalmanState;
    std::unique_ptr<KalmanState> m_state;
};

#endif  // KINEMATICS_WHEEL_EKF_H
