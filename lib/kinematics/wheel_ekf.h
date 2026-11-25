#ifndef _WHEEL_EKF_H_
#define _WHEEL_EKF_H_

#include <esp_attr.h>

#include <cmath>
#include <cstdint>

// Extended Kalman filter over [angle, velocity, acceleration]. The angle is
// sensed modulo one shaft turn; angle_period is that turn in radians (2*pi
// for a bare shaft, less when the reading is scaled down by a gear ratio).
class WheelKalmanFilter
{
   public:
    explicit WheelKalmanFilter(double angle_period = 2.0 * M_PI);

    // Fuses one encoder angle (rad). IRAM hot path: the acquisition task calls
    // this for every published sample.
    void IRAM_ATTR update(double measured_angle);

    // Fuses one encoder angle plus one direct velocity measurement (rad/s) in
    // a single predict/update cycle.
    void IRAM_ATTR update(double measured_angle, double measured_velocity);

    // Drops the current estimate; the next update re-initializes the filter.
    void reset();

    double get_angle_rad() const;
    double get_velocity_rad_s() const;

   private:
    struct KalmanState
    {
        KalmanState();

        double state[3] = {};
        double covariance[3][3] = {};
        int64_t last_update_us = 0;
        bool initialized = false;
    };

    // One predict/update cycle: advances the state to the current time and
    // reports whether the filter had a previous estimate to correct. All the
    // helpers below are in IRAM as part of the acquisition hot path.
    bool IRAM_ATTR prepare_step();
    void IRAM_ATTR predict(double dt);
    void IRAM_ATTR scalar_update(int row, double innovation, double variance);

    // Maps an angle difference into [-angle_period/2, angle_period/2).
    double IRAM_ATTR wrap_angle(double angle) const;

    KalmanState m_state;
    double m_angle_period;
};

#endif  // _WHEEL_EKF_H_
