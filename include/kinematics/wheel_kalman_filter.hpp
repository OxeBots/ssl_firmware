/**
 * @file wheel_kalman_filter.hpp
 * @brief A dedicated class for the wheel's state estimation using an EKF.
 * This class is pure math and has no hardware dependencies.
 */
#ifndef DRIVER_WHEEL_KALMAN_FILTER_HPP
#define DRIVER_WHEEL_KALMAN_FILTER_HPP

#include <cstdint>
#include <memory>

#include "helper_func.h"

class WheelKalmanFilter
{
   public:
    WheelKalmanFilter();
    ~WheelKalmanFilter();

    /**
     * @brief Updates the filter with a new angle measurement.
     * @param measured_angle The new angle in radians.
     */
    void update(float measured_angle);

    // --- State Getters ---
    float get_angle_rad() const;
    float get_velocity_rad_s() const;
    float get_acceleration_rad_s2() const;

   private:
    struct KalmanState;
    std::unique_ptr<KalmanState> m_state;
};

#endif  // DRIVER_WHEEL_KALMAN_FILTER_HPP
