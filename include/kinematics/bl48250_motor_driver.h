#pragma once

#include <ArduinoEigenDense.h>
#include <driver/gpio.h>
#include <driver/ledc.h>

#include <algorithm>
#include <array>
#include <cmath>

#define BL48250_MAX_VEL_RPM 3820.0
#define BL48250_MAX_VEL_RAD (BL48250_MAX_VEL_RPM * 2.0 * M_PI / 60.0)  // rad/s
#define BL48250_MIN_VEL 0.0
#define MOTOR_FORWARD 1
#define MOTOR_BACKWARD 0

/**
 * @class BL48250Driver
 * @brief Send the needed signals to the 4 BL48250 motors of the
 * omnidirectional robot to move it.
 */
class BL48250Driver
{
   private:
    ledc_timer_t timer_;
    ledc_mode_t speed_mode_;
    ledc_timer_bit_t duty_resolution_;
    uint32_t pwm_freq_;
    std::array<gpio_num_t, 4> motor_pwm_pins_;
    std::array<gpio_num_t, 4> motor_dir_pins_;
    std::array<ledc_channel_t, 4> motor_channels_;
    uint32_t max_duty_;

   public:
    /**
     * @brief Construct a new BL48250Driver object
     * @param timer LEDC timer to use for all PWM channels
     * @param speed_mode LEDC speed mode (LEDC_LOW_SPEED_MODE or
     * LEDC_HIGH_SPEED_MODE)
     * @param duty_resolution Bit resolution of PWM duty cycle
     * @param pwm_freq PWM frequency in Hz
     * @param motor_pwm_pins Array of 4 GPIO pins for PWM signals (one per
     * motor)
     * @param motor_dir_pins Array of 4 GPIO pins for direction control (one
     * per motor)
     * @param motor_channels Array of 4 LEDC channels (one per motor)
     */
    BL48250Driver(ledc_timer_t timer, ledc_mode_t speed_mode,
                  ledc_timer_bit_t duty_resolution, uint32_t pwm_freq,
                  const std::array<gpio_num_t, 4> & motor_pwm_pins,
                  const std::array<gpio_num_t, 4> & motor_dir_pins,
                  const std::array<ledc_channel_t, 4> & motor_channels);

    /**
     * @brief Destroy the BL48250Driver object
     * @details Stops all motors by setting their duty cycle to 0
     */
    ~BL48250Driver();

    /**
     * @brief Set velocities for all 4 motors simultaneously
     * @param velocities Eigen::Vector4d containing velocities in rad/s
     * @warning Input velocities should be in range [-BL48250_MAX_VEL_RAD,
     * BL48250_MAX_VEL_RAD] converted to rad/s
     */
    void setVelocities(const Eigen::Vector4d & velocities);

    BL48250Driver(const BL48250Driver &) = delete;
    BL48250Driver & operator=(const BL48250Driver &) = delete;
};
