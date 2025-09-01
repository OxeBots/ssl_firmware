#ifndef DRIVER_BL48250_MOTOR_H
#define DRIVER_BL48250_MOTOR_H

#include <driver/gpio.h>
#include <driver/ledc.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vt_linalg>

namespace config
{
namespace driver
{
constexpr double BL48250_MAX_VEL_RPM = 3820.0;
constexpr double BL48250_MAX_VEL_RAD =
  (BL48250_MAX_VEL_RPM * 2.0 * M_PI / 60.0);  // rad/s
constexpr double BL48250_MIN_VEL = 0.0;
constexpr uint8_t MOTOR_FORWARD = 1;
constexpr uint8_t MOTOR_BACKWARD = 0;
}  // namespace driver
}  // namespace config

/**
 * @class BL48250_motor
 * @brief Send the needed signals to the 4 BL48250_motor motors of the
 * omnidirectional robot to move it.
 */
class BL48250_motor
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

    std::array<uint8_t, 4> directions_;
    std::array<uint32_t, 4> duty_cycles_;

   public:
    /**
     * @brief Construct a new BL48250_motor object
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
    BL48250_motor(ledc_timer_t timer, ledc_mode_t speed_mode,
                  ledc_timer_bit_t duty_resolution, uint32_t pwm_freq,
                  const std::array<gpio_num_t, 4> & motor_pwm_pins,
                  const std::array<gpio_num_t, 4> & motor_dir_pins,
                  const std::array<ledc_channel_t, 4> & motor_channels);

    /**
     * @brief Destroy the BL48250_motor object
     * @details Stops all motors by setting their duty cycle to 0
     */
    ~BL48250_motor();

    /**
     * @brief Set velocities for all 4 motors simultaneously
     * @param velocities Eigen::Vector4d containing velocities in rad/s
     * @warning Input velocities should be in range [-BL48250_MAX_VEL_RAD,
     * BL48250_MAX_VEL_RAD] converted to rad/s
     */
    void setVelocities(const vt::numeric_vector<4> & velocities);

    /**
     * @brief Print the current driver state via the serial interface.
     * @details Outputs configuration parameters and real-time motor states
     * (direction, duty cycle) for debugging purposes.
     */
    void debugPrint() const;

    BL48250_motor(const BL48250_motor &) = delete;
    BL48250_motor & operator=(const BL48250_motor &) = delete;
};

#endif  // DRIVER_BL48250_MOTOR_H
