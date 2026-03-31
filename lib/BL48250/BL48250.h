/**
 * @file BL48250.h
 * @brief Driver for controlling BL48250 motors using ESP32's LEDC peripheral.
 *
 * This class provides an interface to control up to 4 BL48250 motors, allowing
 * for setting individual motor velocities and directions. It abstracts away the
 * details of PWM signal generation and GPIO control, making it easier to integrate
 * the motors into the omnidirectional robot's control system.
 *
 * The driver uses the LEDC (LED Control) peripheral of the ESP32 to generate PWM signals
 * for motor speed control, and GPIO pins for direction control. It supports configurable
 * PWM frequency and resolution, allowing for fine-tuning of motor performance.
 */
#ifndef DRIVER_BL48250_H
#define DRIVER_BL48250_H

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace config
{
namespace driver
{
constexpr float BL48250_MAX_VEL_RPM = 3800.0;
constexpr float BL48250_MAX_VEL_RAD = (BL48250_MAX_VEL_RPM * 2.0 * M_PI / 60.0);  // rad/s
constexpr float BL48250_MIN_VEL = 0.0;
constexpr uint8_t MOTOR_CW = 1;
constexpr uint8_t MOTOR_CCW = 0;
}  // namespace driver
}  // namespace config

/**
 * @class BL48250
 * @brief Send the needed signals to the 4 BL48250 motors of the
 * omnidirectional robot to move it.
 */
class BL48250
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
     * @brief Construct a new BL48250 object
     * @param timer LEDC timer to use for all PWM channels
     * @param speed_mode LEDC speed mode (LEDC_LOW_SPEED_MODE or LEDC_HIGH_SPEED_MODE)
     * @param duty_resolution Bit resolution of PWM duty cycle
     * @param pwm_freq PWM frequency in Hz
     * @param motor_pwm_pins Array of 4 GPIO pins for PWM signals (one per motor)
     * @param motor_dir_pins Array of 4 GPIO pins for direction control (one per motor)
     * @param motor_channels Array of 4 LEDC channels (one per motor)
     */
    BL48250(ledc_timer_t timer, ledc_mode_t speed_mode, ledc_timer_bit_t duty_resolution, uint32_t pwm_freq,
            const std::array<gpio_num_t, 4> & motor_pwm_pins, const std::array<gpio_num_t, 4> & motor_dir_pins,
            const std::array<ledc_channel_t, 4> & motor_channels);

    /**
     * @brief Destroy the BL48250 object
     * @details Stops all motors by setting their duty cycle to 0
     */
    ~BL48250();

    /**
     * @brief Set raw PWM duty cycles and directions for all 4 motors simultaneously
     * @param duties std::array<uint32_t, 4> containing raw PWM duty cycle values
     * @param directions std::array<uint8_t, 4> containing direction values (MOTOR_CW or MOTOR_CCW)
     */
    void set_duties(const std::array<uint32_t, 4> & duties, const std::array<uint8_t, 4> & directions);

    /**
     * @brief Print the current driver state via the serial interface.
     * @details Outputs configuration parameters and real-time motor states (direction, duty cycle)
     * for debugging purposes.
     */
    void debugPrint() const;

    BL48250(const BL48250 &) = delete;
    BL48250 & operator=(const BL48250 &) = delete;
};

#endif  // DRIVER_BL48250_H
