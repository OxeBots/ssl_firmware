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

// Send the needed signals to the 4 BL48250 motors of the omnidirectional robot to move it.
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
    BL48250(ledc_timer_t timer, ledc_mode_t speed_mode, ledc_timer_bit_t duty_resolution, uint32_t pwm_freq,
            const std::array<gpio_num_t, 4> & motor_pwm_pins, const std::array<gpio_num_t, 4> & motor_dir_pins,
            const std::array<ledc_channel_t, 4> & motor_channels);

    ~BL48250();

    void set_duties(const std::array<uint32_t, 4> & duties, const std::array<uint8_t, 4> & directions);

    void debugPrint() const;

    BL48250(const BL48250 &) = delete;
    BL48250 & operator=(const BL48250 &) = delete;
};

#endif  // DRIVER_BL48250_H
