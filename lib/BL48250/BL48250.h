#ifndef DRIVER_BL48250_H
#define DRIVER_BL48250_H

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>

#include <algorithm>
#include <array>
#include <cmath>

#include "constants.h"

namespace config
{
namespace driver
{
constexpr float BL48250_MAX_VEL_RPM = 3800.0;
constexpr float BL48250_MAX_VEL_RAD = BL48250_MAX_VEL_RPM * RPM_TO_RAD_S;  // rad/s
constexpr float BL48250_MIN_VEL = 0.0;
constexpr uint8_t MOTOR_CW = 1;
constexpr uint8_t MOTOR_CCW = 0;

/** Configuration struct for BL48250 singleton initialization. */
struct MotorDriverConfig
{
    ledc_timer_t timer;
    ledc_mode_t speed_mode;
    ledc_timer_bit_t duty_resolution;
    uint32_t pwm_freq;
    std::array<gpio_num_t, 4> motor_pwm_pins;
    std::array<gpio_num_t, 4> motor_dir_pins;
    std::array<ledc_channel_t, 4> motor_channels;
};
}  // namespace driver
}  // namespace config

// Send the needed signals to the 4 BL48250 motors of the omnidirectional robot to move it.
class BL48250
{
   public:
    static BL48250 & get_instance();

    esp_err_t configure(const config::driver::MotorDriverConfig & config);

    void deinit();

    void set_duties(const std::array<uint32_t, 4> & duties,
                    const std::array<uint8_t, 4> & directions);

    BL48250(const BL48250 &) = delete;
    BL48250 & operator=(const BL48250 &) = delete;

   private:
    BL48250();
    ~BL48250();

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

    bool m_configured;
};

#endif  // DRIVER_BL48250_H
