#include "kinematics/bl48250_motor_driver.h"

BL48250Driver::BL48250Driver(
  ledc_timer_t timer, ledc_mode_t speed_mode, ledc_timer_bit_t duty_resolution,
  uint32_t pwm_freq, const std::array<gpio_num_t, 4> & motor_pwm_pins,
  const std::array<gpio_num_t, 4> & motor_dir_pins,
  const std::array<ledc_channel_t, 4> & motor_channels)
: timer_(timer),
  speed_mode_(speed_mode),
  duty_resolution_(duty_resolution),
  pwm_freq_(pwm_freq),
  motor_pwm_pins_(motor_pwm_pins),
  motor_dir_pins_(motor_dir_pins),
  motor_channels_(motor_channels)
{
    max_duty_ = (1 << static_cast<int>(duty_resolution_)) - 1;

    // Configure PWM timer
    ledc_timer_config_t timer_conf = {.speed_mode = speed_mode_,
                                      .duty_resolution = duty_resolution_,
                                      .timer_num = timer_,
                                      .freq_hz = pwm_freq_,
                                      .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&timer_conf);

    // Configure each motor's PWM channel
    for (int i = 0; i < 4; ++i)
    {
        ledc_channel_config_t channel_conf = {.gpio_num = motor_pwm_pins_[i],
                                              .speed_mode = speed_mode_,
                                              .channel = motor_channels_[i],
                                              .intr_type = LEDC_INTR_DISABLE,
                                              .timer_sel = timer_,
                                              .duty = 0,
                                              .hpoint = 0,
                                              .flags = {.output_invert = 0}};
        ledc_channel_config(&channel_conf);

        gpio_reset_pin(motor_dir_pins_[i]);
        gpio_set_direction(motor_dir_pins_[i], GPIO_MODE_OUTPUT_OD);
        gpio_set_level(motor_dir_pins_[i], MOTOR_FORWARD);
    }
}

BL48250Driver::~BL48250Driver()
{
    // Stop all motors
    for (int i = 0; i < 4; ++i)
    {
        ledc_set_duty(speed_mode_, motor_channels_[i], 0);
        ledc_update_duty(speed_mode_, motor_channels_[i]);
    }
}

void BL48250Driver::setVelocities(const Eigen::Vector4d & velocities)
{
    for (int i = 0; i < 4; ++i)
    {
        gpio_set_level(motor_dir_pins_[i],
                       velocities[i] < 0 ? MOTOR_BACKWARD : MOTOR_FORWARD);

        double vel_abs = std::abs(velocities[i]);
        double duty_float = 0.290329861f * vel_abs - 28.679152042f;
        duty_float = constrain(duty_float, 0.0f, 100.0f);
        uint32_t duty =
          static_cast<uint32_t>((duty_float / 100.0) * max_duty_);

        ledc_set_duty(speed_mode_, motor_channels_[i], duty);
        ledc_update_duty(speed_mode_, motor_channels_[i]);
    }
}
