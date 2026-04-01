#include "BL48250.h"

#include "helper_func.h"

using namespace config::driver;

static const char * TAG = "BL48250_driver";

BL48250::BL48250(ledc_timer_t timer, ledc_mode_t speed_mode, ledc_timer_bit_t duty_resolution, uint32_t pwm_freq,
                 const std::array<gpio_num_t, 4> & motor_pwm_pins, const std::array<gpio_num_t, 4> & motor_dir_pins,
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
                                      .clk_cfg = LEDC_USE_APB_CLK,
                                      .deconfigure = false};
    ledc_timer_config(&timer_conf);

    // Configure each motor's PWM channel
    for (size_t i = 0; i < 4; ++i)
    {
        ledc_channel_config_t channel_conf = {
          .gpio_num = motor_pwm_pins_[i],
          .speed_mode = speed_mode_,
          .channel = motor_channels_[i],
          .intr_type = LEDC_INTR_DISABLE,
          .timer_sel = timer_,
          .duty = 0,
          .hpoint = 0,
          .sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE,
          .flags = {.output_invert = 0},
        };
        ledc_channel_config(&channel_conf);

        gpio_reset_pin(motor_dir_pins_[i]);
        gpio_set_direction(motor_dir_pins_[i], GPIO_MODE_OUTPUT_OD);
        gpio_set_level(motor_dir_pins_[i], MOTOR_CW);

        directions_[i] = MOTOR_CW;
        duty_cycles_[i] = 0;
    }
}

BL48250::~BL48250()
{
    // Stop all motors
    for (size_t i = 0; i < 4; ++i)
    {
        ledc_set_duty(speed_mode_, motor_channels_[i], 0);
        ledc_update_duty(speed_mode_, motor_channels_[i]);
    }
    // Reset the GPIO pins to release them from the LEDC peripheral
    for (auto pin : motor_pwm_pins_)
    {
        gpio_reset_pin(pin);
    }
    for (auto pin : motor_dir_pins_)
    {
        gpio_reset_pin(pin);
    }
}

/**
 * @brief Set PWM duty cycles and directions for all 4 motors.
 *
 * @param duties Array of 4 duty cycle values (0 to max_duty_)
 * @param directions Array of 4 direction values (MOTOR_CW or MOTOR_CCW)
 */
void BL48250::set_duties(const std::array<uint32_t, 4> & duties, const std::array<uint8_t, 4> & directions)
{
    for (size_t i = 0; i < 4; ++i)
    {
        directions_[i] = directions[i];
        gpio_set_level(motor_dir_pins_[i], directions[i]);

        duty_cycles_[i] = duties[i];
        ledc_set_duty(speed_mode_, motor_channels_[i], duties[i]);
        ledc_update_duty(speed_mode_, motor_channels_[i]);
    }
}

/**
 * @brief Print driver state (PWM config, duty cycles, directions) to serial log.
 *
 * Outputs PWM frequency, resolution, and per-motor GPIO assignments, direction,
 * and duty cycle percentages. Useful for debugging motor driver configuration.
 */
void BL48250::debugPrint() const
{
    ESP_LOGI(TAG, "\nBL48250 Driver State:");
    ESP_LOGI(TAG, "PWM Frequency: %uHz, Resolution: %d bits\n", pwm_freq_, static_cast<int>(duty_resolution_));
    ESP_LOGI(TAG, "Max Duty: %u (%.1f%%)\n", max_duty_, 100.0);

    for (size_t i = 0; i < 4; ++i)
    {
        ESP_LOGI(TAG, "\nMotor %d:", i + 1);
        ESP_LOGI(TAG, "  PWM: GPIO %-2d (Ch %d)", motor_pwm_pins_[i], motor_channels_[i]);
        ESP_LOGI(TAG, "\n  DIR: GPIO %-2d -> %s", motor_dir_pins_[i], directions_[i] ? "CW" : "CCW");
        ESP_LOGI(TAG, "\n  Duty: %-5u (%.1f%%)", duty_cycles_[i], (duty_cycles_[i] * 100.0) / max_duty_);
    }
    ESP_LOGI(TAG, "\n-----------------------");
}
