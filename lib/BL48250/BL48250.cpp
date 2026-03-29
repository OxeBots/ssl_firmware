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
                                      .clk_cfg = LEDC_AUTO_CLK,
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
        gpio_set_level(motor_dir_pins_[i], MOTOR_FORWARD);

        directions_[i] = MOTOR_FORWARD;
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
}

uint32_t BL48250::velocityToDuty(float velocity, uint32_t max_duty)
{
    const float vel_abs = std::abs(velocity);
    const float duty_float = constrain(0.290329861f * vel_abs - 28.679152042f, 0.0f, 100.0f);
    return static_cast<uint32_t>((duty_float / 100.0f) * max_duty);
}

void BL48250::setVelocities(const std::array<float, 4> & velocities)
{
    for (size_t i = 0; i < 4; ++i)
    {
        const uint8_t dir = velocities[i] < 0 ? MOTOR_BACKWARD : MOTOR_FORWARD;
        directions_[i] = dir;
        gpio_set_level(motor_dir_pins_[i], dir);

        const uint32_t duty = velocityToDuty(velocities[i], max_duty_);

        duty_cycles_[i] = duty;
        ledc_set_duty(speed_mode_, motor_channels_[i], duty);
        ledc_update_duty(speed_mode_, motor_channels_[i]);
    }
}

void BL48250::debugPrint() const
{
    ESP_LOGI(TAG, "\nBL48250 Driver State:");
    ESP_LOGI(TAG, "PWM Frequency: %uHz, Resolution: %d bits\n", pwm_freq_, static_cast<int>(duty_resolution_));
    ESP_LOGI(TAG, "Max Duty: %u (%.1f%%)\n", max_duty_, 100.0);

    for (size_t i = 0; i < 4; ++i)
    {
        ESP_LOGI(TAG, "\nMotor %d:", i + 1);
        ESP_LOGI(TAG, "  PWM: GPIO %-2d (Ch %d)", motor_pwm_pins_[i], motor_channels_[i]);
        ESP_LOGI(TAG, "\n  DIR: GPIO %-2d -> %s", motor_dir_pins_[i], directions_[i] ? "FORWARD" : "BACKWARD");
        ESP_LOGI(TAG, "\n  Duty: %-5u (%.1f%%)", duty_cycles_[i], (duty_cycles_[i] * 100.0) / max_duty_);
    }
    ESP_LOGI(TAG, "\n-----------------------");
}
