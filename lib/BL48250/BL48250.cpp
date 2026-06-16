#include "BL48250.h"

#include "helper_func.h"

using namespace config::driver;

static const char * TAG = "BL48250_driver";

BL48250 & BL48250::get_instance()
{
    static BL48250 instance;
    return instance;
}

BL48250::BL48250()
: timer_(LEDC_TIMER_0),
  speed_mode_(LEDC_LOW_SPEED_MODE),
  duty_resolution_(LEDC_TIMER_10_BIT),
  pwm_freq_(20000),
  motor_pwm_pins_{},
  motor_dir_pins_{},
  motor_channels_{},
  max_duty_(0),
  directions_{MOTOR_CW, MOTOR_CW, MOTOR_CW, MOTOR_CW},
  duty_cycles_{0, 0, 0, 0},
  m_configured(false)
{
}

BL48250::~BL48250()
{
    if (m_configured)
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
}

/**
 * @brief Configure the motor driver with GPIO pins, PWM settings, and LEDC channels.
 *
 * Must be called once before any set_duties() calls. Initializes the LEDC timer,
 * configures 4 PWM channels, and sets up direction pins.
 *
 * @param config MotorDriverConfig struct with PWM frequency, pins, channels, etc.
 * @return ESP_OK on success
 */
esp_err_t BL48250::configure(const MotorDriverConfig & config)
{
    if (m_configured)
        return ESP_OK;

    timer_ = config.timer;
    speed_mode_ = config.speed_mode;
    duty_resolution_ = config.duty_resolution;
    pwm_freq_ = config.pwm_freq;
    motor_pwm_pins_ = config.motor_pwm_pins;
    motor_dir_pins_ = config.motor_dir_pins;
    motor_channels_ = config.motor_channels;

    max_duty_ = (1 << static_cast<int>(duty_resolution_)) - 1;

    // Configure PWM timer
    ledc_timer_config_t timer_conf = {.speed_mode = speed_mode_,
                                      .duty_resolution = duty_resolution_,
                                      .timer_num = timer_,
                                      .freq_hz = pwm_freq_,
                                      .clk_cfg = LEDC_USE_APB_CLK,
                                      .deconfigure = false};
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(err));
        return err;
    }

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
          .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
          .flags = {.output_invert = 0},
        };
        err = ledc_channel_config(&channel_conf);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to configure LEDC channel %zu: %s", i, esp_err_to_name(err));
            return err;
        }

        gpio_reset_pin(motor_dir_pins_[i]);
        gpio_set_direction(motor_dir_pins_[i], GPIO_MODE_OUTPUT_OD);
        gpio_set_level(motor_dir_pins_[i], MOTOR_CW);

        directions_[i] = MOTOR_CW;
        duty_cycles_[i] = 0;
    }

    m_configured = true;
    ESP_LOGI(TAG,
             "BL48250 configured. PWM: %luHz, %d-bit duty",
             (unsigned long)pwm_freq_,
             static_cast<int>(duty_resolution_));
    return ESP_OK;
}

/**
 * @brief Deinitialize the motor driver: stop all motors, reset GPIO pins, clear state.
 *
 * Safe to call multiple times. After deinit(), configure() can be called again.
 */
void BL48250::deinit()
{
    if (!m_configured)
        return;

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

    m_configured = false;
    ESP_LOGI(TAG, "BL48250 deinitialized.");
}

/**
 * @brief Set PWM duty cycles and directions for all 4 motors.
 *
 * @param duties Array of 4 duty cycle values (0 to max_duty_)
 * @param directions Array of 4 direction values (MOTOR_CW or MOTOR_CCW)
 */
void BL48250::set_duties(const std::array<uint32_t, 4> & duties,
                         const std::array<uint8_t, 4> & directions)
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
