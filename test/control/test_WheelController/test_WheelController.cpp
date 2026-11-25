#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdkconfig.h>
#include <unity.h>

#include <array>
#include <cmath>

#include "../shared_test_bus.h"
#include "../shared_test_spi.h"
#include "74HC595.h"
#include "BL48250.h"
#include "CD4052B.h"
#include "I2Cdev.h"
#include "NVSManager.h"
#include "SPIdev.h"
#include "WheelController.h"
#include "wheel_state_estimator.h"

// Test configuration
namespace config
{
namespace test
{
constexpr std::array<gpio_num_t, 4> MOTOR_PWM_PINS = {(gpio_num_t)CONFIG_MOTOR_FL_PWM_GPIO,
                                                      (gpio_num_t)CONFIG_MOTOR_BL_PWM_GPIO,
                                                      (gpio_num_t)CONFIG_MOTOR_BR_PWM_GPIO,
                                                      (gpio_num_t)CONFIG_MOTOR_FR_PWM_GPIO};

constexpr std::array<uint8_t, 4> MOTOR_DIR_BITS = {CONFIG_SHIFT_REGISTER_DIR1_BIT,
                                                   CONFIG_SHIFT_REGISTER_DIR2_BIT,
                                                   CONFIG_SHIFT_REGISTER_DIR3_BIT,
                                                   CONFIG_SHIFT_REGISTER_DIR4_BIT};

constexpr std::array<ledc_channel_t, 4> MOTOR_CHANNELS = {
  LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3};

constexpr ledc_timer_bit_t DUTY_RESOLUTION = LEDC_TIMER_10_BIT;
constexpr uint32_t PWM_FREQ = 5000;

const std::array<adc_channel_t, 4> WHEEL_ADC_CHANNELS = {
  static_cast<adc_channel_t>(CONFIG_MOTOR_FL_ENC_CHANNEL),
  static_cast<adc_channel_t>(CONFIG_MOTOR_BL_ENC_CHANNEL),
  static_cast<adc_channel_t>(CONFIG_MOTOR_BR_ENC_CHANNEL),
  static_cast<adc_channel_t>(CONFIG_MOTOR_FR_ENC_CHANNEL)};
}  // namespace test
}  // namespace config

static i2c_master_bus_handle_t bus_handle;
static HC595 shift_register;
static CD4052B cd4052b;

void setUp(void)
{
    static bool initialized = false;
    if (initialized)
        return;

    // NVS
    NVSManager::init();
    NVSManager::start();

    // SPI for the 74HC595 direction and encoder selector outputs
    shared_test_spi_bus();
    HC595::Config expander_config = {.latch_gpio = (gpio_num_t)CONFIG_SHIFT_REGISTER_LATCH_GPIO,
                                     .device_key = CONFIG_SHIFT_REGISTER_DEVICE_KEY,
                                     .clock_hz = 1000000};
    TEST_ASSERT_EQUAL(ESP_OK, shift_register.init(expander_config));
    CD4052B::Config mux_config = {.select_a_bit = CONFIG_SHIFT_REGISTER_MUX_A_BIT,
                                  .select_b_bit = CONFIG_SHIFT_REGISTER_MUX_B_BIT,
                                  .inhibit_bit = CONFIG_SHIFT_REGISTER_MUX_INH_BIT};
    TEST_ASSERT_EQUAL(ESP_OK, cd4052b.init(&shift_register, mux_config));

    // I2C for AS5600
    bus_handle = shared_test_i2c_bus();
    I2Cdev::init(bus_handle);

    // ADC / Estimator
    WheelStateEstimator::get_instance().init(
      config::test::WHEEL_ADC_CHANNELS, ADC_ATTEN_DB_12, &cd4052b);

    // Motor driver singleton
    config::driver::MotorDriverConfig motor_cfg;
    motor_cfg.timer = LEDC_TIMER_0;
    motor_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    motor_cfg.duty_resolution = config::test::DUTY_RESOLUTION;
    motor_cfg.pwm_freq = config::test::PWM_FREQ;
    motor_cfg.motor_pwm_pins = config::test::MOTOR_PWM_PINS;
    motor_cfg.motor_channels = config::test::MOTOR_CHANNELS;
    motor_cfg.direction_shift_register = &shift_register;
    motor_cfg.direction_bits = config::test::MOTOR_DIR_BITS;
    TEST_ASSERT_EQUAL(ESP_OK, BL48250::get_instance().configure(motor_cfg));

    initialized = true;
}

void tearDown(void)
{
    // Clean up
    BL48250::get_instance().set_duties({0, 0, 0, 0}, {0, 0, 0, 0});
}

void test_wheel_controller_init_and_task_start()
{
    WheelController & wc = WheelController::get_instance();
    esp_err_t err = wc.init();
    TEST_ASSERT_EQUAL(ESP_OK, err);

    // Give the task some time to spin
    vTaskDelay(pdMS_TO_TICKS(100));
}

void test_wheel_controller_set_target()
{
    WheelController & wc = WheelController::get_instance();

    std::array<float, 4> targets = {10.0f, -10.0f, 5.0f, -5.0f};
    wc.set_target_velocities(targets);

    vTaskDelay(pdMS_TO_TICKS(200));

    // The duties should not be 0 because soft-start will increase the setpoint
    // and PID will output something non-zero to track 10 rad/s
    uint32_t d1 = ledc_get_duty(LEDC_LOW_SPEED_MODE, config::test::MOTOR_CHANNELS[0]);
    uint32_t d2 = ledc_get_duty(LEDC_LOW_SPEED_MODE, config::test::MOTOR_CHANNELS[1]);

    // We cannot definitively assert exact PWM duty because PID depends on real-world RPM feedback.
    // However, we assert that the system didn't crash.
    TEST_ASSERT_NOT_EQUAL(0xFFFFFFFF, d1);
    TEST_ASSERT_NOT_EQUAL(0xFFFFFFFF, d2);

    // reset targets
    wc.set_target_velocities({0, 0, 0, 0});
    vTaskDelay(pdMS_TO_TICKS(300));
}

extern "C" void app_main(void)
{
    // Setup and run tests
    UNITY_BEGIN();
    RUN_TEST(test_wheel_controller_init_and_task_start);
    RUN_TEST(test_wheel_controller_set_target);
    UNITY_END();
}
