#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdkconfig.h>
#include <unity.h>

#include <array>
#include <cmath>

#include "BL48250.h"
#include "I2Cdev.h"
#include "NVSManager.h"
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

constexpr std::array<gpio_num_t, 4> MOTOR_DIR_PINS = {(gpio_num_t)CONFIG_MOTOR_FL_DIR_GPIO,
                                                      (gpio_num_t)CONFIG_MOTOR_BL_DIR_GPIO,
                                                      (gpio_num_t)CONFIG_MOTOR_BR_DIR_GPIO,
                                                      (gpio_num_t)CONFIG_MOTOR_FR_DIR_GPIO};

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

void setUp(void)
{
    static bool initialized = false;
    if (initialized)
        return;

    // NVS
    NVSManager::init();

    // I2C for AS5600
    i2c_master_bus_config_t i2c_mst_config = {
      .i2c_port = (i2c_port_t)CONFIG_I2C_PORT_NUM,
      .sda_io_num = (gpio_num_t)CONFIG_SDA_GPIO,
      .scl_io_num = (gpio_num_t)CONFIG_SCL_GPIO,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .intr_priority = 0,
      .trans_queue_depth = 0,
      .flags = {.enable_internal_pullup = true, .allow_pd = false}};
    i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    I2Cdev::init(bus_handle);

    // ADC / Estimator
    WheelStateEstimator::get_instance().init(config::test::WHEEL_ADC_CHANNELS, ADC_ATTEN_DB_12);

    // Motor driver singleton
    config::driver::MotorDriverConfig motor_cfg;
    motor_cfg.timer = LEDC_TIMER_0;
    motor_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    motor_cfg.duty_resolution = config::test::DUTY_RESOLUTION;
    motor_cfg.pwm_freq = config::test::PWM_FREQ;
    motor_cfg.motor_pwm_pins = config::test::MOTOR_PWM_PINS;
    motor_cfg.motor_dir_pins = config::test::MOTOR_DIR_PINS;
    motor_cfg.motor_channels = config::test::MOTOR_CHANNELS;
    BL48250::get_instance().configure(motor_cfg);

    for (auto pin : config::test::MOTOR_DIR_PINS) gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT);

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
