#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <unity.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

#include "kinematics/bl48250_motor_driver.cpp" // Include the implementation file for testing
#include "kinematics/bl48250_motor_driver.h"
#include "pins_assignments.h"

// Test configuration
namespace TestConfig
{
constexpr std::array<gpio_num_t, 4> MOTOR_PWM_PINS = {
  PIN_MOTOR_FRONT_LEFT_PWM, PIN_MOTOR_BACK_LEFT_PWM, PIN_MOTOR_BACK_RIGHT_PWM,
  PIN_MOTOR_FRONT_RIGHT_PWM};

constexpr std::array<gpio_num_t, 4> MOTOR_DIR_PINS = {
  PIN_MOTOR_FRONT_LEFT_DIR, PIN_MOTOR_BACK_LEFT_DIR, PIN_MOTOR_BACK_RIGHT_DIR,
  PIN_MOTOR_FRONT_RIGHT_DIR};

constexpr std::array<ledc_channel_t, 4> MOTOR_CHANNELS = {
  LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3, LEDC_CHANNEL_4};

constexpr ledc_timer_bit_t DUTY_RESOLUTION = LEDC_TIMER_10_BIT;
constexpr uint32_t PWM_FREQ = 5000;
constexpr uint32_t MAX_DUTY = (1 << static_cast<int>(DUTY_RESOLUTION)) - 1;
}  // namespace TestConfig

// Helper macros
#define DELAY_MS(ms) vTaskDelay((ms) / portTICK_PERIOD_MS)

// Helper functions
std::unique_ptr<BL48250Driver> create_driver()
{
    std::unique_ptr<BL48250Driver> driver(new BL48250Driver(
      LEDC_TIMER_0, LEDC_HIGH_SPEED_MODE, TestConfig::DUTY_RESOLUTION,
      TestConfig::PWM_FREQ, TestConfig::MOTOR_PWM_PINS,
      TestConfig::MOTOR_DIR_PINS, TestConfig::MOTOR_CHANNELS));

    // necessary input/output to read the pins in tests
    for (auto pin : TestConfig::MOTOR_DIR_PINS)
        gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT);

    return driver;
}

uint32_t calculate_expected_duty(double velocity)
{
    const double linear_term = 0.290329861 * std::abs(velocity) - 28.679152042;
    const double clamped = constrain(linear_term, 0.0, 100.0);
    return static_cast<uint32_t>((clamped / 100.0) * TestConfig::MAX_DUTY);
}

void assert_motor_direction(size_t motor_idx, int expected,
                            const char * context)
{
    const int actual = gpio_get_level(TestConfig::MOTOR_DIR_PINS[motor_idx]);
    char msg[128];
    snprintf(msg, sizeof(msg), "Motor %zu: %s (expected %d, got %d)",
             motor_idx, context, expected, actual);
    TEST_ASSERT_EQUAL_MESSAGE(expected, actual, msg);
}

void assert_motor_duty(size_t motor_idx, uint32_t expected,
                       const char * context)
{
    const uint32_t actual = ledc_get_duty(
      LEDC_HIGH_SPEED_MODE, TestConfig::MOTOR_CHANNELS[motor_idx]);
    char msg[128];
    snprintf(msg, sizeof(msg), "Motor %zu: %s (expected %u, got %u)",
             motor_idx, context, expected, actual);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(expected, actual, msg);
}

// Test cases
void test_setVelocities_positive_values()
{
    auto driver = create_driver();
    const Eigen::Vector4d velocities(100.0, 200.0, 300.0, 400.0);
    driver->setVelocities(velocities);

    DELAY_MS(10);

    for (size_t i = 0; i < 4; ++i)
        assert_motor_direction(i, MOTOR_FORWARD, "positive velocity");

    for (size_t i = 0; i < 4; ++i)
        assert_motor_duty(i, calculate_expected_duty(velocities[i]),
                          "positive velocity duty mismatch");
}

void test_setVelocities_negative_values()
{
    auto driver = create_driver();
    const Eigen::Vector4d velocities(-100.0, -200.0, -300.0, -400.0);
    driver->setVelocities(velocities);

    DELAY_MS(10);

    for (size_t i = 0; i < 4; ++i)
        assert_motor_direction(i, MOTOR_BACKWARD, "negative velocity");

    for (size_t i = 0; i < 4; ++i)
        assert_motor_duty(i, calculate_expected_duty(velocities[i]),
                          "negative velocity duty mismatch");
}

void test_setVelocities_out_of_range_values()
{
    auto driver = create_driver();
    const Eigen::Vector4d velocities(50.0, 0.0, -50.0,
                                     BL48250_MAX_VEL_RAD * 2.0);
    driver->setVelocities(velocities);

    DELAY_MS(10);

    assert_motor_direction(0, MOTOR_FORWARD, "velocity 50.0");
    assert_motor_direction(1, MOTOR_FORWARD, "velocity 0.0");
    assert_motor_direction(2, MOTOR_BACKWARD, "velocity -50.0");
    assert_motor_direction(3, MOTOR_FORWARD, "over max velocity");

    assert_motor_duty(0, calculate_expected_duty(50.0), "velocity 50.0");
    assert_motor_duty(1, calculate_expected_duty(0.0), "velocity 0.0");
    assert_motor_duty(2, calculate_expected_duty(-50.0), "velocity -50.0");
    assert_motor_duty(3, TestConfig::MAX_DUTY, "over max velocity");
}

void test_setVelocities_zero_input_stop_motors()
{
    auto driver = create_driver();
    driver->setVelocities(Eigen::Vector4d::Zero());

    DELAY_MS(10);

    for (size_t i = 0; i < 4; ++i)
    {
        assert_motor_direction(i, MOTOR_FORWARD, "zero velocity");
        assert_motor_duty(i, 0, "zero velocity duty");
    }
}

void test_destructor_reset_outputs()
{
    {
        auto driver = create_driver();
        driver->setVelocities(Eigen::Vector4d(100.0, 200.0, 300.0, 400.0));
        DELAY_MS(10);
    }  // Driver destroyed here

    DELAY_MS(10);

    for (size_t i = 0; i < 4; ++i)
        assert_motor_duty(i, 0, "post-destructor duty");
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_setVelocities_positive_values);
    RUN_TEST(test_setVelocities_negative_values);
    RUN_TEST(test_setVelocities_out_of_range_values);
    RUN_TEST(test_setVelocities_zero_input_stop_motors);
    RUN_TEST(test_destructor_reset_outputs);
    UNITY_END();
}

void loop() {}
