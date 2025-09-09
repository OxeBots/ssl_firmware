#include <freertos/FreeRTOS.h>
#include <unity.h>

#include <algorithm>
#include <array>
#include <cinttypes>
#include <cmath>
#include <memory>

#include "driver/bl48250.cpp"  // Include the implementation file for testing
#include "driver/bl48250.hpp"
#include "pins_assignments.h"

// Test configuration
namespace config
{
namespace test
{
constexpr std::array<gpio_num_t, 4> MOTOR_PWM_PINS = {
  config::pin::MOTOR_FRONT_LEFT_PWM, config::pin::MOTOR_BACK_LEFT_PWM, config::pin::MOTOR_BACK_RIGHT_PWM,
  config::pin::MOTOR_FRONT_RIGHT_PWM};

constexpr std::array<gpio_num_t, 4> MOTOR_DIR_PINS = {
  config::pin::MOTOR_FRONT_LEFT_DIR, config::pin::MOTOR_BACK_LEFT_DIR, config::pin::MOTOR_BACK_RIGHT_DIR,
  config::pin::MOTOR_FRONT_RIGHT_DIR};

constexpr std::array<ledc_channel_t, 4> MOTOR_CHANNELS = {LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3,
                                                          LEDC_CHANNEL_4};

constexpr ledc_timer_bit_t DUTY_RESOLUTION = LEDC_TIMER_10_BIT;
constexpr uint32_t PWM_FREQ = 5000;
constexpr uint32_t MAX_DUTY = (1 << static_cast<int>(DUTY_RESOLUTION)) - 1;
}  // namespace test
}  // namespace config

// Helper macros
#define DELAY_MS(ms) vTaskDelay((ms) / portTICK_PERIOD_MS)

// Helper functions
std::unique_ptr<BL48250> create_driver()
{
    std::unique_ptr<BL48250> driver = std::make_unique<BL48250>(
      LEDC_TIMER_0, LEDC_HIGH_SPEED_MODE, config::test::DUTY_RESOLUTION, config::test::PWM_FREQ,
      config::test::MOTOR_PWM_PINS, config::test::MOTOR_DIR_PINS, config::test::MOTOR_CHANNELS);

    // necessary input/output to read the pins in tests
    for (auto pin : config::test::MOTOR_DIR_PINS) gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT);

    return driver;
}

void assert_motor_direction(size_t motor_idx, int expected, const char * context)
{
    const int actual = gpio_get_level(config::test::MOTOR_DIR_PINS[motor_idx]);
    char msg[128];
    snprintf(msg, sizeof(msg), "Motor %zu: %s (expected %d, got %d)", motor_idx, context, expected, actual);
    TEST_ASSERT_EQUAL_MESSAGE(expected, actual, msg);
}

void assert_motor_duty(size_t motor_idx, uint32_t expected, const char * context)
{
    const uint32_t actual = ledc_get_duty(LEDC_HIGH_SPEED_MODE, config::test::MOTOR_CHANNELS[motor_idx]);
    char msg[128];
    snprintf(msg, sizeof(msg), "Motor %zu: %s (expected %" PRIu32 ", got %" PRIu32 ")", motor_idx, context,
             expected, actual);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(expected, actual, msg);
}

// Test cases
void test_setVelocities_positive_values()
{
    // FIXME: something is wrong with this test, it fails even though the code is correct
    auto driver = create_driver();
    const std::array<float, 4> velocities = {100.0, 200.0, 300.0, 400.0};
    driver->setVelocities(velocities);

    DELAY_MS(20);

    for (size_t i = 0; i < 4; ++i) assert_motor_direction(i, config::driver::MOTOR_FORWARD, "positive velocity");

    for (size_t i = 0; i < 4; ++i)
    {
        uint32_t expected_duty = BL48250::velocityToDuty(velocities[i], config::test::MAX_DUTY);
        assert_motor_duty(i, expected_duty, "positive velocity duty mismatch");
    }
}

void test_setVelocities_negative_values()
{
    auto driver = create_driver();
    const std::array<float, 4> velocities = {-100.0, -200.0, -300.0, -400.0};
    driver->setVelocities(velocities);

    DELAY_MS(20);

    for (size_t i = 0; i < 4; ++i) assert_motor_direction(i, config::driver::MOTOR_BACKWARD, "negative velocity");

    for (size_t i = 0; i < 4; ++i)
    {
        uint32_t expected_duty = BL48250::velocityToDuty(velocities[i], config::test::MAX_DUTY);
        assert_motor_duty(i, expected_duty, "negative velocity duty mismatch");
    }
}

void test_setVelocities_out_of_range_values()
{
    auto driver = create_driver();
    const std::array<float, 4> velocities = {50.0, 0.0, -50.0, BL48250_MAX_VEL_RAD * 2.0};
    driver->setVelocities(velocities);

    DELAY_MS(20);

    assert_motor_direction(0, config::driver::MOTOR_FORWARD, "velocity 50.0");
    assert_motor_direction(1, config::driver::MOTOR_FORWARD, "velocity 0.0");
    assert_motor_direction(2, config::driver::MOTOR_BACKWARD, "velocity -50.0");
    assert_motor_direction(3, config::driver::MOTOR_FORWARD, "over max velocity");

    assert_motor_duty(0, BL48250::velocityToDuty(50.0, config::test::MAX_DUTY), "velocity 50.0");
    assert_motor_duty(1, BL48250::velocityToDuty(0.0, config::test::MAX_DUTY), "velocity 0.0");
    assert_motor_duty(2, BL48250::velocityToDuty(-50.0, config::test::MAX_DUTY), "velocity -50.0");
    assert_motor_duty(3, config::test::MAX_DUTY, "over max velocity");
}

void test_setVelocities_zero_input_stop_motors()
{
    auto driver = create_driver();
    driver->setVelocities(std::array<float, 4>({0.0, 0.0, 0.0, 0.0}));

    DELAY_MS(20);

    for (size_t i = 0; i < 4; ++i)
    {
        assert_motor_direction(i, config::driver::MOTOR_FORWARD, "zero velocity");
        assert_motor_duty(i, 0, "zero velocity duty");
    }
}

void test_destructor_reset_outputs()
{
    {
        auto driver = create_driver();
        driver->setVelocities(std::array<float, 4>({100.0, 200.0, 300.0, 350.0}));
        DELAY_MS(20);
    }  // Driver destroyed here

    DELAY_MS(20);

    for (size_t i = 0; i < 4; ++i) assert_motor_duty(i, 0, "post-destructor duty");
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

extern "C" void app_main(void)
{
    setup();
}

