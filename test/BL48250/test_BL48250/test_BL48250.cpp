#include <BL48250.h>
#include <freertos/FreeRTOS.h>
#include <sdkconfig.h>
#include <unity.h>

#include <algorithm>
#include <array>
#include <cinttypes>
#include <cmath>
#include <memory>
#include <sdkconfig.h>

// Test configuration
namespace config
{
    namespace test
    {
        constexpr std::array<gpio_num_t, 4> MOTOR_PWM_PINS = {
            (gpio_num_t)CONFIG_MOTOR_FL_PWM_GPIO, (gpio_num_t)CONFIG_MOTOR_BL_PWM_GPIO, (gpio_num_t)CONFIG_MOTOR_BR_PWM_GPIO,
            (gpio_num_t)CONFIG_MOTOR_FR_PWM_GPIO};

        constexpr std::array<gpio_num_t, 4> MOTOR_DIR_PINS = {
            (gpio_num_t)CONFIG_MOTOR_FL_DIR_GPIO, (gpio_num_t)CONFIG_MOTOR_BL_DIR_GPIO, (gpio_num_t)CONFIG_MOTOR_BR_DIR_GPIO,
            (gpio_num_t)CONFIG_MOTOR_FR_DIR_GPIO};

        constexpr std::array<ledc_channel_t, 4> MOTOR_CHANNELS = {LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3,
                                                                  LEDC_CHANNEL_4};

        constexpr ledc_timer_bit_t DUTY_RESOLUTION = LEDC_TIMER_10_BIT;
        constexpr uint32_t PWM_FREQ = 5000;
        constexpr uint32_t MAX_DUTY = (1 << static_cast<int>(DUTY_RESOLUTION)) - 1;
    } // namespace test
} // namespace config

// Helper macros
#define DELAY_MS(ms) vTaskDelay((ms) / portTICK_PERIOD_MS)

// Helper functions
std::unique_ptr<BL48250> create_driver()
{
    std::unique_ptr<BL48250> driver = std::make_unique<BL48250>(
        LEDC_TIMER_0, LEDC_LOW_SPEED_MODE, config::test::DUTY_RESOLUTION, config::test::PWM_FREQ,
        config::test::MOTOR_PWM_PINS, config::test::MOTOR_DIR_PINS, config::test::MOTOR_CHANNELS);

    // necessary input/output to read the pins in tests
    for (auto pin : config::test::MOTOR_DIR_PINS)
        gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT);

    return driver;
}

void assert_motor_direction(size_t motor_idx, int expected, const char *context)
{
    const int actual = gpio_get_level(config::test::MOTOR_DIR_PINS[motor_idx]);
    char msg[128];
    snprintf(msg, sizeof(msg), "Motor %zu: %s (expected %d, got %d)", motor_idx, context, expected, actual);
    TEST_ASSERT_EQUAL_MESSAGE(expected, actual, msg);
}

void assert_motor_duty(size_t motor_idx, uint32_t expected, const char *context)
{
    const uint32_t actual = ledc_get_duty(LEDC_LOW_SPEED_MODE, config::test::MOTOR_CHANNELS[motor_idx]);
    char msg[128];
    snprintf(msg, sizeof(msg), "Motor %zu: %s (expected %" PRIu32 ", got %" PRIu32 ")", motor_idx, context, expected,
             actual);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(expected, actual, msg);
}

// Test cases
void test_set_duties_cw()
{
    auto driver = create_driver();
    const std::array<uint32_t, 4> duties = {100, 200, 300, 400};
    const std::array<uint8_t, 4> dirs = {config::driver::MOTOR_CW, config::driver::MOTOR_CW,
                                         config::driver::MOTOR_CW, config::driver::MOTOR_CW};
    driver->set_duties(duties, dirs);

    DELAY_MS(20);

    for (size_t i = 0; i < 4; ++i)
        assert_motor_direction(i, config::driver::MOTOR_CW, "cw");
    for (size_t i = 0; i < 4; ++i)
        assert_motor_duty(i, duties[i], "cw duty mismatch");
}

void test_set_duties_ccw()
{
    auto driver = create_driver();
    const std::array<uint32_t, 4> duties = {100, 200, 300, 400};
    const std::array<uint8_t, 4> dirs = {config::driver::MOTOR_CCW, config::driver::MOTOR_CCW,
                                         config::driver::MOTOR_CCW, config::driver::MOTOR_CCW};
    driver->set_duties(duties, dirs);

    DELAY_MS(20);

    for (size_t i = 0; i < 4; ++i)
        assert_motor_direction(i, config::driver::MOTOR_CCW, "ccw");
    for (size_t i = 0; i < 4; ++i)
        assert_motor_duty(i, duties[i], "ccw duty mismatch");
}

void test_set_duties_mixed()
{
    auto driver = create_driver();
    const std::array<uint32_t, 4> duties = {500, 0, 1023, 256};
    const std::array<uint8_t, 4> dirs = {config::driver::MOTOR_CW, config::driver::MOTOR_CW,
                                         config::driver::MOTOR_CCW, config::driver::MOTOR_CW};
    driver->set_duties(duties, dirs);

    DELAY_MS(20);

    assert_motor_direction(0, config::driver::MOTOR_CW, "mixed 0");
    assert_motor_direction(1, config::driver::MOTOR_CW, "mixed 1");
    assert_motor_direction(2, config::driver::MOTOR_CCW, "mixed 2");
    assert_motor_direction(3, config::driver::MOTOR_CW, "mixed 3");

    assert_motor_duty(0, 500, "mixed duty 0");
    assert_motor_duty(1, 0, "mixed duty 1");
    assert_motor_duty(2, 1023, "mixed duty 2");
    assert_motor_duty(3, 256, "mixed duty 3");
}

void test_set_duties_zero()
{
    auto driver = create_driver();
    driver->set_duties({0, 0, 0, 0}, {config::driver::MOTOR_CW, config::driver::MOTOR_CW,
                                      config::driver::MOTOR_CW, config::driver::MOTOR_CW});

    DELAY_MS(20);

    for (size_t i = 0; i < 4; ++i)
    {
        assert_motor_direction(i, config::driver::MOTOR_CW, "zero duty");
        assert_motor_duty(i, 0, "zero duty mismatch");
    }
}

void test_destructor_reset_outputs()
{
    {
        auto driver = create_driver();
        driver->set_duties({100, 200, 300, 350}, {config::driver::MOTOR_CW, config::driver::MOTOR_CW,
                                                  config::driver::MOTOR_CW, config::driver::MOTOR_CW});
        DELAY_MS(20);
    } // Driver destroyed here

    DELAY_MS(20);

    for (size_t i = 0; i < 4; ++i)
        assert_motor_duty(i, 0, "post-destructor duty");
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_set_duties_cw);
    RUN_TEST(test_set_duties_ccw);
    RUN_TEST(test_set_duties_mixed);
    RUN_TEST(test_set_duties_zero);
    RUN_TEST(test_destructor_reset_outputs);
    UNITY_END();
}

extern "C" void app_main(void)
{
    setup();
}
