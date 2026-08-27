#include <BL48250.h>
#include <DigitalOutputExpander.h>
#include <SPIdev.h>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <sdkconfig.h>
#include <unity.h>

#include <algorithm>
#include <array>
#include <cinttypes>
#include <cmath>

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
  LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3, LEDC_CHANNEL_4};

constexpr ledc_timer_bit_t DUTY_RESOLUTION = LEDC_TIMER_10_BIT;
constexpr uint32_t PWM_FREQ = 5000;
constexpr uint32_t MAX_DUTY = (1 << static_cast<int>(DUTY_RESOLUTION)) - 1;
}  // namespace test
}  // namespace config

// Helper macros
#define DELAY_MS(ms) vTaskDelay((ms) / portTICK_PERIOD_MS)

DigitalOutputExpander & output_expander()
{
    static DigitalOutputExpander expander;
    static bool initialized = false;
    if (!initialized)
    {
        spi_bus_config_t bus_config = {};
        bus_config.mosi_io_num = (gpio_num_t)CONFIG_MOSI_GPIO;
        bus_config.miso_io_num = (gpio_num_t)CONFIG_MISO_GPIO;
        bus_config.sclk_io_num = (gpio_num_t)CONFIG_SCLK_GPIO;
        bus_config.quadwp_io_num = -1;
        bus_config.quadhd_io_num = -1;
        TEST_ASSERT_EQUAL(ESP_OK, spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO));
        SPIdev::init(SPI2_HOST);

        DigitalOutputExpander::Config expander_config = {
          .latch_gpio = (gpio_num_t)CONFIG_SHIFT_REGISTER_LATCH_GPIO,
          .device_key = CONFIG_SHIFT_REGISTER_DEVICE_KEY,
          .clock_hz = 1000000};
        TEST_ASSERT_EQUAL(ESP_OK, expander.init(expander_config));
        initialized = true;
    }
    return expander;
}

void configure_driver()
{
    auto & expander = output_expander();
    config::driver::MotorDriverConfig motor_cfg;
    motor_cfg.timer = LEDC_TIMER_0;
    motor_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    motor_cfg.duty_resolution = config::test::DUTY_RESOLUTION;
    motor_cfg.pwm_freq = config::test::PWM_FREQ;
    motor_cfg.motor_pwm_pins = config::test::MOTOR_PWM_PINS;
    motor_cfg.motor_channels = config::test::MOTOR_CHANNELS;
    motor_cfg.direction_expander = &expander;
    motor_cfg.direction_bits = config::test::MOTOR_DIR_BITS;
    TEST_ASSERT_EQUAL(ESP_OK, BL48250::get_instance().configure(motor_cfg));
}

void assert_motor_direction(size_t motor_idx, int expected, const char * context)
{
    const int actual =
      (output_expander().get_output() >> config::test::MOTOR_DIR_BITS[motor_idx]) & 1U;
    char msg[128];
    snprintf(msg,
             sizeof(msg),
             "Motor %zu: %s (expected %d, got %d)",
             motor_idx,
             context,
             expected,
             actual);
    TEST_ASSERT_EQUAL_MESSAGE(expected, actual, msg);
}

void assert_motor_duty(size_t motor_idx, uint32_t expected, const char * context)
{
    const uint32_t actual =
      ledc_get_duty(LEDC_LOW_SPEED_MODE, config::test::MOTOR_CHANNELS[motor_idx]);
    char msg[128];
    snprintf(msg,
             sizeof(msg),
             "Motor %zu: %s (expected %" PRIu32 ", got %" PRIu32 ")",
             motor_idx,
             context,
             expected,
             actual);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(expected, actual, msg);
}

// Test cases
void test_set_duties_cw()
{
    configure_driver();
    const std::array<uint32_t, 4> duties = {100, 200, 300, 400};
    const std::array<uint8_t, 4> dirs = {config::driver::MOTOR_CW,
                                         config::driver::MOTOR_CW,
                                         config::driver::MOTOR_CW,
                                         config::driver::MOTOR_CW};
    BL48250::get_instance().set_duties(duties, dirs);

    DELAY_MS(20);

    for (size_t i = 0; i < 4; ++i) assert_motor_direction(i, config::driver::MOTOR_CW, "cw");
    for (size_t i = 0; i < 4; ++i) assert_motor_duty(i, duties[i], "cw duty mismatch");

    BL48250::get_instance().deinit();
}

void test_set_duties_ccw()
{
    configure_driver();
    const std::array<uint32_t, 4> duties = {100, 200, 300, 400};
    const std::array<uint8_t, 4> dirs = {config::driver::MOTOR_CCW,
                                         config::driver::MOTOR_CCW,
                                         config::driver::MOTOR_CCW,
                                         config::driver::MOTOR_CCW};
    BL48250::get_instance().set_duties(duties, dirs);

    DELAY_MS(20);

    for (size_t i = 0; i < 4; ++i) assert_motor_direction(i, config::driver::MOTOR_CCW, "ccw");
    for (size_t i = 0; i < 4; ++i) assert_motor_duty(i, duties[i], "ccw duty mismatch");

    BL48250::get_instance().deinit();
}

void test_set_duties_mixed()
{
    configure_driver();
    const std::array<uint32_t, 4> duties = {500, 0, 1023, 256};
    const std::array<uint8_t, 4> dirs = {config::driver::MOTOR_CW,
                                         config::driver::MOTOR_CW,
                                         config::driver::MOTOR_CCW,
                                         config::driver::MOTOR_CW};
    BL48250::get_instance().set_duties(duties, dirs);

    DELAY_MS(20);

    assert_motor_direction(0, config::driver::MOTOR_CW, "mixed 0");
    assert_motor_direction(1, config::driver::MOTOR_CW, "mixed 1");
    assert_motor_direction(2, config::driver::MOTOR_CCW, "mixed 2");
    assert_motor_direction(3, config::driver::MOTOR_CW, "mixed 3");

    assert_motor_duty(0, 500, "mixed duty 0");
    assert_motor_duty(1, 0, "mixed duty 1");
    assert_motor_duty(2, 1023, "mixed duty 2");
    assert_motor_duty(3, 256, "mixed duty 3");

    BL48250::get_instance().deinit();
}

void test_set_duties_zero()
{
    configure_driver();
    BL48250::get_instance().set_duties({0, 0, 0, 0},
                                       {config::driver::MOTOR_CW,
                                        config::driver::MOTOR_CW,
                                        config::driver::MOTOR_CW,
                                        config::driver::MOTOR_CW});

    DELAY_MS(20);

    for (size_t i = 0; i < 4; ++i)
    {
        assert_motor_direction(i, config::driver::MOTOR_CW, "zero duty");
        assert_motor_duty(i, 0, "zero duty mismatch");
    }

    BL48250::get_instance().deinit();
}

void test_destructor_reset_outputs()
{
    {
        configure_driver();
        BL48250::get_instance().set_duties({100, 200, 300, 350},
                                           {config::driver::MOTOR_CW,
                                            config::driver::MOTOR_CW,
                                            config::driver::MOTOR_CW,
                                            config::driver::MOTOR_CW});
        DELAY_MS(20);
        BL48250::get_instance().deinit();
    }  // Driver deinitialized here

    DELAY_MS(20);

    for (size_t i = 0; i < 4; ++i) assert_motor_duty(i, 0, "post-destructor duty");
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
