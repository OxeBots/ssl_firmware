#include <74HC595.h>
#include <SPIdev.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <sdkconfig.h>
#include <unity.h>

#include "../shared_test_bus.h"

static HC595 s_expander;

void setup_expander()
{
    shared_test_spi_bus();

    HC595::Config config = {.latch_gpio = (gpio_num_t)CONFIG_SHIFT_REGISTER_LATCH_GPIO,
                            .device_key = CONFIG_SHIFT_REGISTER_DEVICE_KEY,
                            .clock_hz = 1000000};
    TEST_ASSERT_EQUAL(ESP_OK, s_expander.init(config));
}

void test_initial_output_is_zero()
{
    TEST_ASSERT_TRUE(s_expander.is_initialized());
    TEST_ASSERT_EQUAL_UINT8(0, s_expander.get_output());
    TEST_ASSERT_EQUAL(0, gpio_get_level((gpio_num_t)CONFIG_SHIFT_REGISTER_LATCH_GPIO));
}

void test_write_updates_latched_output()
{
    TEST_ASSERT_EQUAL(ESP_OK, s_expander.write(0xA5));
    TEST_ASSERT_EQUAL_UINT8(0xA5, s_expander.get_output());
    TEST_ASSERT_EQUAL(0, gpio_get_level((gpio_num_t)CONFIG_SHIFT_REGISTER_LATCH_GPIO));

    TEST_ASSERT_EQUAL(ESP_OK, s_expander.write(0x5A));
    TEST_ASSERT_EQUAL_UINT8(0x5A, s_expander.get_output());
}

void test_mask_preserves_unrelated_outputs()
{
    TEST_ASSERT_EQUAL(ESP_OK, s_expander.write(0xF0));
    TEST_ASSERT_EQUAL(ESP_OK, s_expander.update_mask(0x0F, 0x05));
    TEST_ASSERT_EQUAL_UINT8(0xF5, s_expander.get_output());
    TEST_ASSERT_EQUAL(ESP_OK, s_expander.update_mask(0x30, 0x00));
    TEST_ASSERT_EQUAL_UINT8(0xC5, s_expander.get_output());
}

extern "C" void app_main(void)
{
    setup_expander();

    UNITY_BEGIN();
    RUN_TEST(test_initial_output_is_zero);
    RUN_TEST(test_write_updates_latched_output);
    RUN_TEST(test_mask_preserves_unrelated_outputs);
    UNITY_END();
}
