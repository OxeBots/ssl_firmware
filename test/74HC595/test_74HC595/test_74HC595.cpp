#include <driver/gpio.h>
#include <sdkconfig.h>
#include <unity.h>

#include "../shared_test_shift_register.h"
#include "74HC595.h"

static HC595 * s_expander = nullptr;

void setup_expander()
{
    // Shared with the other hardware suites: the latch is one SPI device, so
    // every suite that writes the output byte uses the same instance.
    s_expander = &shared_test_shift_register();
    TEST_ASSERT_EQUAL(ESP_OK, s_expander->write(0));
}

void test_initial_output_is_zero()
{
    TEST_ASSERT_TRUE(s_expander->is_initialized());
    TEST_ASSERT_EQUAL_UINT8(0, s_expander->get_output());
    // The latch is the device chip select, driven by the SPI peripheral:
    // it idles high and its rising edge after each transaction latches.
    TEST_ASSERT_EQUAL(1, gpio_get_level((gpio_num_t)CONFIG_SHIFT_REGISTER_LATCH_GPIO));
}

void test_write_updates_latched_output()
{
    TEST_ASSERT_EQUAL(ESP_OK, s_expander->write(0xA5));
    TEST_ASSERT_EQUAL_UINT8(0xA5, s_expander->get_output());
    TEST_ASSERT_EQUAL(1, gpio_get_level((gpio_num_t)CONFIG_SHIFT_REGISTER_LATCH_GPIO));

    TEST_ASSERT_EQUAL(ESP_OK, s_expander->write(0x5A));
    TEST_ASSERT_EQUAL_UINT8(0x5A, s_expander->get_output());
}

void test_mask_preserves_unrelated_outputs()
{
    TEST_ASSERT_EQUAL(ESP_OK, s_expander->write(0xF0));
    TEST_ASSERT_EQUAL(ESP_OK, s_expander->update_mask(0x0F, 0x05));
    TEST_ASSERT_EQUAL_UINT8(0xF5, s_expander->get_output());
    TEST_ASSERT_EQUAL(ESP_OK, s_expander->update_mask(0x30, 0x00));
    TEST_ASSERT_EQUAL_UINT8(0xC5, s_expander->get_output());
}

void test_pin_write_sets_and_clears_bit()
{
    TEST_ASSERT_EQUAL(ESP_OK, s_expander->write(0xC5));
    HC595::Pin pin = {s_expander, static_cast<uint8_t>(1U << 5)};

    HC595::Pin::write(&pin, 1);
    TEST_ASSERT_EQUAL_UINT8(0xE5, s_expander->get_output());

    HC595::Pin::write(&pin, 0);
    TEST_ASSERT_EQUAL_UINT8(0xC5, s_expander->get_output());
}

extern "C" void app_main(void)
{
    setup_expander();

    UNITY_BEGIN();
    RUN_TEST(test_initial_output_is_zero);
    RUN_TEST(test_write_updates_latched_output);
    RUN_TEST(test_mask_preserves_unrelated_outputs);
    RUN_TEST(test_pin_write_sets_and_clears_bit);
    UNITY_END();
}
