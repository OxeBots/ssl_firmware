#include <74HC595.h>
#include <CD4052B.h>
#include <SPIdev.h>
#include <driver/spi_master.h>
#include <sdkconfig.h>
#include <unity.h>

#include <array>

#include "../shared_test_bus.h"

static HC595 s_expander;
static CD4052B s_mux;

void setup_mux()
{
    shared_test_spi_bus();

    HC595::Config expander_config = {.latch_gpio = (gpio_num_t)CONFIG_SHIFT_REGISTER_LATCH_GPIO,
                                     .device_key = CONFIG_SHIFT_REGISTER_DEVICE_KEY,
                                     .clock_hz = 1000000};
    TEST_ASSERT_EQUAL(ESP_OK, s_expander.init(expander_config));

    CD4052B::Config mux_config = {.select_a_bit = CONFIG_SHIFT_REGISTER_MUX_A_BIT,
                                  .select_b_bit = CONFIG_SHIFT_REGISTER_MUX_B_BIT,
                                  .inhibit_bit = CONFIG_SHIFT_REGISTER_MUX_INH_BIT};
    TEST_ASSERT_EQUAL(ESP_OK, s_mux.init(&s_expander, mux_config));
}

void test_mux_starts_disabled()
{
    TEST_ASSERT_FALSE(s_mux.is_enabled());
    TEST_ASSERT_EQUAL_UINT8(1U << CONFIG_SHIFT_REGISTER_MUX_INH_BIT,
                            s_expander.get_output() & (1U << CONFIG_SHIFT_REGISTER_MUX_INH_BIT));
}

void test_selects_all_four_channels()
{
    constexpr std::array<uint8_t, 4> expected_select = {
      0,
      static_cast<uint8_t>(1U << CONFIG_SHIFT_REGISTER_MUX_A_BIT),
      static_cast<uint8_t>(1U << CONFIG_SHIFT_REGISTER_MUX_B_BIT),
      static_cast<uint8_t>((1U << CONFIG_SHIFT_REGISTER_MUX_A_BIT) |
                           (1U << CONFIG_SHIFT_REGISTER_MUX_B_BIT))};
    const uint8_t select_mask = static_cast<uint8_t>((1U << CONFIG_SHIFT_REGISTER_MUX_A_BIT) |
                                                     (1U << CONFIG_SHIFT_REGISTER_MUX_B_BIT) |
                                                     (1U << CONFIG_SHIFT_REGISTER_MUX_INH_BIT));

    for (uint8_t channel = 0; channel < 4; ++channel)
    {
        TEST_ASSERT_EQUAL(ESP_OK, s_mux.select(channel));
        TEST_ASSERT_TRUE(s_mux.is_enabled());
        TEST_ASSERT_EQUAL_UINT8(channel, s_mux.selected_channel());
        TEST_ASSERT_EQUAL_UINT8(expected_select[channel], s_expander.get_output() & select_mask);
    }
}

void test_disable_and_reject_invalid_channel()
{
    TEST_ASSERT_EQUAL(ESP_OK, s_mux.disable());
    TEST_ASSERT_FALSE(s_mux.is_enabled());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, s_mux.select(4));
}

extern "C" void app_main(void)
{
    setup_mux();

    UNITY_BEGIN();
    RUN_TEST(test_mux_starts_disabled);
    RUN_TEST(test_selects_all_four_channels);
    RUN_TEST(test_disable_and_reject_invalid_channel);
    UNITY_END();
}
