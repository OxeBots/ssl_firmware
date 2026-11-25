#include <driver/i2c_master.h>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdkconfig.h>
#include <unity.h>

#include "../shared_test_bus.h"
#include "../shared_test_spi.h"
#include "74HC595.h"
#include "AS5600.h"
#include "CD4052B.h"
#include "I2Cdev.h"
#include "SPIdev.h"

// The AS5600 shares the I2C bus only through the CD4052B analog switch:
// one encoder at address 0x36 is exposed at a time, so every test that
// talks I2C must first select the mux channel of a connected wheel.
static HC595 s_shift_register;
static CD4052B s_mux;
static AS5600 s_encoder(ADC_CHANNEL_0, nullptr, false);
static uint8_t s_connected_channel = 0;

static const AS5600::OutputStage CONFIG_OUTPUT_STAGE = AS5600::OutputStage::ANALOG_REDUCED;
static const AS5600::SlowFilter CONFIG_SLOW_FILTER = AS5600::SlowFilter::FILTER_2X;
static const AS5600::FastFilter CONFIG_FAST_FILTER = AS5600::FastFilter::THRESH_6LSB;

void setUp(void)
{
    static bool initialized = false;
    if (initialized)
        return;

    // SPI for the 74HC595 direction and encoder selector outputs
    shared_test_spi_bus();

    HC595::Config expander_config = {.latch_gpio = (gpio_num_t)CONFIG_SHIFT_REGISTER_LATCH_GPIO,
                                     .device_key = CONFIG_SHIFT_REGISTER_DEVICE_KEY,
                                     .clock_hz = 1000000};
    TEST_ASSERT_EQUAL(ESP_OK, s_shift_register.init(expander_config));

    CD4052B::Config mux_config = {.select_a_bit = CONFIG_SHIFT_REGISTER_MUX_A_BIT,
                                  .select_b_bit = CONFIG_SHIFT_REGISTER_MUX_B_BIT,
                                  .inhibit_bit = CONFIG_SHIFT_REGISTER_MUX_INH_BIT};
    TEST_ASSERT_EQUAL(ESP_OK, s_mux.init(&s_shift_register, mux_config));

    // I2C master bus; the AS5600 answers at 0x36 only while a mux channel is selected
    I2Cdev::init(shared_test_i2c_bus());

    initialized = true;
}

static void select_encoder_channel()
{
    TEST_ASSERT_EQUAL(ESP_OK, s_mux.select(s_connected_channel));
    vTaskDelay(pdMS_TO_TICKS(10));
}

void test_i2c_requires_mux_channel_selected()
{
    TEST_ASSERT_EQUAL(ESP_OK, s_mux.disable());
    TEST_ASSERT_EQUAL(ESP_FAIL, s_encoder.init_i2c());
    TEST_ASSERT_FALSE(s_encoder.is_i2c_initialized());

    // Scan every wheel: like during calibration, a single AS5600 is routed
    // through the mux and answers at 0x36 on whichever channel is selected.
    bool connected = false;
    for (uint8_t channel = 0; channel < 4 && !connected; ++channel)
    {
        if (s_mux.select(channel) != ESP_OK)
            continue;
        vTaskDelay(pdMS_TO_TICKS(10));
        connected = (s_encoder.init_i2c() == ESP_OK);
        if (connected)
            s_connected_channel = channel;
    }
    TEST_ASSERT_TRUE_MESSAGE(connected, "No AS5600 answered through CD4052B channels 0-3");
    TEST_ASSERT_TRUE(s_encoder.is_i2c_initialized());

    // Second initialization of the same device must be rejected.
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, s_encoder.init_i2c());
}

void test_i2c_raw_angle_is_12_bit()
{
    select_encoder_channel();

    uint16_t angle = 0xFFFF;
    TEST_ASSERT_EQUAL(ESP_OK, s_encoder.get_i2c_raw_angle(&angle));
    TEST_ASSERT_LESS_OR_EQUAL_UINT16(0x0FFF, angle);
}

void test_configuration_register_roundtrip()
{
    select_encoder_channel();

    TEST_ASSERT_EQUAL(ESP_OK, s_encoder.set_output_stage(CONFIG_OUTPUT_STAGE));
    TEST_ASSERT_EQUAL(ESP_OK, s_encoder.set_slow_filter(CONFIG_SLOW_FILTER));
    TEST_ASSERT_EQUAL(ESP_OK, s_encoder.set_fast_filter(CONFIG_FAST_FILTER));

    // Written values match the production AS5600Settings defaults, so the
    // sensor is left in its normal operating configuration.
    AS5600::OutputStage stage;
    AS5600::SlowFilter slow;
    AS5600::FastFilter fast;
    TEST_ASSERT_EQUAL(ESP_OK, s_encoder.read_configuration(&stage, &slow, &fast));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CONFIG_OUTPUT_STAGE), static_cast<uint8_t>(stage));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CONFIG_SLOW_FILTER), static_cast<uint8_t>(slow));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CONFIG_FAST_FILTER), static_cast<uint8_t>(fast));
}

void test_mux_disable_gates_i2c_access()
{
    select_encoder_channel();
    uint16_t angle = 0;
    TEST_ASSERT_EQUAL(ESP_OK, s_encoder.get_i2c_raw_angle(&angle));

    TEST_ASSERT_EQUAL(ESP_OK, s_mux.disable());
    TEST_ASSERT_EQUAL(ESP_FAIL, s_encoder.get_i2c_raw_angle(&angle));

    select_encoder_channel();
    TEST_ASSERT_EQUAL(ESP_OK, s_encoder.get_i2c_raw_angle(&angle));
}

extern "C" void app_main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_i2c_requires_mux_channel_selected);
    RUN_TEST(test_i2c_raw_angle_is_12_bit);
    RUN_TEST(test_configuration_register_roundtrip);
    RUN_TEST(test_mux_disable_gates_i2c_access);

    UNITY_END();
}
