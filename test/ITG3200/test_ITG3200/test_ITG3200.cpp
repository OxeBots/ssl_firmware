#include <driver/i2c_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <unity.h>

#include "I2Cdev.h"
#include "ITG3200.h"

// Pin Definitions
#define PIN_SDA 21
#define PIN_CLK 22
#define I2C_PORT_NUM I2C_NUM_0

ITG3200 gyro;
const char * TAG = "ITG3200_TEST";
i2c_master_bus_handle_t bus_handle;

void setup_i2c()
{
    i2c_master_bus_config_t i2c_mst_config = {.i2c_port = I2C_PORT_NUM,
                                              .sda_io_num = (gpio_num_t)PIN_SDA,
                                              .scl_io_num = (gpio_num_t)PIN_CLK,
                                              .clk_source = I2C_CLK_SRC_DEFAULT,
                                              .glitch_ignore_cnt = 7,
                                              .intr_priority = 0,
                                              .trans_queue_depth = 0,
                                              .flags = {.enable_internal_pullup = true, .allow_pd = false}};

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    I2Cdev::init(bus_handle);
}

void test_connection_and_device_id(void)
{
    ESP_LOGI(TAG, "Testing Connection...");

    // Test 1: Helper function
    bool connected = gyro.test_connection();
    TEST_ASSERT_TRUE_MESSAGE(connected, "ITG3200 connection failed (Check wiring or address 0x68/0x69)");

    // Test 2: Verify Device ID explicitly
    // Default ID is 0b110100 (0x34 or 52 decimal)
    uint8_t devId = gyro.get_device_id();
    ESP_LOGI(TAG, "Device ID Read: 0x%02X", devId);
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(0x34, devId, "Device ID mismatch! Expected 0x34");
}

void test_configuration_read_write(void)
{
    ESP_LOGI(TAG, "Testing Configuration...");

    // Test 1: Set Sample Rate Divider
    // F_sample = F_internal / (divider + 1)
    gyro.set_rate(7);  // divider = 7
    vTaskDelay(10 / portTICK_PERIOD_MS);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(7, gyro.get_rate(), "Failed to set Sample Rate Divider");

    // Test 2: Set DLPF (Digital Low Pass Filter)
    gyro.set_dlpf_bandwidth(ITG3200::Bandwidth::BW_42);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    TEST_ASSERT_EQUAL_MESSAGE((int)ITG3200::Bandwidth::BW_42, (int)gyro.get_dlpf_bandwidth(),
                              "Failed to set DLPF Bandwidth");

    // Test 3: Clock Source
    gyro.set_clock_source(ITG3200::ClockSource::PLL_XGYRO);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    TEST_ASSERT_EQUAL_MESSAGE((int)ITG3200::ClockSource::PLL_XGYRO, (int)gyro.get_clock_source(),
                              "Failed to set Clock Source");
}

void test_sensor_data_read(void)
{
    ESP_LOGI(TAG, "Testing Data Reading...");

    // Initialize (Sets FullScale 2000, Clock PLL X)
    gyro.init();

    // Allow sensor to settle and gather samples
    vTaskDelay(100 / portTICK_PERIOD_MS);

    int16_t x, y, z;
    gyro.get_rotation(&x, &y, &z);

    // ITG3200 also has a temperature sensor
    int16_t raw_temp = gyro.get_temperature();
    // Offset -13200, Scale 280 LSB/C, + 35C offset (Approx calculation from datasheet)
    float temp_c = 35.0 + ((raw_temp + 13200) / 280.0);

    ESP_LOGI(TAG, "Gyro: X=%d, Y=%d, Z=%d | Temp Raw: %d (approx %.2f C)", x, y, z, raw_temp, temp_c);

    // Sanity Checks
    bool all_zeros = (x == 0 && y == 0 && z == 0);
    TEST_ASSERT_FALSE_MESSAGE(all_zeros, "Sensor returned all zeros! Data read failed.");

    // Temperature sanity check (should be roughly room temp, not 0 or -MAX)
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, raw_temp, "Temperature read zero (unlikely)");
}

extern "C" void app_main(void)
{
    setup_i2c();

    // Give hardware time to power up
    vTaskDelay(200 / portTICK_PERIOD_MS);

    UNITY_BEGIN();

    RUN_TEST(test_connection_and_device_id);
    RUN_TEST(test_configuration_read_write);
    RUN_TEST(test_sensor_data_read);

    UNITY_END();
}
