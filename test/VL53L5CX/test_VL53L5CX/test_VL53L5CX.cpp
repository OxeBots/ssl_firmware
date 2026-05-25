#include <driver/i2c_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <unity.h>

#include "I2Cdev.h"
#include "VL53L5CX.h"

VL53L5CX sensor;
const char * TAG = "VL53L5CX_TEST";
i2c_master_bus_handle_t bus_handle;

void setUp(void)
{
}

void tearDown(void)
{
}

// Creates the I2C bus and initialises I2Cdev + VL53L5CX.
void setup_i2c()
{
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = CONFIG_I2C_PORT_NUM,
        .sda_io_num = (gpio_num_t)CONFIG_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {.enable_internal_pullup = true, .allow_pd = false},
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    I2Cdev::init(bus_handle);
}

// Verifies the sensor responds with the correct device/revision IDs.
void test_connection(void)
{
    ESP_LOGI(TAG, "Testing connection...");
    TEST_ASSERT_TRUE_MESSAGE(sensor.test_connection(),
                             "VL53L5CX not detected (check wiring)");
}

// Starts ranging, waits for one measurement, prints all 4x4 zones.
void test_single_range_sample(void)
{
    uint8_t status = sensor.start_ranging();
    TEST_ASSERT_EQUAL_MESSAGE(VL53L5CX_STATUS_OK, status, "start_ranging failed");

    vTaskDelay(50 / portTICK_PERIOD_MS);

    VL53L5CX_ResultsData results;
    uint8_t is_ready = 0;
    int retries = 20;

    // Poll until data ready or timeout
    while (retries > 0)
    {
        status = sensor.check_data_ready(&is_ready);
        TEST_ASSERT_EQUAL_MESSAGE(VL53L5CX_STATUS_OK, status,
                                  "check_data_ready failed");

        if (is_ready) break;
        vTaskDelay(10 / portTICK_PERIOD_MS);
        retries--;
    }

    TEST_ASSERT_TRUE_MESSAGE(is_ready, "Timed out waiting for ranging data");

    status = sensor.get_ranging_data(&results);
    TEST_ASSERT_EQUAL_MESSAGE(VL53L5CX_STATUS_OK, status,
                              "get_ranging_data failed");

    // Print zone distances for visual inspection
    for (int i = 0; i < 16; i++)
    {
        printf("Zone: %3d  |  Status: %3u  |  Distance: %4d mm\n",
               i,
               results.target_status[VL53L5CX_NB_TARGET_PER_ZONE * i],
               results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE * i]);
    }

    sensor.stop_ranging();
}

// Collects 5 range samples in a loop, logs each one.
void test_multi_range_cycle(void)
{
    uint8_t status = sensor.start_ranging();
    TEST_ASSERT_EQUAL_MESSAGE(VL53L5CX_STATUS_OK, status, "start_ranging failed");

    VL53L5CX_ResultsData results;
    int sample_count = 0;

    while (sample_count < 5)
    {
        uint8_t is_ready = 0;
        status = sensor.check_data_ready(&is_ready);

        if (status == VL53L5CX_STATUS_OK && is_ready)
        {
            status = sensor.get_ranging_data(&results);
            TEST_ASSERT_EQUAL_MESSAGE(VL53L5CX_STATUS_OK, status,
                                      "get_ranging_data failed");

            ESP_LOGI(TAG, "Sample %d: zone 0 = %d mm",
                     sample_count, results.distance_mm[0]);
            sample_count++;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    TEST_ASSERT_EQUAL_MESSAGE(5, sample_count, "Failed to collect all samples");
    sensor.stop_ranging();
}

extern "C" void app_main(void)
{
    setup_i2c();

    esp_err_t ret = sensor.init(bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Sensor init failed: %s", esp_err_to_name(ret));
        return;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);

    UNITY_BEGIN();
    RUN_TEST(test_connection);
    RUN_TEST(test_single_range_sample);
    RUN_TEST(test_multi_range_cycle);
    UNITY_END();
}
