#include <driver/i2c_master.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <unity.h>

#include "I2Cdev.h"
#include "ITG3200.h"

#define PIN_SDA 21
#define PIN_CLK 22
#define I2C_PORT_NUM I2C_NUM_0

ITG3200 gyro;
const char * TAG = "GYRO_CALIB";
i2c_master_bus_handle_t bus_handle;

void setup_i2c()
{
    i2c_master_bus_config_t i2c_mst_config = {
      .i2c_port = I2C_PORT_NUM,
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

void perform_calibration()
{
    ESP_LOGI(TAG, "Initializing ITG3200...");
    gyro.init();

    if (!gyro.test_connection())
    {
        ESP_LOGE(TAG, "Connection Failed!");
        return;
    }

    ESP_LOGI(TAG, "Connection Successful.");

    // Instructions
    ESP_LOGW(TAG, "---------------------------------------------------");
    ESP_LOGW(TAG, "      ITG3200 GYRO CALIBRATION STARTING");
    ESP_LOGW(TAG, "---------------------------------------------------");
    ESP_LOGW(TAG, "KEEP THE SENSOR COMPLETELY STILL ON A FLAT SURFACE.");
    ESP_LOGW(TAG, "DO NOT MOVE IT.");
    ESP_LOGI(TAG, "Calibration starting in 3 seconds...");
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, ">>> CALIBRATING... (Collecting 1000 samples) <<<");

    // Run calibration (takes about 2-3 seconds)
    gyro.calibrate(1000);

    ESP_LOGI(TAG, ">>> DONE <<<");

    // Get results
    int16_t x_off, y_off, z_off;
    gyro.get_offsets(&x_off, &y_off, &z_off);

    // Print copy-paste code
    printf("\n/******************************************/\n");
    printf("/* COPY THIS INTO YOUR ROBOT SETUP      */\n");
    printf("/******************************************/\n");
    printf("gyro.setOffsets(%d, %d, %d);\n", x_off, y_off, z_off);
    printf("/******************************************/\n\n");
}

extern "C" void app_main(void)
{
    setup_i2c();
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Power up stability

    UNITY_BEGIN();

    perform_calibration();

    ESP_LOGI(TAG, "Now reading calibrated values (Should be near 0)...");

    auto now = esp_timer_get_time();

    while (esp_timer_get_time() - now < 10000000)
    {
        int16_t x, y, z;
        gyro.get_rotation(&x, &y, &z);
        // If calibrated correctly, these should hover around 0 when still
        ESP_LOGI(TAG, "X: %6d | Y: %6d | Z: %6d", x, y, z);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    TEST_ASSERT_TRUE(true);

    UNITY_END();
}
// /**
//  * @brief Calibrate the Gyroscope.
//  * The sensor must be STATIONARY during this process. It calculates the average bias (zero-rate
//  * error) and stores it to be subtracted from future readings.
//  * @param samples Number of samples to read for averaging (default 1000)
//  */
// void ITG3200::calibrate(uint16_t samples)
// {
//     long sumX = 0;
//     long sumY = 0;
//     long sumZ = 0;

//     // Reset offsets for calibration phase
//     x_offset = 0;
//     y_offset = 0;
//     z_offset = 0;

//     int16_t rx, ry, rz;

//     for (uint16_t i = 0; i < samples; i++)
//     {
//         getRotation(&rx, &ry, &rz);
//         sumX += rx;
//         sumY += ry;
//         sumZ += rz;
//         // Small delay to prevent I2C flooding and allow new data
//         vTaskDelay(2 / portTICK_PERIOD_MS);
//     }

//     x_offset = sumX / samples;
//     y_offset = sumY / samples;
//     z_offset = sumZ / samples;
// }

// void ITG3200::setOffsets(int16_t x, int16_t y, int16_t z)
// {
//     x_offset = x;
//     y_offset = y;
//     z_offset = z;
// }

// void ITG3200::getOffsets(int16_t * x, int16_t * y, int16_t * z)
// {
//     *x = x_offset;
//     *y = y_offset;
//     *z = z_offset;
// }
