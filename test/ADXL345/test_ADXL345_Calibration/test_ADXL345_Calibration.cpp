#include <driver/i2c_master.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <unity.h>

#include "ADXL345.h"
#include "I2Cdev.h"

#define PIN_SDA 21
#define PIN_CLK 22
#define I2C_PORT_NUM I2C_NUM_0

ADXL345 accel;
const char * TAG = "CALIBRATION";
i2c_master_bus_handle_t bus_handle;

// Global flag to control the monitoring task
volatile bool is_calibrating = false;

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

// This runs in parallel with calibration to show you what's happening
void monitoring_task(void * pvParameters)
{
    int16_t x, y, z;
    ESP_LOGI("MONITOR", "Monitoring task started...");

    while (is_calibrating)
    {
        // Read raw values (calibrate() resets offsets to 0 at start, so these are raw)
        accel.get_acceleration(&x, &y, &z);

        if (x == 0 && y == 0)
            ESP_LOGI(TAG, "z calibrated.");
        if (x == 0 && z == 0)
            ESP_LOGI(TAG, "y calibrated.");
        if (y == 0 && z == 0)
            ESP_LOGI(TAG, "x calibrated.");

        ESP_LOGI("MONITOR", "Raw Input -> X: %d | Y: %d | Z: %d", x, y, z);

        // Update frequency: 5Hz (every 200ms)
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    ESP_LOGI("MONITOR", "Monitoring task stopping...");
    vTaskDelete(NULL);
}

void perform_calibration()
{
    // Initialize Sensor
    accel.init();

    if (!accel.test_connection())
    {
        ESP_LOGE(TAG, "ADXL345 Connection Failed! Check wiring.");
        return;
    }

    // Instructions
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "      ADXL345 CALIBRATION UTILITY");
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGW(TAG, "1. Rotate the sensor in ALL 6 DIRECTIONS.");
    ESP_LOGW(TAG, "   (Face X up, X down, Y up, Y down, Z up, Z down)");
    ESP_LOGW(TAG, "2. Hold steady briefly at each orientation.");
    ESP_LOGW(TAG, "3. The routine will AUTO-COMPLETE when all sides are done.");
    ESP_LOGI(TAG, "Starting in 3 seconds...");

    for (int i = 3; i > 0; i--)
    {
        ESP_LOGI(TAG, "%d...", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // Start Monitoring Task
    is_calibrating = true;
    xTaskCreate(monitoring_task, "monitor", 4096, NULL, 5, NULL);

    // Calibrate (Blocking call until data is gathered)
    ESP_LOGW(TAG, ">>> START ROTATING THE SENSOR NOW! <<<");
    accel.calibrate();
    ESP_LOGI(TAG, ">>> STOP. Calibration calculation done. <<<");

    // Stop monitoring
    is_calibrating = false;
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Retrieve Data
    float offX = accel.get_calibration_offset(0);
    float offY = accel.get_calibration_offset(1);
    float offZ = accel.get_calibration_offset(2);
    float scaleX = accel.get_calibration_scale(0);
    float scaleY = accel.get_calibration_scale(1);
    float scaleZ = accel.get_calibration_scale(2);

    accel.set_offset(offX, offY, offZ);

    // Print formatted code for copy-paste
    printf("\n\n/**************************************************/\n");
    printf("/* CALIBRATION RESULTS (Copy into your setup)   */\n");
    printf("/**************************************************/\n\n");

    printf("accel.setCalibrationOffsets(%.2ff, %.2ff, %.2ff);\n", offX, offY, offZ);
    printf("accel.setCalibrationScales(%.2ff, %.2ff, %.2ff);\n", scaleX, scaleY, scaleZ);

    printf("\n/**************************************************/\n\n");
}

extern "C" void app_main(void)
{
    setup_i2c();
    vTaskDelay(200 / portTICK_PERIOD_MS);

    UNITY_BEGIN();

    // perform_calibration();
    accel.set_range(ADXL345::Range::RNG_16G);
    // We enable Full Resolution by default for best precision (4mg/LSB across all ranges)
    accel.set_full_resolution(true);

    accel.set_offset(7.0f / 256.0f, 3.0f / 256.0f, -200.0f / 256.0f);
    accel.set_calibration_scales(0.98f, 0.99f, 1.01f);

    // Optional: Test live data
    ESP_LOGI(TAG, "Testing live data with new calibration...");
    accel.set_measure_enabled(true);

    float scaleX = accel.get_calibration_scale(0);
    float scaleY = accel.get_calibration_scale(1);
    float scaleZ = accel.get_calibration_scale(2);

    while (true)
    {
        int16_t ax, ay, az;
        accel.get_acceleration(&ax, &ay, &az);

        ESP_LOGI(TAG, "Calibrated -> X: %d | Y: %d | Z: %d", ax, ay, az);
        ESP_LOGI(TAG,
                 "Scaled -> X: %f | Y: %f | Z: %f",
                 ax * scaleX / 256.0f,
                 ay * scaleY / 256.0f,
                 az * scaleZ / 256.0f);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    TEST_ASSERT_TRUE(true);

    UNITY_END();
}
// void ADXL345::clearCalibration()
// {
//     _cal_offset[0] = 0.0f;
//     _cal_offset[1] = 0.0f;
//     _cal_offset[2] = 0.0f;
//     _cal_scale[0] = 1.0f;
//     _cal_scale[1] = 1.0f;
//     _cal_scale[2] = 1.0f;
// }

// void ADXL345::setCalibrationOffsets(float x, float y, float z)
// {
//     _cal_offset[0] = x;
//     _cal_offset[1] = y;
//     _cal_offset[2] = z;
// }

// void ADXL345::setCalibrationScales(float x, float y, float z)
// {
//     _cal_scale[0] = x;
//     _cal_scale[1] = y;
//     _cal_scale[2] = z;
// }

// float ADXL345::getCalibrationOffset(uint8_t index)
// {
//     if (index < 3)
//         return _cal_offset[index];
//     return 0.0f;
// }

// float ADXL345::getCalibrationScale(uint8_t index)
// {
//     if (index < 3)
//         return _cal_scale[index];
//     return 1.0f;
// }

// /**
//  * Perform offset calibration. Assumes the sensor is placed flat on a level surface (Z-axis =
//  1g).
//  * Calculates offsets for X, Y, and Z and writes them to the OFS registers.
//  * @param samples Number of samples to take for averaging (default 100)
//  */

// void ADXL345::calibrate()
// {
//     // 1. Reset SW calibration to ensure we read raw data
//     clearCalibration();

//     // 2. Reset HW offset registers to 0 to ensure pure raw data
//     setOffsetX(0);
//     setOffsetY(0);
//     setOffsetZ(0);

//     // Initial min/max values
//     int16_t x, y, z;
//     int16_t minX = 32000, maxX = -32000;
//     int16_t minY = 32000, maxY = -32000;
//     int16_t minZ = 32000, maxZ = -32000;

//     // Threshold for axis isolation (approx 0.6g)
//     // We only record a min/max for an axis if the OTHER two axes are below this value.
//     const int16_t ISO_THRESHOLD = 150;

//     // Completion Threshold:
//     // 1g is ~256 LSB. A full rotation (+1g to -1g) is ~512 LSB.
//     // We require a span of at least 400 LSB (approx 1.5g) on an axis to consider it
//     "calibrated". const int16_t SPAN_THRESHOLD = 500;

//     bool is_calibrating = true;

//     while (is_calibrating)
//     {
//         getAcceleration(&x, &y, &z);

//         // Update X only if Y and Z are close to 0
//         if (y == 0 && z == 0)
//         {
//             if (x < minX)
//                 minX = x;
//             if (x > maxX)
//                 maxX = x;
//         }

//         // Update Y only if X and Z are close to 0
//         if (x == 0 && z == 0)
//         {
//             if (y < minY)
//                 minY = y;
//             if (y > maxY)
//                 maxY = y;
//         }

//         // Update Z only if X and Y are close to 0
//         if (x == 0 && y == 0)
//         {
//             if (z < minZ)
//                 minZ = z;
//             if (z > maxZ)
//                 maxZ = z;
//         }

//         // Check if we have gathered enough data for all axes
//         bool xReady = (maxX - minX) > SPAN_THRESHOLD;
//         bool yReady = (maxY - minY) > SPAN_THRESHOLD;
//         bool zReady = (maxZ - minZ) > SPAN_THRESHOLD;

//         if (xReady && yReady && zReady)
//         {
//             is_calibrating = false;
//         }

//         vTaskDelay(10 / portTICK_PERIOD_MS);
//     }

//     // 3. Calculate Offsets: (Max + Min) / 2
//     float offX = (maxX + minX) / (256.0f *2.0f);
//     float offY = (maxY + minY) / (256.0f *2.0f);
//     float offZ = (maxZ + minZ) / (256.0f *2.0f);

//     setCalibrationOffsets(offX, offY, offZ);

//     // 4. Calculate Scales: Target / ((Max - Min) / 2)
//     // Target is 256.0f (Standard 1g LSB for ADXL345)
//     float target = 256.0f;

//     float semiRangeX = (maxX - minX) / 2.0f;
//     float semiRangeY = (maxY - minY) / 2.0f;
//     float semiRangeZ = (maxZ - minZ) / 2.0f;

//     if (semiRangeX > 0)
//         _cal_scale[0] = target / semiRangeX;
//     if (semiRangeY > 0)
//         _cal_scale[1] = target / semiRangeY;
//     if (semiRangeZ > 0)
//         _cal_scale[2] = target / semiRangeZ;
// }
