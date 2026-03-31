#include <driver/i2c_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <unity.h>

#include "I2Cdev.h"
#include "IMUGY85.h"

// IMU Instance
IMUGY85 imu;
const char * TAG = "IMUGY85_TEST";
i2c_master_bus_handle_t bus_handle;

void tearDown(void)
{
    // Clean up after each test if needed
}

void setUp(void)
{
    // Run before each test
}

void setup_i2c()
{
    // Configuration for ESP32 I2C Master
    i2c_master_bus_config_t i2c_mst_config = {.i2c_port = (i2c_port_t)CONFIG_I2C_PORT_NUM,
                                              .sda_io_num = (gpio_num_t)CONFIG_SDA_GPIO,
                                              .scl_io_num = (gpio_num_t)CONFIG_SCL_GPIO,
                                              .clk_source = I2C_CLK_SRC_DEFAULT,
                                              .glitch_ignore_cnt = 7,
                                              .intr_priority = 0,
                                              .trans_queue_depth = 0,
                                              .flags = {.enable_internal_pullup = 1, .allow_pd = 0}};

    // Initialize the master bus
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    // Pass the handle to I2Cdev to enable caching/transactions
    I2Cdev::init(bus_handle);
}

// ============================================================================
// TESTS
// ============================================================================

void test_initialization(void)
{
    ESP_LOGI(TAG, "Initializing IMU sensors and Fusion AHRS...");

    // This calls initialize() on ADXL345, ITG3200, QMC5883L
    // And initializes FusionAhrs and FusionOffset structures
    imu.init();

    // Allow hardware to settle and initial readings to stabilize
    vTaskDelay(200 / portTICK_PERIOD_MS);
}

void test_sensor_fusion(void)
{
    ESP_LOGI(TAG, "Testing Fusion AHRS Output...");

    // Run the update loop for a short duration to let the Fusion algorithm converge.
    // The Fusion AHRS generally converges faster than raw Madgwick, but we still
    // need a few iterations to populate the buffers and calculate the first valid Quaternion.

    // IMU_SAMPLE_RATE is defined as 100Hz in IMUGY85.h
    const int iterations = 200;  // Run for 2 seconds

    for (int i = 0; i < iterations; i++)
    {
        imu.update();

        // Run loop at approx 100Hz (10ms delay) to match IMU_SAMPLE_RATE
        // This is critical for the Fusion algorithm's dt calculation
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // Retrieve computed orientation
    double roll = imu.getRoll();
    double pitch = imu.getPitch();
    double yaw = imu.getYaw();

    ESP_LOGI(TAG, "Fusion Result -> Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll, pitch, yaw);

    // Sanity Checks

    // 1. Check for NaN (Not a Number) which indicates math errors (div by zero in normalization)
    TEST_ASSERT_FALSE_MESSAGE(isnan(roll), "Roll is NaN - Fusion Algorithm failed");
    TEST_ASSERT_FALSE_MESSAGE(isnan(pitch), "Pitch is NaN - Fusion Algorithm failed");
    TEST_ASSERT_FALSE_MESSAGE(isnan(yaw), "Yaw is NaN - Fusion Algorithm failed");

    // 2. Check for all zeros
    // While 0,0,0 is physically possible, it is extremely unlikely with sensor noise.
    // If exact 0.0 is returned, it usually means the update loop isn't processing data.
    bool all_zeros = (roll == 0.0 && pitch == 0.0 && yaw == 0.0);
    if (all_zeros)
    {
        ESP_LOGW(TAG, "Warning: All angles are exactly 0.0. Check if sensors are reading data or if I2C failed.");
    }

    // 3. Basic Range Checks (Assumes board is roughly flat on desk)
    // Roll and Pitch should be within +/- 180 (Fusion output)
    TEST_ASSERT_TRUE_MESSAGE(roll >= -180.0 && roll <= 180.0, "Roll out of range");
    TEST_ASSERT_TRUE_MESSAGE(pitch >= -180.0 && pitch <= 180.0, "Pitch out of range");
    // Yaw is normalized to 0-360 in our getYaw() wrapper
    TEST_ASSERT_TRUE_MESSAGE(yaw >= -180.0 && yaw <= 180.0, "Yaw out of range");
}

void test_raw_data_access(void)
{
    ESP_LOGI(TAG, "Testing Raw Data Access...");

    // Call update once to refresh internal buffers
    imu.update();

    double ax, ay, az;
    double gx, gy, gz;
    double mx, my, mz;

    imu.getAcceleration(&ax, &ay, &az);
    imu.getGyro(&gx, &gy, &gz);
    imu.getMagnetometer(&mx, &my, &mz);
    int azimuth = imu.mag.getAzimuth();

    char direction[4];
    imu.mag.getDirection(direction, azimuth);

    ESP_LOGI(TAG, "Accel (g): X=%.2f Y=%.2f Z=%.2f", ax, ay, az);
    ESP_LOGI(TAG, "Gyro (deg/s): X=%.2f Y=%.2f Z=%.2f", gx, gy, gz);
    ESP_LOGI(TAG, "Mag (mG): X=%.2f Y=%.2f Z=%.2f", mx, my, mz);
    ESP_LOGI(TAG, "Mag Azimuth: %d, Direction: %s", azimuth, direction);

    // Sanity Check: Z-gravity should be present
    // We check magnitude to be safe against orientation changes
    double accel_magnitude = sqrt(ax * ax + ay * ay + az * az);
    ESP_LOGI(TAG, "Accel Magnitude: %.2f g", accel_magnitude);

    // Expect approx 1g vector (allowing for sensor noise and uncalibrated offset)
    TEST_ASSERT_FLOAT_WITHIN(0.3, 1.0, accel_magnitude);
}

extern "C" void app_main(void)
{
    setup_i2c();

    // Give hardware time to power up
    vTaskDelay(200 / portTICK_PERIOD_MS);

    UNITY_BEGIN();

    RUN_TEST(test_initialization);
    RUN_TEST(test_raw_data_access);  // Check data flow first (sanity check hardware)
    RUN_TEST(test_sensor_fusion);    // Check algorithm output

    UNITY_END();
}
