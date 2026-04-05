#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "I2Cdev.h"
#include "IMUGY85.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "unity.h"

// Standard ESP32 I2C Pins (Change if using a different board/wiring)
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_NUM I2C_NUM_0

static const char * TAG = "GY85_TEST";

// Instantiate the IMU driver (which owns accel, gyro, and mag objects)
IMUGY85 & imu = IMUGY85::get_instance();

// Handle for the new I2C driver
i2c_master_bus_handle_t bus_handle;

static void i2c_master_init(void)
{
    i2c_master_bus_config_t i2c_mst_config = {
      .i2c_port = I2C_MASTER_NUM,
      .sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO,
      .scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .intr_priority = 0,
      .trans_queue_depth = 0,  // Use default
      .flags =
        {
          .enable_internal_pullup = true,
          .allow_pd = false,
        },
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
}
void get_dominant_axis(int16_t x, int16_t y, int16_t z, int threshold, char * output)
{
    int16_t ax = abs(x);
    int16_t ay = abs(y);
    int16_t az = abs(z);

    if (ax < threshold && ay < threshold && az < threshold)
    {
        sprintf(output, " - ");
        return;
    }

    if (ax > ay && ax > az)
    {
        sprintf(output, "%c X", (x > 0) ? '+' : '-');
    }
    else if (ay > ax && ay > az)
    {
        sprintf(output, "%c Y", (y > 0) ? '+' : '-');
    }
    else
    {
        sprintf(output, "%c Z", (z > 0) ? '+' : '-');
    }
}

void get_dominant_range(int16_t rx, int16_t ry, int16_t rz, int threshold, char * output)
{
    if (rx < threshold && ry < threshold && rz < threshold)
    {
        sprintf(output, " - ");
        return;
    }
    // For Mag, we just identify the active axis. Sign depends on North pole direction relative to sensor.
    if (rx > ry && rx > rz)
    {
        sprintf(output, " X");
    }
    else if (ry > rx && ry > rz)
    {
        sprintf(output, " Y");
    }
    else
    {
        sprintf(output, " Z");
    }
}

void setUp(void)
{
    // Runs before every test
}

void tearDown(void)
{
    // Runs after every test
}

void test_sensor_orientation_check(void)
{
    // Initialize GY-85 Sensors
    ESP_LOGI(TAG, "Initializing Sensors...");
    imu.init();

    // Clear Calibrations for Orientation Testing
    ESP_LOGI(TAG, "Clearing default calibrations for raw data inspection...");
    imu.get_accel().set_offset(0, 0, 0);
    imu.get_accel().set_calibration_scales(1.0f, 1.0f, 1.0f);
    imu.get_gyro().set_offsets(0, 0, 0);
    imu.get_mag().clear_calibration();

    printf("\n\n");
    printf("================================================================\n");
    printf("GY-85 ORIENTATION CHECK TOOL\n");
    printf("================================================================\n");
    printf("Instructions to verify Reference Frames:\n");
    printf("1. ACCEL: Hold board FLAT. Gravity should be dominant on Z.\n");
    printf("2. GYRO:  Rotate swiftly around an axis to see it spike.\n");
    printf("3. MAG:   ROTATE 360 deg FIRST! Then point North to find axis.\n");
    printf("          (Dominant axis is determined by widest range of motion)\n");
    printf("================================================================\n\n");

    // Header
    printf("%-15s | %-15s | %-15s\n", "ACCEL (Corrected)", "GYRO (Rotation)", "MAG (Raw Range)");
    printf("%-15s | %-15s | %-15s\n", "X    Y    Z  (Dom)", "X    Y    Z  (Dom)", "X    Y    Z  (Act)");

    // Variables for Mag Range tracking (Min/Max initialization)
    int16_t mag_min[3] = {32000, 32000, 32000};
    int16_t mag_max[3] = {-32000, -32000, -32000};

    // Infinite loop for manual verification
    while (1)
    {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        int16_t mx, my, mz;

        // Read Raw Data
        imu.get_accel().get_acceleration(&ax, &ay, &az);

        // User observed that Accel X and Y are inverted relative to Gyro/Board frame.
        // We invert them here to align the frames.
        ax = -ax;
        ay = -ay;

        imu.get_gyro().get_rotation(&gx, &gy, &gz);
        imu.get_mag().get_orientation(&mx, &my, &mz);

        // --- UPDATE MAG RANGE STATS ---
        if (mx < mag_min[0])
            mag_min[0] = mx;
        if (mx > mag_max[0])
            mag_max[0] = mx;
        if (my < mag_min[1])
            mag_min[1] = my;
        if (my > mag_max[1])
            mag_max[1] = my;
        if (mz < mag_min[2])
            mag_min[2] = mz;
        if (mz > mag_max[2])
            mag_max[2] = mz;

        int16_t range_x = mag_max[0] - mag_min[0];
        int16_t range_y = mag_max[1] - mag_min[1];
        int16_t range_z = mag_max[2] - mag_min[2];

        // Detect Dominant Axes
        char dom_a[5], dom_g[5], dom_m[5];

        // Accel Threshold: ~0.5G (approx 128 LSB)
        get_dominant_axis(ax, ay, az, 128, dom_a);

        // Gyro Threshold: ~50 deg/s (approx 700 LSB) to ignore drift/noise
        get_dominant_axis(gx, gy, gz, 700, dom_g);

        // Mag Threshold: ~200 LSB Range to consider it active
        get_dominant_range(range_x, range_y, range_z, 200, dom_m);

        // Print Formatted Table
        // Note: For Mag, we print the raw value, but the Dom column is based on RANGE
        printf("%4d %4d %4d (%s) | %4d %4d %4d (%s) | %4d %4d %4d (%s)\n",  //
               ax, ay, az, dom_a,                                           //
               gx, gy, gz, dom_g,                                           //
               mx, my, mz, dom_m);                                          //

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

extern "C" void app_main(void)
{
    // Initialize I2C Bus (New Driver)
    ESP_LOGI(TAG, "Initializing I2C (New Driver)...");
    i2c_master_init();

    // Pass the bus handle to the I2Cdev wrapper class
    I2Cdev::init(bus_handle);

    // Unity Test Framework Start
    UNITY_BEGIN();

    // Run the manual verification loop as a test case
    RUN_TEST(test_sensor_orientation_check);

    // Unity Test Framework End
    UNITY_END();
}
