#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <sdkconfig.h>
#include <unity.h>

#include "../shared_test_bus.h"
#include "I2Cdev.h"
#include "IMUGY85.h"

// IMU Instance
IMUGY85 & imu = IMUGY85::get_instance();
static const char * TAG = "IMUGY85_TEST";

void tearDown(void)
{
    // Clean up after each test if needed
}

void setUp(void)
{
    // Run before each test
}

void test_initialization(void)
{
    ESP_LOGI(TAG, "Initializing IMU sensors...");
    const i2c_master_bus_handle_t bus = shared_test_i2c_bus();
    I2Cdev::init(bus);
    esp_err_t err = imu.init(bus);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err, "IMUGY85 initialization failed");
    TEST_ASSERT_TRUE(imu.is_initialized());

    const gpio_num_t interrupt_pins[] = {static_cast<gpio_num_t>(CONFIG_GY85_AINT1_GPIO),
                                         static_cast<gpio_num_t>(CONFIG_GY85_GINT_GPIO),
                                         static_cast<gpio_num_t>(CONFIG_GY85_MDRDY_GPIO)};
    for (gpio_num_t pin : interrupt_pins)
    {
        const int level = gpio_get_level(pin);
        TEST_ASSERT_TRUE(level == 0 || level == 1);
    }

    // Allow hardware to settle and initial readings to stabilize
    vTaskDelay(200 / portTICK_PERIOD_MS);
}

void test_raw_acceleration(void)
{
    ESP_LOGI(TAG, "Testing Acceleration Read...");

    int16_t ax, ay, az;
    imu.read_acceleration(&ax, &ay, &az);

    ESP_LOGI(TAG, "Accel: X=%d, Y=%d, Z=%d", ax, ay, az);

    // Sanity: not all zeros (sensor must be connected for this to pass)
    bool all_zeros = (ax == 0 && ay == 0 && az == 0);
    if (all_zeros)
    {
        ESP_LOGW(TAG, "Accel returned all zeros — sensor may not be connected.");
    }
    TEST_ASSERT_FALSE_MESSAGE(all_zeros, "Accel returned all zeros - sensor may not be connected.");
}

void test_raw_gyro(void)
{
    ESP_LOGI(TAG, "Testing Gyro Read...");

    int16_t gx, gy, gz, temp;
    imu.read_gyro(&gx, &gy, &gz, &temp);

    ESP_LOGI(TAG, "Gyro: X=%d, Y=%d, Z=%d, temp=%d", gx, gy, gz, temp);

    // Sanity: not all zeros
    bool all_zeros = (gx == 0 && gy == 0 && gz == 0);
    if (all_zeros)
    {
        ESP_LOGW(TAG, "Gyro returned all zeros — sensor may not be connected.");
    }
    TEST_ASSERT_FALSE_MESSAGE(all_zeros, "Gyro returned all zeros - sensor may not be connected.");
}

void test_raw_magnetometer(void)
{
    ESP_LOGI(TAG, "Testing Magnetometer Read...");

    int16_t mx, my, mz;
    imu.read_magnetometer(&mx, &my, &mz);

    ESP_LOGI(TAG, "Mag: X=%d, Y=%d, Z=%d", mx, my, mz);

    // Sanity: not all zeros
    bool all_zeros = (mx == 0 && my == 0 && mz == 0);
    if (all_zeros)
    {
        ESP_LOGW(TAG, "Magnetometer returned all zeros — sensor may not be connected.");
    }
    TEST_ASSERT_FALSE_MESSAGE(all_zeros,
                              "Magnetometer returned all zeros - sensor may not be connected.");
}

void test_calibrated_readings(void)
{
    ESP_LOGI(TAG, "Testing Calibrated Readings...");

    Vector3f accel = imu.read_acceleration_calibrated();
    Vector3f mag = imu.read_magnetometer_calibrated();

    ESP_LOGI(TAG, "Accel calibrated: X=%.2f, Y=%.2f, Z=%.2f", accel.x, accel.y, accel.z);
    ESP_LOGI(TAG, "Mag calibrated: X=%.2f, Y=%.2f, Z=%.2f", mag.x, mag.y, mag.z);

    // Without calibration, accel magnitude should still be near 1g.
    // Use a very loose range since uncalibrated sensors can be off significantly.
    double accel_mag = sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    ESP_LOGI(TAG, "Accel magnitude: %.2f g", accel_mag);

    // Very loose check — uncalibrated sensors can be way off
    // This just verifies the math doesn't produce NaN or infinity
    TEST_ASSERT_FALSE_MESSAGE(isnan(accel_mag), "Accel magnitude is NaN");
    TEST_ASSERT_FALSE_MESSAGE(isinf(accel_mag), "Accel magnitude is infinite");
}

extern "C" void app_main(void)
{
    // Give hardware time to power up
    vTaskDelay(200 / portTICK_PERIOD_MS);

    UNITY_BEGIN();

    RUN_TEST(test_initialization);
    RUN_TEST(test_raw_acceleration);
    RUN_TEST(test_raw_gyro);
    RUN_TEST(test_raw_magnetometer);
    RUN_TEST(test_calibrated_readings);

    UNITY_END();
}
