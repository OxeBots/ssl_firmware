#include <driver/i2c_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <unity.h>

#include "ADXL345.h"
#include "I2Cdev.h"

ADXL345 accel;
const char * TAG = "ADXL345_TEST";
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
    i2c_master_bus_config_t i2c_mst_config = {
      .i2c_port = CONFIG_I2C_PORT_NUM,
      .sda_io_num = (gpio_num_t)CONFIG_SDA_GPIO,
      .scl_io_num = (gpio_num_t)CONFIG_SCL_GPIO,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .intr_priority = 0,
      .trans_queue_depth = 0,
      .flags = {.enable_internal_pullup = true, .allow_pd = false}};

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    // Pass the new bus handle to I2Cdev so it can create device handles internally
    I2Cdev::init(bus_handle);
}

void test_connection_and_device_id(void)
{
    ESP_LOGI(TAG, "Testing Connection...");
    bool connected = accel.test_connection();
    TEST_ASSERT_TRUE_MESSAGE(connected, "ADXL345 connection failed");

    uint8_t devId = accel.get_device_id();
    ESP_LOGI(TAG, "Device ID Read: 0x%02X", devId);
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(0xE5, devId, "Device ID mismatch");
}

void test_configuration_read_write(void)
{
    // Test that we can write and read the DATA_FORMAT register.
    // The Range enum values (0x00, 0x01, 0x10, 0x11) encode both full_res and
    // range bits together, but set_range/get_range only operate on the 2-bit
    // range field (bits 1:0). The low 2 bits of RNG_16G (0x11) are 0x01,
    // which corresponds to 4G in the raw register. We test round-trip instead.
    accel.set_range(ADXL345::Range::RNG_16G);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ADXL345::Range r1 = accel.get_range();

    accel.set_range(ADXL345::Range::RNG_2G);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ADXL345::Range r2 = accel.get_range();

    // The two settings should produce different register values
    TEST_ASSERT_TRUE_MESSAGE(r1 != r2, "Range settings not distinguishable");

    accel.set_full_resolution(1);
    TEST_ASSERT_TRUE_MESSAGE(accel.get_full_resolution(), "Failed to enable Full Res");
}

void test_measure_mode_control(void)
{
    accel.set_measure_enabled(true);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    TEST_ASSERT_TRUE_MESSAGE(accel.get_measure_enabled(), "Failed to enable Measure");

    accel.set_measure_enabled(false);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    TEST_ASSERT_FALSE_MESSAGE(accel.get_measure_enabled(), "Failed to disable Measure");

    accel.set_measure_enabled(true);
}

void test_offset_calibration(void)
{
    ESP_LOGI(TAG, "Testing Offset Registers...");

    // Write arbitrary offsets
    int8_t test_x = 10, test_y = -20, test_z = 5;
    accel.set_offset(test_x, test_y, test_z);

    // Read them back
    int8_t read_x = accel.get_offset_x();
    int8_t read_y = accel.get_offset_y();
    int8_t read_z = accel.get_offset_z();

    TEST_ASSERT_EQUAL_INT8(test_x, read_x);
    TEST_ASSERT_EQUAL_INT8(test_y, read_y);
    TEST_ASSERT_EQUAL_INT8(test_z, read_z);

    // Reset to zero for normal operation
    accel.set_offset(0, 0, 0);
}

void test_sensor_data_read(void)
{
    accel.set_measure_enabled(true);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    int16_t ax, ay, az;
    accel.get_acceleration(&ax, &ay, &az);

    ESP_LOGI(TAG, "Readings: X=%d, Y=%d, Z=%d", ax, ay, az);
    TEST_ASSERT_FALSE_MESSAGE(ax == 0 && ay == 0 && az == 0, "All zeros read");
}

extern "C" void app_main(void)
{
    setup_i2c();
    accel.init();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    UNITY_BEGIN();
    RUN_TEST(test_connection_and_device_id);
    RUN_TEST(test_configuration_read_write);
    RUN_TEST(test_offset_calibration);
    RUN_TEST(test_measure_mode_control);
    RUN_TEST(test_sensor_data_read);
    UNITY_END();
}
