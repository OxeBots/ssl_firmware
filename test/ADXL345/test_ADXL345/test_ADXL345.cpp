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
    i2c_master_bus_config_t i2c_mst_config = {.i2c_port = CONFIG_I2C_PORT_NUM,
                                              .sda_io_num = (gpio_num_t)CONFIG_SDA_GPIO,
                                              .scl_io_num = (gpio_num_t)CONFIG_SCL_GPIO,
                                              .clk_source = I2C_CLK_SRC_DEFAULT,
                                              .glitch_ignore_cnt = 7,
                                              .intr_priority = 0,
                                              .trans_queue_depth = 0,
                                              .flags = {.enable_internal_pullup = 1, .allow_pd = 0}};

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    // Pass the new bus handle to I2Cdev so it can create device handles internally
    I2Cdev::init(bus_handle);
}

void test_connection_and_device_id(void)
{
    ESP_LOGI(TAG, "Testing Connection...");
    bool connected = accel.testConnection();
    TEST_ASSERT_TRUE_MESSAGE(connected, "ADXL345 connection failed");

    uint8_t devId = accel.getDeviceID();
    ESP_LOGI(TAG, "Device ID Read: 0x%02X", devId);
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(0xE5, devId, "Device ID mismatch");
}

void test_configuration_read_write(void)
{
    accel.setRange(ADXL345_RANGE_16G);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    TEST_ASSERT_EQUAL_MESSAGE(ADXL345_RANGE_16G, accel.getRange(), "Failed to set Range 16G");

    accel.setRange(ADXL345_RANGE_2G);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    TEST_ASSERT_EQUAL_MESSAGE(ADXL345_RANGE_2G, accel.getRange(), "Failed to set Range 2G");

    accel.setFullResolution(1);
    TEST_ASSERT_TRUE_MESSAGE(accel.getFullResolution(), "Failed to enable Full Res");
}

void test_measure_mode_control(void)
{
    accel.setMeasureEnabled(true);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    TEST_ASSERT_TRUE_MESSAGE(accel.getMeasureEnabled(), "Failed to enable Measure");

    accel.setMeasureEnabled(false);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    TEST_ASSERT_FALSE_MESSAGE(accel.getMeasureEnabled(), "Failed to disable Measure");

    accel.setMeasureEnabled(true);
}

void test_offset_calibration(void)
{
    ESP_LOGI(TAG, "Testing Offset Registers...");

    // Write arbitrary offsets
    int8_t test_x = 10, test_y = -20, test_z = 5;
    accel.setOffset(test_x, test_y, test_z);

    // Read them back
    int8_t read_x = accel.getOffsetX();
    int8_t read_y = accel.getOffsetY();
    int8_t read_z = accel.getOffsetZ();

    TEST_ASSERT_EQUAL_INT8(test_x, read_x);
    TEST_ASSERT_EQUAL_INT8(test_y, read_y);
    TEST_ASSERT_EQUAL_INT8(test_z, read_z);

    // Reset to zero for normal operation
    accel.setOffset(0, 0, 0);
}

void test_sensor_data_read(void)
{
    accel.setMeasureEnabled(true);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    int16_t ax, ay, az;
    accel.getAcceleration(&ax, &ay, &az);

    ESP_LOGI(TAG, "Readings: X=%d, Y=%d, Z=%d", ax, ay, az);
    TEST_ASSERT_FALSE_MESSAGE(ax == 0 && ay == 0 && az == 0, "All zeros read");
}

extern "C" void app_main(void)
{
    setup_i2c();
    accel.initialize();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    UNITY_BEGIN();
    RUN_TEST(test_connection_and_device_id);
    RUN_TEST(test_configuration_read_write);
    RUN_TEST(test_offset_calibration);
    RUN_TEST(test_measure_mode_control);
    RUN_TEST(test_sensor_data_read);
    UNITY_END();
}
