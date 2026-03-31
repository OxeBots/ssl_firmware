#include <driver/i2c_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <unity.h>

#include "I2Cdev.h"
#include "QMC5883L.h"

QMC5883L mag;
const char * TAG = "QMC5883L_TEST";
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
    i2c_master_bus_config_t i2c_mst_config = {.i2c_port = (i2c_port_t)CONFIG_I2C_PORT_NUM,
                                              .sda_io_num = (gpio_num_t)CONFIG_SDA_GPIO,
                                              .scl_io_num = (gpio_num_t)CONFIG_SCL_GPIO,
                                              .clk_source = I2C_CLK_SRC_DEFAULT,
                                              .glitch_ignore_cnt = 7,
                                              .intr_priority = 0,
                                              .trans_queue_depth = 0,
                                              .flags = {.enable_internal_pullup = 1, .allow_pd = 0}};

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    I2Cdev::init(bus_handle);
}

void test_connection(void)
{
    ESP_LOGI(TAG, "Testing Connection...");
    TEST_ASSERT_TRUE_MESSAGE(mag.testConnection(), "QMC5883L Not Connected!");
}

void test_reading_and_azimuth(void)
{
    mag.initialize();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    mag.read();

    int16_t x = mag.getX();
    int16_t y = mag.getY();
    int16_t z = mag.getZ();
    int azimuth = mag.getAzimuth();

    char direction[4];
    mag.getDirection(direction, azimuth);

    ESP_LOGI(TAG, "X: %d, Y: %d, Z: %d | Azimuth: %d deg | Direction: %s", x, y, z, azimuth, direction);

    TEST_ASSERT_FALSE_MESSAGE(x == 0 && y == 0 && z == 0, "All readings are zero");
}

extern "C" void app_main(void)
{
    setup_i2c();

    // Small delay to ensure boot up
    vTaskDelay(100 / portTICK_PERIOD_MS);
    UNITY_BEGIN();
    RUN_TEST(test_connection);
    RUN_TEST(test_reading_and_azimuth);
    UNITY_END();
}
