#include <driver/i2c_master.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs_flash.h>

#include "I2Cdev.h"
#include "QMC5883L.h"
#include "sdkconfig.h"

static const char * TAG = "CALIBRATION";

QMC5883L mag;
i2c_master_bus_handle_t bus_handle;

void setup_i2c()
{
    i2c_master_bus_config_t i2c_mst_config = {.i2c_port = (i2c_port_t)CONFIG_I2C_PORT_NUM,
                                              .sda_io_num = (gpio_num_t)CONFIG_SDA_GPIO,
                                              .scl_io_num = (gpio_num_t)CONFIG_SCL_GPIO,
                                              .clk_source = I2C_CLK_SRC_DEFAULT,
                                              .glitch_ignore_cnt = 7,
                                              .intr_priority = 0,
                                              .trans_queue_depth = 0,
                                              .flags = {.enable_internal_pullup = true, .allow_pd = false}};

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    I2Cdev::init(bus_handle);
}

extern "C" void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    setup_i2c();
    vTaskDelay(200 / portTICK_PERIOD_MS);

    mag.init();
    if (!mag.test_connection())
    {
        ESP_LOGE(TAG, "QMC5883L Connection Failed! Check wiring.");
        return;
    }

    // Check for existing calibration
    if (mag.load_calibration_from_nvs() == ESP_OK)
    {
        ESP_LOGI(TAG, "Existing calibration loaded. Starting new calibration will overwrite it.");
    }

    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "      QMC5883L CALIBRATION UTILITY");
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGW(TAG, "This process will run for 10 seconds.");
    ESP_LOGW(TAG,
             "When it starts, rotate the robot slowly in its axis (yaw) direction, ideally completing multiple full "
             "rotations. Try to keep the sensor level and avoid tilting.");
    ESP_LOGI(TAG, "Starting in 5 seconds...");

    for (int i = 5; i > 0; i--)
    {
        ESP_LOGI(TAG, "%d...", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    ESP_LOGW(TAG, ">>> START MOVING THE ROBOT NOW! <<<");
    mag.start_calibration_mode(10);  // 10 seconds

    bool finished = false;
    while (!finished)
    {
        finished = mag.calibration_update();  // reads sensor, updates min/max
        vTaskDelay(10 / portTICK_PERIOD_MS);  // yield
    }

    // This computes results AND saves to NVS
    mag.stop_calibration_mode();
    ESP_LOGI(TAG, ">>> STOP. Calibration done and saved. <<<");

    ESP_LOGI(TAG, "Testing live data with new calibration...");
    while (true)
    {
        mag.read();
        int az = mag.get_azimuth();
        char dir[4];
        mag.get_direction(dir, az);
        ESP_LOGI(TAG, "Azimuth: %d deg | Direction: %s | X: %d Y: %d Z: %d", az, dir, mag.get_x(), mag.get_y(),
                 mag.get_z());
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
