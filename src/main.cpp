#include <driver/ledc.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <sdkconfig.h>
#include <stdio.h>

#include <numeric>
#include <vector>

#include "AS5600.h"
#include "BL48250.h"
#include "mirf.h"
#include "omni_robot.h"
#include "wheel_state_estimator.h"

static const char * TAG = "MAIN";

i2c_master_bus_handle_t bus_handle;
#define I2C_PORT_NUM I2C_NUM_0

void heartbeat_task(void * pvParam)
{
    ledc_timer_config_t timer_config = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                        .duty_resolution = LEDC_TIMER_10_BIT,
                                        .timer_num = LEDC_TIMER_0,
                                        .freq_hz = 1,
                                        .clk_cfg = LEDC_AUTO_CLK,
                                        .deconfigure = false};

    ledc_timer_config(&timer_config);

    ledc_channel_config_t channel_config = {.gpio_num = (gpio_num_t)CONFIG_BLINK_GPIO,
                                            .speed_mode = LEDC_LOW_SPEED_MODE,
                                            .channel = LEDC_CHANNEL_0,
                                            .intr_type = LEDC_INTR_DISABLE,
                                            .timer_sel = LEDC_TIMER_0,
                                            .duty = 1UL << (timer_config.duty_resolution - 1),
                                            .hpoint = 0,
                                            .sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE,
                                            .flags = {.output_invert = 0}};

    ledc_channel_config(&channel_config);
    vTaskDelete(nullptr);
}

void setup_i2c()
{
    i2c_master_bus_config_t i2c_mst_config = {.i2c_port = I2C_PORT_NUM,
                                              .sda_io_num = (gpio_num_t)CONFIG_SDA_GPIO,
                                              .scl_io_num = (gpio_num_t)CONFIG_SCL_GPIO,
                                              .clk_source = I2C_CLK_SRC_DEFAULT,
                                              .glitch_ignore_cnt = 7,
                                              .intr_priority = 0,
                                              .trans_queue_depth = 0,
                                              .flags = {.enable_internal_pullup = 1, .allow_pd = 0}};

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    I2Cdev::init(bus_handle);
    ESP_LOGI(TAG, "I2C Initialized");
}

extern "C" void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    setup_i2c();

    const std::array<adc_channel_t, 4> wheel_adc_channels = {
      static_cast<adc_channel_t>(CONFIG_MOTOR_FL_ENC_CHANNEL), static_cast<adc_channel_t>(CONFIG_MOTOR_BL_ENC_CHANNEL),
      static_cast<adc_channel_t>(CONFIG_MOTOR_BR_ENC_CHANNEL), static_cast<adc_channel_t>(CONFIG_MOTOR_FR_ENC_CHANNEL)};

    WheelStateEstimator & w_state_estimator = WheelStateEstimator::get_instance();
    ESP_ERROR_CHECK(w_state_estimator.init(wheel_adc_channels, ADC_ATTEN_DB_12));

    // --- CALIBRATION STEP ---
    // ESP_LOGI(TAG, "Starting sensor range calibration...");

    // if (w_state_estimator.calibrate_wheel_encoders_range(2000) == ESP_OK)
    //     ESP_LOGI(TAG, "All channels calibrated.");
    // else
    //     ESP_LOGE(TAG, "Failed to calibrate all channels.");

    xTaskCreate(heartbeat_task, "LED Blink", configMINIMAL_STACK_SIZE * 2, nullptr, 5, nullptr);

    w_state_estimator.load_or_calibrate(5000);

    // Main loop
    while (true)
    {
        // Get the latest filtered data from the wheel state estimator
        const std::array<float, NUM_ENC_CHANNELS> angles_rad = w_state_estimator.get_filtered_angle_rad();
        const std::array<float, NUM_ENC_CHANNELS> angles_deg = w_state_estimator.get_filtered_angle_deg();
        const std::array<float, NUM_ENC_CHANNELS> rpms = w_state_estimator.get_filtered_rpm();
        const std::array<float, NUM_ENC_CHANNELS> accels = w_state_estimator.get_filtered_acceleration_rps2();

        // Log the data for each wheel
        for (int i = 0; i < NUM_ENC_CHANNELS; ++i)
        {
            printf(">w_%d_rad:%f\n", i + 1, angles_rad[i]);
            printf(">w_%d_deg:%f\n", i + 1, angles_deg[i]);
            printf(">w_%d_rpm:%f\n", i + 1, rpms[i]);
            printf(">w_%d_acc:%f\n", i + 1, accels[i]);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
