#include <driver/ledc.h>
#include <esp_log.h>
#include <sdkconfig.h>
#include <stdio.h>

#include <numeric>
#include <vector>

#include "AS5600.h"
#include "BL48250.h"
#include "mirf.h"
#include "omni_robot.h"
#include "wheel_odom.h"

static const char * TAG = "MAIN";

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

extern "C" void app_main(void)
{
    const std::array<adc_channel_t, 4> wheel_adc_channels = {
      static_cast<adc_channel_t>(CONFIG_MOTOR_FL_ENC_CHANNEL), static_cast<adc_channel_t>(CONFIG_MOTOR_BL_ENC_CHANNEL),
      static_cast<adc_channel_t>(CONFIG_MOTOR_BR_ENC_CHANNEL), static_cast<adc_channel_t>(CONFIG_MOTOR_FR_ENC_CHANNEL)};

    WheelOdometry & w_odom = WheelOdometry::get_instance();
    ESP_ERROR_CHECK(w_odom.init(wheel_adc_channels, ADC_ATTEN_DB_12));

    // Create single I2C_Wrapper instance for the entire bus
    // I2C_Wrapper i2c_wrapper;

    // esp_err_t ret = i2c_wrapper.init_bus(I2C_NUM_0, (gpio_num_t)CONFIG_IMU_SDA_GPIO,
    // (gpio_num_t)CONFIG_IMU_SCL_GPIO);

    // if (ret != ESP_OK)
    // {
    //     ESP_LOGE(TAG, "Failed to initialize I2C bus");
    // }

    // Initialize AS5600 sensor using the shared I2C wrapper
    // AS5600 as5600_i2c_driver;
    // if (as5600_i2c_driver.init_i2c() == ESP_OK)
    // {
    //     ESP_LOGI(TAG, "AS5600 I2C driver initialized successfully.");

    //     // Configure AS5600 settings
    //     as5600_i2c_driver.set_output_stage(AS5600::OutputStage::ANALOG_REDUCED);
    //     as5600_i2c_driver.set_slow_filter(AS5600::SlowFilter::FILTER_2X);
    //     as5600_i2c_driver.set_fast_filter(AS5600::FastFilter::THRESH_6LSB);
    //     ESP_LOGI(TAG, "Set AS5600 for fast read.");

    //     // BURN settings, use with caution because it's physically limited
    //     // as5600_i2c_driver.burn_settings();
    // }
    // else
    // {
    //     ESP_LOGE(TAG, "Failed to initialize AS5600 driver.");
    // }

    // --- CALIBRATION STEP ---
    // ESP_LOGI(TAG, "Starting sensor range calibration...");

    // if (w_odom.calibrate_wheel_encoders(2000) == ESP_OK)
    //     ESP_LOGI(TAG, "All channels calibrated.");
    // else
    //     ESP_LOGE(TAG, "Failed to calibrate all channels.");

    xTaskCreate(heartbeat_task, "LED Blink", configMINIMAL_STACK_SIZE * 2, nullptr, 5, nullptr);

    // Main loop
    while (true)
    {
        // Get the latest filtered data from the wheel odometry
        const std::array<float, NUM_ENC_CHANNELS> angles_rad = w_odom.get_filtered_angle_rad();
        const std::array<float, NUM_ENC_CHANNELS> angles_deg = w_odom.get_filtered_angle_deg();
        const std::array<float, NUM_ENC_CHANNELS> rpms = w_odom.get_filtered_rpm();
        const std::array<float, NUM_ENC_CHANNELS> accels = w_odom.get_filtered_acceleration_rps2();

        // Log the data for each wheel
        for (int i = 0; i < NUM_ENC_CHANNELS; ++i)
        {
            ESP_LOGI(TAG,
                     "Wheel %d -> Angle: %.2f rad (%.2f deg), RPM: %.2f, "
                     "Accel: %.2f rps^2",
                     i + 1, angles_rad[i], angles_deg[i], rpms[i], accels[i]);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
