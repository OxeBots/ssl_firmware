#include <driver/ledc.h>
#include <esp_log.h>
#include <stdio.h>

#include <numeric>

#include "driver/as5600_i2c.hpp"
#include "driver/bl48250.hpp"
#include "kinematics/omnidirectional_robot.hpp"
#include "kinematics/wheel_odometry.hpp"
#include "pins_assignments.h"

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
    ledc_channel_config_t channel_config = {.gpio_num = GPIO_NUM_2,
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
    const std::vector<adc_channel_t> wheel_adc_channels = {
      config::pin::MOTOR_FRONT_LEFT_ENC, config::pin::MOTOR_BACK_LEFT_ENC, config::pin::MOTOR_BACK_RIGHT_ENC,
      config::pin::MOTOR_FRONT_RIGHT_ENC};

    WheelOdometry & w_odom = WheelOdometry::get_instance();
    ESP_ERROR_CHECK(w_odom.init(wheel_adc_channels, ADC_ATTEN_DB_12));
    AS5600_I2C as5600_i2c_driver;

    if (as5600_i2c_driver.init(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22) == ESP_OK)
    {
        ESP_LOGI(TAG, "AS5600_I2C I2C driver initialized successfully.");
        as5600_i2c_driver.set_output_stage(AS5600_I2C::OutputStage::ANALOG_REDUCED);
        as5600_i2c_driver.set_slow_filter(AS5600_I2C::SlowFilter::FILTER_2X);
        as5600_i2c_driver.set_fast_filter(AS5600_I2C::FastFilter::THRESH_6LSB);
        // BURN settings, use with caution because its physically limited
        // as5600_i2c_driver.burn_settings();
        // wait indefinitely
        while (true) vTaskDelay(pdMS_TO_TICKS(100));

        ESP_LOGI(TAG, "Set AS5600_I2C for fast read.");
    }
    else
        ESP_LOGE(TAG, "Failed to initialize AS5600_I2C driver.");

    // --- CALIBRATION STEP ---
    ESP_LOGI(TAG, "Starting sensor range calibration...");

    // This will iterate through all active channels for calibration, each
    // encoder need to be rotated
    if (w_odom.calibrate_wheel_encoders(2000) == ESP_OK)
        ESP_LOGI(TAG, "All channels calibrated.");
    else
        ESP_LOGE(TAG, "Failed to calibrate all channels.");

    xTaskCreate(heartbeat_task, "LED Blink", configMINIMAL_STACK_SIZE * 2, nullptr, 5, nullptr);

    while (true)
    {
        // Get the latest filtered data from the wheel odometry
        std::vector<float> angles_rad = w_odom.get_filtered_angle_rad();
        std::vector<float> angles_deg = w_odom.get_filtered_angle_deg();
        std::vector<float> rpms = w_odom.get_filtered_rpm();
        std::vector<float> accels = w_odom.get_filtered_acceleration_rps2();

        // Log the data for each wheel
        for (size_t i = 0; i < angles_rad.size(); ++i)
        {
            if (i == 0)
                ESP_LOGI(TAG,
                         "Wheel %d -> Angle: %.2f rad (%.2f deg), RPM: %.2f, "
                         "Accel: %.2f rps^2",
                         i, angles_rad[i], angles_deg[i], rpms[i], accels[i]);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
