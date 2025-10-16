#include <driver/ledc.h>
#include <esp_log.h>
#include <stdio.h>

#include <numeric>

#include "driver/as5600_i2c.hpp"
#include "driver/bl48250.hpp"
#include "driver/gy-85.hpp"
#include "driver/i2c_wrapper.hpp"
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

    // Create single I2C_Wrapper instance for the entire bus
    I2C_Wrapper i2c_wrapper;

    esp_err_t ret = i2c_wrapper.init_bus(I2C_NUM_0, config::pin::I2C_SDA, config::pin::I2C_SCL);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize I2C bus");
    }

    // Initialize AS5600 sensor using the shared I2C wrapper
    // AS5600_I2C as5600_i2c_driver;
    // if (as5600_i2c_driver.init(i2c_wrapper) == ESP_OK)
    // {
    //     ESP_LOGI(TAG, "AS5600 I2C driver initialized successfully.");

    //     // Configure AS5600 settings
    //     as5600_i2c_driver.set_output_stage(AS5600_I2C::OutputStage::ANALOG_REDUCED);
    //     as5600_i2c_driver.set_slow_filter(AS5600_I2C::SlowFilter::FILTER_2X);
    //     as5600_i2c_driver.set_fast_filter(AS5600_I2C::FastFilter::THRESH_6LSB);
    //     ESP_LOGI(TAG, "Set AS5600 for fast read.");

    //     // BURN settings, use with caution because it's physically limited
    //     // as5600_i2c_driver.burn_settings();
    // }
    // else
    // {
    //     ESP_LOGE(TAG, "Failed to initialize AS5600 driver.");
    // }

    // Initialize GY-85 IMU using the shared I2C wrapper
    GY85_I2C gy85_imu;
    if (gy85_imu.init(i2c_wrapper) == ESP_OK)
    {
        ESP_LOGI(TAG, "GY-85 IMU initialized successfully.");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to initialize GY-85 IMU.");
    }

    // --- CALIBRATION STEP ---
    ESP_LOGI(TAG, "Starting sensor range calibration...");

    if (w_odom.calibrate_wheel_encoders(2000) == ESP_OK)
        ESP_LOGI(TAG, "All channels calibrated.");
    else
        ESP_LOGE(TAG, "Failed to calibrate all channels.");

    xTaskCreate(heartbeat_task, "LED Blink", configMINIMAL_STACK_SIZE * 2, nullptr, 5, nullptr);

    // Main loop
    while (true)
    {
        // Get the latest filtered data from the wheel odometry
        std::vector<float> angles_rad = w_odom.get_filtered_angle_rad();
        std::vector<float> angles_deg = w_odom.get_filtered_angle_deg();
        std::vector<float> rpms = w_odom.get_filtered_rpm();
        std::vector<float> accels = w_odom.get_filtered_acceleration_rps2();

        // Read IMU data
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        int16_t mx, my, mz;

        if (gy85_imu.read_accel(ax, ay, az) == ESP_OK && gy85_imu.read_gyro(gx, gy, gz) == ESP_OK &&
            gy85_imu.read_mag(mx, my, mz) == ESP_OK)
        {
            ESP_LOGI("IMU", "ACC: %d %d %d | GYRO: %d %d %d | MAG: %d %d %d", ax, ay, az, gx, gy, gz, mx, my, mz);
        }
        else
        {
            ESP_LOGE("IMU", "Failed to read IMU data");
        }

        // Log the data for each wheel
        for (size_t i = 0; i < angles_rad.size(); ++i)
        {
            ESP_LOGI(TAG,
                     "Wheel %d -> Angle: %.2f rad (%.2f deg), RPM: %.2f, "
                     "Accel: %.2f rps^2",
                     i, angles_rad[i], angles_deg[i], rpms[i], accels[i]);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
