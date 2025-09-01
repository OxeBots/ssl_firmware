/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <numeric>

#include "driver/ledc.h"
#include "driver/wheel_state_estimator.hpp"
#include "esp_log.h"
#include "pins_assignments.h"

void heartbeat_task(void * pvParam)
{
    ledc_timer_config_t timer_config = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                        .duty_resolution = LEDC_TIMER_10_BIT,
                                        .timer_num = LEDC_TIMER_0,
                                        .freq_hz = 1,
                                        .clk_cfg = LEDC_AUTO_CLK,
                                        .deconfigure = false};

    ledc_timer_config(&timer_config);
    ledc_channel_config_t channel_config = {
      .gpio_num = GPIO_NUM_2,
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
    std::vector<adc_channel_t> channels = {
      config::pin::MOTOR_FRONT_LEFT_ENC, config::pin::MOTOR_BACK_LEFT_ENC,
      config::pin::MOTOR_BACK_RIGHT_ENC, config::pin::MOTOR_FRONT_RIGHT_ENC};

    WheelStateEstimator & estimator = WheelStateEstimator::get_instance();
    estimator.init(channels);
    estimator.init_i2c(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22);

    // --- SENSOR CONFIGURATION ---
    ESP_LOGI("MAIN", "Configuring AS5600 sensor...");
    // set Output pin to reduced mode (10% - 90% of VCC) to be in linear region
    // of ESP32 ADC
    estimator.setOutputStage(AS5600_OUTPUT_STAGE_ANALOG_REDUCED);
    // Set filters for maximum speed:
    estimator.setSlowFilter(AS5600_SLOW_FILTER_2X);
    estimator.setFastFilter(AS5600_FAST_FILTER_THRESH_6LSB);

    // --- CALIBRATION STEP ---
    for (auto ch : channels)
    {
        ESP_LOGI("MAIN", "Calibrating channel %d.", ch);
        if (ch == config::pin::MOTOR_FRONT_LEFT_ENC)
        {
            ESP_ERROR_CHECK(estimator.calibrate_channel(ch));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI("MAIN", "All channels calibrated.");
    // --- BURN SETTINGS (USE WITH CAUTION!) ---
    // Uncomment the following line ONLY ONCE to permanently save the settings
    // above. After running it once, you should comment it out again.
    // ESP_LOGW("MAIN", "Permanently burning settings to sensor...");
    // estimator.burn_settings();
    // ESP_LOGI("MAIN", "Burn command sent. Please power-cycle the device.");
    // while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); } // Halt after burning

    xTaskCreate(heartbeat_task, "LED Blink", configMINIMAL_STACK_SIZE * 2,
                nullptr, 5, nullptr);

    while (true)
    {
        // Log the filtered state for each channel
        for (auto ch : channels)
        {
            if (ch == config::pin::MOTOR_FRONT_LEFT_ENC)
            {
                float angle_deg = estimator.get_filtered_angle_deg(ch);
                float rpm = estimator.get_filtered_rpm(ch);
                float accel_rps2 =
                  estimator.get_filtered_acceleration_rps2(ch);

                ESP_LOGI(
                  "MAIN",
                  "Ch %d | Angle: %7.2f deg | RPM: %8.2f | Accel: %8.2f rps^2",
                  ch, angle_deg, rpm, accel_rps2);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
