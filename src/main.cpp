/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <numeric>

#include "driver/ledc.h"
#include "esp_log.h"
#include "driver/wheel_state_estimator.hpp"  // Use the new header
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

    // Use the new class name and a more descriptive variable name
    WheelStateEstimator & estimator = WheelStateEstimator::get_instance();
    estimator.init(channels);
    estimator.init_i2c(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22);
    estimator.setOutputStage(AS5600_OUTPUT_STAGE_ANALOG_REDUCED);

    xTaskCreate(heartbeat_task, "LED Blink", configMINIMAL_STACK_SIZE * 2,
                nullptr, 5, nullptr);

    while (true)
    {
        // Log the filtered state for each channel
        for (auto ch : channels)
        {
            if (ch == config::pin::MOTOR_FRONT_LEFT_ENC)
            {
                // Use the new get_instance() and method calls
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
