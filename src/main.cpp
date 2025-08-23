/*

* SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD

*

* SPDX-License-Identifier: Apache-2.0

*/

#include "driver/as5600_sensor.hpp"
// #include "driver/bl48250_motor.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "hal/adc_reader.hpp"
#include "hal/encoder_handler.hpp"
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

    ADC_Reader & adc_reader = ADC_Reader::get_instance();
    adc_reader.init(channels);

    // EncoderHandler & encoder_handler = EncoderHandler::get_instance();
    // encoder_handler.init(channels);

    xTaskCreate(heartbeat_task, "LED Blink", configMINIMAL_STACK_SIZE * 2,
                nullptr, 5, nullptr);

    while (true)
    {
        // printf("RPM: FL:%.2f\tBL:%.2f\tBR:%.2f\tFR:%.2f\n",
        //        encoder_handler.get_rpm(0), encoder_handler.get_rpm(1),
        //        encoder_handler.get_rpm(2), encoder_handler.get_rpm(3));

        ESP_LOGI("MAIN", "Raw ADC: FL:%d\tBL:%d\tBR:%d\tFR:%d",
                 adc_reader.get_raw_data(config::pin::MOTOR_FRONT_LEFT_ENC),
                 adc_reader.get_raw_data(config::pin::MOTOR_BACK_LEFT_ENC),
                 adc_reader.get_raw_data(config::pin::MOTOR_BACK_RIGHT_ENC),
                 adc_reader.get_raw_data(config::pin::MOTOR_FRONT_RIGHT_ENC));

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
