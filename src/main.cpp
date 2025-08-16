/*

* SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD

*

* SPDX-License-Identifier: Apache-2.0

*/

#include "driver/as5600_sensor.hpp"
// #include "driver/bl48250_motor.h"

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
    // Define all channels to be read by the ADC
    std::vector<adc_channel_t> channels = {
      config::pin::MOTOR_FRONT_LEFT_ENC, config::pin::MOTOR_BACK_LEFT_ENC,
      config::pin::MOTOR_BACK_RIGHT_ENC, config::pin::MOTOR_FRONT_RIGHT_ENC};

    // Get the single ADC_Reader instance (statically allocated)
    ADC_Reader & adc_reader = ADC_Reader::get_instance();
    adc_reader.init(channels);

    // Create sensor objects on the stack
    std::array<AS5600_Sensor, 4> vel_enc = {
      AS5600_Sensor(config::pin::MOTOR_FRONT_LEFT_ENC, adc_reader),
      AS5600_Sensor(config::pin::MOTOR_BACK_LEFT_ENC, adc_reader),
      AS5600_Sensor(config::pin::MOTOR_BACK_RIGHT_ENC, adc_reader),
      AS5600_Sensor(config::pin::MOTOR_FRONT_RIGHT_ENC, adc_reader)};

    xTaskCreate(heartbeat_task, "LED Blink", configMINIMAL_STACK_SIZE * 2,
                nullptr, 5, nullptr);

    // do código antigo:
    // constexpr std::array<gpio_num_t, 4> motor_pwm_pins = {
    // config::pin::MOTOR_FRONT_LEFT_PWM, config::pin::MOTOR_BACK_LEFT_PWM,
    // config::pin::MOTOR_BACK_RIGHT_PWM,
    // config::pin::MOTOR_FRONT_RIGHT_PWM};
    // constexpr std::array<gpio_num_t, 4> motor_dir_pins = {
    // config::pin::MOTOR_FRONT_LEFT_DIR, config::pin::MOTOR_BACK_LEFT_DIR,
    // config::pin::MOTOR_BACK_RIGHT_DIR,
    // config::pin::MOTOR_FRONT_RIGHT_DIR};
    // constexpr std::array<ledc_channel_t, 4> motor_channels = {
    // LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3, LEDC_CHANNEL_4};
    // BL48250_motor motor_driver(LEDC_TIMER_1, LEDC_HIGH_SPEED_MODE,
    // LEDC_TIMER_12_BIT, 10000, motor_pwm_pins,
    // motor_dir_pins, motor_channels);

    while (true)
    {
        printf("mV: CH0:%-4d\tCH3:%-4d\tCH6:%-4d\tCH7:%-4d\n",
               vel_enc[0].get_calibrated_voltage(),
               vel_enc[1].get_calibrated_voltage(),
               vel_enc[2].get_calibrated_voltage(),
               vel_enc[3].get_calibrated_voltage());

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
