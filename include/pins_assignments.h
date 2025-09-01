#ifndef PINS_ASSIGNMENTS_H
#define PINS_ASSIGNMENTS_H

#include <driver/gpio.h>

// #include "driver/adc_types_legacy.h"

#ifdef ARDUINO_ESP32_DEV

namespace config
{
namespace pin
{
constexpr gpio_num_t BLINK = GPIO_NUM_2;

// M1
constexpr gpio_num_t MOTOR_FRONT_LEFT_FB = GPIO_NUM_32;
constexpr gpio_num_t MOTOR_FRONT_LEFT_DIR = GPIO_NUM_33;
constexpr gpio_num_t MOTOR_FRONT_LEFT_PWM = GPIO_NUM_25;
constexpr adc_channel_t MOTOR_FRONT_LEFT_ENC = ADC_CHANNEL_0;  // GPIO_NUM_36
// M2
constexpr gpio_num_t MOTOR_BACK_LEFT_FB = GPIO_NUM_27;
constexpr gpio_num_t MOTOR_BACK_LEFT_DIR = GPIO_NUM_14;
constexpr gpio_num_t MOTOR_BACK_LEFT_PWM = GPIO_NUM_12;
constexpr adc_channel_t MOTOR_BACK_LEFT_ENC = ADC_CHANNEL_3;  // GPIO_NUM_39
// M3
constexpr gpio_num_t MOTOR_BACK_RIGHT_FB = GPIO_NUM_0;
constexpr gpio_num_t MOTOR_BACK_RIGHT_DIR = GPIO_NUM_26;
constexpr gpio_num_t MOTOR_BACK_RIGHT_PWM = GPIO_NUM_13;
constexpr adc_channel_t MOTOR_BACK_RIGHT_ENC = ADC_CHANNEL_6;  // GPIO_NUM_34
// M4
constexpr gpio_num_t MOTOR_FRONT_RIGHT_FB = GPIO_NUM_4;
constexpr gpio_num_t MOTOR_FRONT_RIGHT_DIR = GPIO_NUM_16;
constexpr gpio_num_t MOTOR_FRONT_RIGHT_PWM = GPIO_NUM_17;
constexpr adc_channel_t MOTOR_FRONT_RIGHT_ENC = ADC_CHANNEL_7;  // GPIO_NUM_35

// Extra sensor
constexpr adc_channel_t EXTRA_SENSOR_ENC = ADC_CHANNEL_9; // GPIO_NUM_26

// Kicker circuit
constexpr gpio_num_t KICKER_EN = GPIO_NUM_1;
constexpr gpio_num_t KICKER_PWM = GPIO_NUM_3;

// IMU
constexpr gpio_num_t IMU_SDA = GPIO_NUM_21;
constexpr gpio_num_t IMU_SCL = GPIO_NUM_22;

// NRF24L01 radio module
constexpr gpio_num_t NRF_CE = GPIO_NUM_15;
constexpr gpio_num_t NRF_CS = GPIO_NUM_5;
constexpr gpio_num_t NRF_MISO = GPIO_NUM_19;
constexpr gpio_num_t NRF_MOSI = GPIO_NUM_23;
constexpr gpio_num_t NRF_SCK = GPIO_NUM_18;

}  // namespace pin
}  // namespace config

#endif  // ARDUINO_ESP32_DEV

#endif  // PINS_ASSIGNMENTS_H
