#ifndef PINS_ASSIGNMENTS_H
#define PINS_ASSIGNMENTS_H

#include <driver/gpio.h>

#ifdef ARDUINO_ESP32_DEV

namespace config
{
namespace pin
{
constexpr gpio_num_t BLINK = GPIO_NUM_2;

// M1
constexpr gpio_num_t MOTOR_FRONT_LEFT_FB = GPIO_NUM_35;
constexpr gpio_num_t MOTOR_FRONT_LEFT_DIR = GPIO_NUM_32;
constexpr gpio_num_t MOTOR_FRONT_LEFT_PWM = GPIO_NUM_33;
// M2
constexpr gpio_num_t MOTOR_BACK_LEFT_FB = GPIO_NUM_27;
constexpr gpio_num_t MOTOR_BACK_LEFT_DIR = GPIO_NUM_14;
constexpr gpio_num_t MOTOR_BACK_LEFT_PWM = GPIO_NUM_12;
// M3
constexpr gpio_num_t MOTOR_BACK_RIGHT_FB = GPIO_NUM_15;
constexpr gpio_num_t MOTOR_BACK_RIGHT_DIR = GPIO_NUM_26;
constexpr gpio_num_t MOTOR_BACK_RIGHT_PWM = GPIO_NUM_0;
// M4
constexpr gpio_num_t MOTOR_FRONT_RIGHT_FB = GPIO_NUM_4;
constexpr gpio_num_t MOTOR_FRONT_RIGHT_DIR = GPIO_NUM_16;
constexpr gpio_num_t MOTOR_FRONT_RIGHT_PWM = GPIO_NUM_17;
}  // namespace pins
}  // namespace config

#endif  // ARDUINO_ESP32_DEV

#endif  // PINS_ASSIGNMENTS_H
