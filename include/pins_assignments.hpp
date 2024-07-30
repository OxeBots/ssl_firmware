#pragma once
#include <pins_arduino.h>

// STM32 bluepill_f103c8
#ifdef STM32F1

#define M1_A PB8
#define M1_B PB9
#define M1_C PA0

#define M2_A PA8
#define M2_B PA9
#define M2_C PA10

#define M3_A PA1
#define M3_B PA2
#define M3_C PA3

#define M4_A PA6
#define M4_B PA7
#define M4_C PB0

#endif

// Arduino Mega 2560
#ifdef ARDUINO_AVR_MEGA2560

#define M1_A 2 // pwm pin for phase A of motor 1
#define M1_B 3 // pwm pin for phase B of motor 1
#define M1_C 4 // pwm pin for phase C of motor 1

#define M2_A 5 // pwm pin for phase A of motor 2
#define M2_B 6 // pwm pin for phase B of motor 2
#define M2_C 7 // pwm pin for phase C of motor 2

#define M3_A 8 // pwm pin for phase A of motor 3
#define M3_B 9 // pwm pin for phase B of motor 3
#define M3_C 10 // pwm pin for phase C of motor 3

#define M4_A 11 // pwm pin for phase A of motor 4
#define M4_B 12 // pwm pin for phase B of motor 4
#define M4_C 13 // pwm pin for phase C of motor 4

#define M1_CA A0 // current sense pin for phase A of motor 1
#define M1_CB A1 // current sense pin for phase B of motor 1

#define M2_CA A2 // current sense pin for phase A of motor 2
#define M2_CB A3 // current sense pin for phase B of motor 2

#define M3_CA A4 // current sense pin for phase A of motor 3
#define M3_CB A5 // current sense pin for phase B of motor 3

#define M4_CA A6 // current sense pin for phase A of motor 4
#define M4_CB A7 // current sense pin for phase B of motor 4

#endif

// ESP32 Dev kit
// TODO: Check the pins
#ifdef ARDUINO_ESP32_DEV

#define M1_A 2
#define M1_B 3
#define M1_C 4

#define M2_A 5
#define M2_B 6
#define M2_C 7

#define M3_A 8
#define M3_B 9
#define M3_C 10

#define M4_A 11
#define M4_B 12
#define M4_C 13

#endif
