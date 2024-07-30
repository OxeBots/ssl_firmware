#pragma once

#define POWER_SUPPLY_VOLTAGE 12         // measured in volts (power supply)
#define SENSOR_DIRECTION Direction::CW  // find_sensor_offset_and_direction.cpp

// Motor parameters
// TODO: get these values from find_sensor_offset_and_direction for each motor

// encoder sensor offset
// #define M1_SENSOR_ZERO_OFFSET XXXXX
#define M2_SENSOR_ZERO_OFFSET 0.4418
// #define M3_SENSOR_ZERO_OFFSET XXXXX
// #define M4_SENSOR_ZERO_OFFSET XXXXX
// To use these values add them to the code:
//    motor.sensor_direction=SENSOR_DIRECTION;
//    motor.zero_electric_angle=M2_SENSOR_ZERO_OFFSET;

#define HD_MOTOR_POLE_PAIRS 4          // find_pole_pairs.cpp
#define HD_MOTOR_PHASE_RESISTANCE 2.6  // measured in ohms (multimeter)
#define HD_MOTOR_KV 450                // find_kv_rating.cpp

#define MKS_SHUNT_RESISTANCE 0.01  // measured in ohms (multimeter)

// TODO: verify this value with part number in the board
// INA181A1 - 20
// INA181A2 - 50
// INA181A3 - 100
// INA181A4 - 200
#define MKS_SENSE_CURRENT_GAIN 50
