// MKS DUAL FOC open-loop speed control routine.Test library: SimpleFOC 2.1.1
// Test hardware: MKS DUAL FOC V3.1 Enter "T+number" in the serial port to set
// the speed of the two motors. For example, set the motor to rotate at
// 10rad/s, input "T10", and the motor will rotate at 5rad/s by default when it
// is powered on When using your own motor, please remember to modify the
// default number of pole pairs, that is, the value in BLDCMotor(7), and set it
// to your own number of pole pairs The default power supply voltage set by the
// program is 12V, please remember to modify the values in voltage_power_supply
// and voltage_limit variables if you use other voltages for power supply

#include <Arduino.h>
#include <SimpleFOC.h>

#include <parameters.hpp>

#include "pins_assignments.hpp"

BLDCMotor motor1 =
  BLDCMotor(HD_MOTOR_POLE_PAIRS, HD_MOTOR_PHASE_RESISTANCE, HD_MOTOR_KV);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(M1_A, M1_B, M1_C);

// BLDC motor & driver instance
BLDCMotor motor2 =
  BLDCMotor(HD_MOTOR_POLE_PAIRS, HD_MOTOR_PHASE_RESISTANCE, HD_MOTOR_KV);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(M2_A, M2_B, M2_C);

BLDCMotor motor3 =
  BLDCMotor(HD_MOTOR_POLE_PAIRS, HD_MOTOR_PHASE_RESISTANCE, HD_MOTOR_KV);
BLDCDriver3PWM driver3 = BLDCDriver3PWM(M3_A, M3_B, M3_C);

BLDCMotor motor4 =
  BLDCMotor(HD_MOTOR_POLE_PAIRS, HD_MOTOR_PHASE_RESISTANCE, HD_MOTOR_KV);
BLDCDriver3PWM driver4 = BLDCDriver3PWM(M4_A, M4_B, M4_C);

// Target variable
float target_rpm = 5;
float voltage_limit = 1;

// Serial command setting
Commander command = Commander(Serial);

void doTarget(char * cmd)
{
    command.scalar(&target_rpm, cmd);
    motor1.target = target_rpm;
    motor2.target = target_rpm;
    motor3.target = target_rpm;
    motor4.target = target_rpm;
}

void doLimit(char * cmd)
{
    command.scalar(&voltage_limit, cmd);
    motor1.voltage_limit = voltage_limit;
    motor2.voltage_limit = voltage_limit;
    motor3.voltage_limit = voltage_limit;
    motor4.voltage_limit = voltage_limit;
}

void setup()
{
    driver1.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
    driver1.init();
    motor1.linkDriver(&driver1);
    motor1.voltage_limit = 1;    // [V]

    driver2.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
    driver2.init();
    motor2.linkDriver(&driver2);
    motor2.voltage_limit = 1;    // [V]

    driver3.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
    driver3.init();
    motor3.linkDriver(&driver3);
    motor3.voltage_limit = 1;    // [V]

    driver4.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
    driver4.init();
    motor4.linkDriver(&driver4);
    motor4.voltage_limit = 1;    // [V]

    // Open loop control mode setting
    motor1.controller = MotionControlType::velocity_openloop;
    motor2.controller = MotionControlType::velocity_openloop;
    motor3.controller = MotionControlType::velocity_openloop;
    motor4.controller = MotionControlType::velocity_openloop;

    // Initialize the hardware
    motor1.init();
    motor2.init();
    motor3.init();
    motor4.init();

    // Add T command
    command.add('T', doTarget, "target velocity");
    command.add('L', doLimit, "target voltage");

    Serial.begin(BAUD_RATE);
    Serial.println("Motor ready!");
    Serial.println("Set target velocity [rad/s]");
    _delay(1000);
}

void loop()
{
    motor1.move();
    motor2.move();
    motor3.move();
    motor4.move();

    // User newsletter
    command.run();
}
