#include <Arduino.h>
#include <SimpleFOC.h>

#include <parameters.hpp>

#include "pins_assignments.hpp"

MagneticSensorI2C encoder1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C encoder2 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C encoder3 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C encoder4 = MagneticSensorI2C(AS5600_I2C);

BLDCMotor motor1 =
  BLDCMotor(HD_MOTOR_POLE_PAIRS, HD_MOTOR_PHASE_RESISTANCE, HD_MOTOR_KV);

BLDCMotor motor2 =
  BLDCMotor(HD_MOTOR_POLE_PAIRS, HD_MOTOR_PHASE_RESISTANCE, HD_MOTOR_KV);

BLDCMotor motor3 =
  BLDCMotor(HD_MOTOR_POLE_PAIRS, HD_MOTOR_PHASE_RESISTANCE, HD_MOTOR_KV);

BLDCMotor motor4 =
  BLDCMotor(HD_MOTOR_POLE_PAIRS, HD_MOTOR_PHASE_RESISTANCE, HD_MOTOR_KV);

BLDCDriver3PWM driver1 = BLDCDriver3PWM(M1_A, M1_B, M1_C);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(M2_A, M2_B, M2_C);
BLDCDriver3PWM driver3 = BLDCDriver3PWM(M3_A, M3_B, M3_C);
BLDCDriver3PWM driver4 = BLDCDriver3PWM(M4_A, M4_B, M4_C);

float rpm = 0.0;

Commander command = Commander(Serial);
void doTarget(char * cmd)
{
    command.scalar(&rpm, cmd);
    motor1.target = rpm;
    motor2.target = rpm;
    motor3.target = rpm;
    motor4.target = rpm;
}

void setup()
{
    // configure i2C
    Wire.setClock(400000);

    // initialise encoders
    encoder1.init();
    encoder2.init();
    encoder3.init();
    encoder4.init();

    // link encoders to motors
    motor1.linkSensor(&encoder1);
    motor2.linkSensor(&encoder2);
    motor3.linkSensor(&encoder3);
    motor4.linkSensor(&encoder4);

    // initialise drivers
    driver1.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
    driver2.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
    driver3.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
    driver4.voltage_power_supply = POWER_SUPPLY_VOLTAGE;

    driver1.init();
    driver2.init();
    driver3.init();
    driver4.init();

    // link drivers to motors
    motor1.linkDriver(&driver1);
    motor2.linkDriver(&driver2);
    motor3.linkDriver(&driver3);
    motor4.linkDriver(&driver4);

    // set motion control loop to be used
    motor1.controller = MotionControlType::velocity;
    motor2.controller = MotionControlType::velocity;
    motor3.controller = MotionControlType::velocity;
    motor4.controller = MotionControlType::velocity;

    // velocity PI controller parameters
    motor1.PID_velocity.P = 0.01f;
    motor1.PID_velocity.I = 20;
    motor1.PID_velocity.D = 0.01;

    motor2.PID_velocity.P = 0.01f;
    motor2.PID_velocity.I = 20;
    motor2.PID_velocity.D = 0.01;

    motor3.PID_velocity.P = 0.01f;
    motor3.PID_velocity.I = 20;
    motor3.PID_velocity.D = 0.01;

    motor4.PID_velocity.P = 0.01f;
    motor4.PID_velocity.I = 20;
    motor4.PID_velocity.D = 0.01;

    // jerk control using voltage voltage ramp
    // default value is 300 volts per sec  ~ 0.3V per millisecond
    motor1.PID_velocity.output_ramp = 1000;
    motor2.PID_velocity.output_ramp = 1000;
    motor3.PID_velocity.output_ramp = 1000;
    motor4.PID_velocity.output_ramp = 1000;

    motor voltage limit, used to not burn the motor
    motor1.voltage_limit = 3;
    motor2.voltage_limit = 1;
    motor3.voltage_limit = 3;
    motor4.voltage_limit = 3;

    // velocity low pass filtering
    // default 5ms - try different values to see what is the best.
    // the lower the less filtered
    motor1.LPF_velocity.Tf = 0.01f;
    motor2.LPF_velocity.Tf = 0.01f;
    motor3.LPF_velocity.Tf = 0.01f;
    motor4.LPF_velocity.Tf = 0.01f;

    // use monitoring with serial
    Serial.begin(BAUD_RATE);

    // initialize motors
    motor1.init();
    motor2.init();
    motor3.init();
    motor4.init();

    // align sensor and start FOC
    motor1.initFOC();
    motor2.initFOC();
    motor3.initFOC();
    motor4.initFOC();

    //   add target command T
    command.add('T', doTarget, "target velocity");

    Serial.println(F("Motors ready."));
    Serial.println(F("Set the target velocity using serial terminal:"));
    _delay(1000);
}

void loop()
{
    // main FOC algorithm function
    // the faster you run this function the better
    // it calls the encoder update function
    // Arduino UNO loop  ~1kHz
    // Bluepill loop ~10kHz
    motor1.loopFOC();
    motor2.loopFOC();
    motor3.loopFOC();
    motor4.loopFOC();

    // Motion control function
    // velocity, position or voltage (defined in motor.controller)
    // this function can be run at much lower frequency than loopFOC() function
    motor1.move();
    motor2.move();
    motor3.move();
    motor4.move();

    // user communication over serial terminal
    command.run();
}
