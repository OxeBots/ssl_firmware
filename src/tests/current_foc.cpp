#include <Arduino.h>
#include <SimpleFOC.h>

#include "parameters.hpp"
#include "pins_assignments.hpp"

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire();

BLDCMotor motor =
  BLDCMotor(HD_MOTOR_POLE_PAIRS, HD_MOTOR_PHASE_RESISTANCE, HD_MOTOR_KV);

BLDCDriver3PWM driver = BLDCDriver3PWM(M2_A, M2_B, M2_C);

InlineCurrentSense current_sense = InlineCurrentSense(
  MKS_SHUNT_RESISTANCE, MKS_SENSE_CURRENT_GAIN, M2_CA, M2_CB);

float rpm = 10.0;

Commander command = Commander(Serial);
void doTarget(char * cmd) { command.scalar(&motor.target, cmd); }
void doLimit(char * cmd) { command.scalar(&motor.voltage_limit, cmd); }

void setup()
{
    Serial.begin(BAUD_RATE);

    SimpleFOCDebug::enable(&Serial);

    Wire.setClock(400000);

    // TODO: Set address for I2C sensors
    sensor.init();

    motor.linkSensor(&sensor);

    Serial.println("Sensor ready");
    _delay(1000);

    driver.voltage_power_supply = POWER_SUPPLY_VOLTAGE;

    // limit the maximal dc voltage the driver can set as a protection measure
    // for the low-resistance motors this value is fixed on startup
    driver.voltage_limit = 12;

    if (not driver.init())
    {
        Serial.println("Driver init failed!");
        return;
    }

    // link the motor and the driver
    motor.linkDriver(&driver);
    // link driver to current sense
    current_sense.linkDriver(&driver);
    // limiting motor movements
    // limit the voltage to be set to the motor
    // start very low for high resistance motors
    // current = voltage / resistance, so try to be well under 1Amp
    motor.voltage_limit = 1;  // [V]
    motor.current_limit = 2;  // Amps - default 0.2Amps

    // open loop control config
    motor.controller = MotionControlType::velocity;
    // set torque control type to FOC current
    motor.torque_controller = TorqueControlType::foc_current;
    // set FOC modulation type to sinusoidal
    motor.foc_modulation = FOCModulationType::SinePWM;

    // jerk control using voltage voltage ramp
    // https://docs.simplefoc.com/velocity_loop
    // default value is 300 volts per sec  ~ 0.3V per millisecond
    motor.PID_velocity.output_ramp = 10;

    // controller configuration based on the control type
    // velocity PID controller parameters
    // default P=0.5 I = 10 D = 0
    motor.PID_velocity.P = 2;
    motor.PID_velocity.I = 0;
    motor.PID_velocity.D = 0.0;

    // velocity low pass filtering
    // default 5ms - try different values to see what is the best.
    // the lower the less filtered
    motor.LPF_velocity.Tf = 0.01;

    // init motor hardware
    if (not motor.init())
    {
        Serial.println("Motor init failed!");
        return;
    }

    // initialize current sensing and link it to the motor
    // https://docs.simplefoc.com/inline_current_sense#where-to-place-the-current_sense-configuration-in-your-foc-code
    if (current_sense.init())
        Serial.println("Current sense init success!");
    else
    {
        Serial.println("Current sense init failed!");
        return;
    }

    current_sense.gain_b *= -1;
    current_sense.gain_a *= -1;
    // current_sense.skip_align = true;
    motor.linkCurrentSense(&current_sense);

    command.add('T', doTarget, "target velocity");
    command.add('L', doLimit, "voltage limit");

    // align sensor and start FOC
    motor.initFOC();

    Serial.println("Motor ready!");
    Serial.println("Set target velocity [rad/s]");
    _delay(1000);
}

void loop()
{
    // main FOC algorithm function
    // the faster you run this function the better
    // Arduino UNO loop  ~1kHz
    // Bluepill loop ~10kHz
    motor.loopFOC();

    // Motion control function
    // velocity, position or voltage (defined in motor.controller)
    // this function can be run at much lower frequency than loopFOC() function
    // You can also use motor.move() and set the motor.target in the code
    motor.move();

    // user communication
    command.run();
}
