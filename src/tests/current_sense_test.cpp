/**
 * Testing example code for the Inline current sensing class
 */
#include <SimpleFOC.h>

#include <parameters.hpp>
#include <pins_assignments.hpp>

// current sensor
// pins phase A,B
InlineCurrentSense current_sense1 = InlineCurrentSense(
  MKS_SHUNT_RESISTANCE, MKS_SENSE_CURRENT_GAIN, M1_CA, M1_CB);

InlineCurrentSense current_sense2 = InlineCurrentSense(
  MKS_SHUNT_RESISTANCE, MKS_SENSE_CURRENT_GAIN, M2_CA, M2_CB);

void setup()
{
    // use monitoring with serial
    Serial.begin(BAUD_RATE);
    // enable more verbose output for debugging
    // comment out if not needed
    SimpleFOCDebug::enable(&Serial);

    // initialise the current sensing
    if (!current_sense1.init() && !current_sense2.init())
    {
        Serial.println("Current sense init failed.");
        return;
    }

    // for SimpleFOCShield v2.01/v2.0.2
    current_sense1.gain_b *= -1;
    current_sense2.gain_b *= -1;

    Serial.println("Current sense ready.");
}

void loop()
{
    PhaseCurrent_s currents1 = current_sense1.readAverageCurrents();
    float current_magnitude1 = current_sense1.getDCCurrent();
    PhaseCurrent_s currents2 = current_sense2.readAverageCurrents();
    float current_magnitude2 = current_sense2.getDCCurrent();

    Serial.print("Motor 2: ");
    Serial.print(currents2.a * 1000);  // milli Amps
    Serial.print("\t");
    Serial.print(currents2.b * 1000);  // milli Amps
    Serial.print("\t");
    Serial.print(currents2.c * 1000);  // milli Amps
    Serial.print("\t");
    Serial.println(current_magnitude2 * 1000);  // milli Amps

    delay(100);

    Serial.print("Motor 1: ");
    Serial.print(currents1.a * 1000);  // milli Amps
    Serial.print("\t");
    Serial.print(currents1.b * 1000);  // milli Amps
    Serial.print("\t");
    Serial.print(currents1.c * 1000);  // milli Amps
    Serial.print("\t");
    Serial.println(current_magnitude1 * 1000);  // milli Amps
}
