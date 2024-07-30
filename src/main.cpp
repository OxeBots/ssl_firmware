#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include <nRF24L01.h>

#include "data.hpp"

// Create an RF24 object
RF24 radio(9, 10);  // CE, CSN pins
struct uart_data data;

// Address for the communication. Both sender and receiver must use the same
// address
const byte address[6] = "00001";

void setup()
{
    // Initialize Serial Monitor
    Serial.begin(BAUD_RATE);
    // Initialize the RF24 radio
    if (!radio.begin())
    {
        Serial.println("Radio hardware is not responding!!!");
        while (1)
        {
        };
    }
    // Set the address
    radio.openReadingPipe(0, address);
    // Start listening
    radio.startListening();
}

void loop()
{
    if (radio.available())
    {
        radio.read(&data, sizeof(data));
        Serial.print("Front Left: ");
        Serial.print(data.front_left);
        Serial.print(" Front Right: ");
        Serial.print(data.front_right);
        Serial.print(" Rear Left: ");
        Serial.print(data.rear_left);
        Serial.print(" Rear Right: ");
        Serial.print(data.rear_right);
        Serial.print(" Kick: ");
        Serial.print(data.kick);
        Serial.print(" ID: ");
        Serial.println(data.id);
    }
}
