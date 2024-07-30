#pragma once
#include <Arduino.h>

struct uart_data
{
    float front_left;
    float front_right;
    float rear_left;
    float rear_right;
    float kick;
    uint32_t id;
};