#ifndef _VL53L5CX_H_
#define _VL53L5CX_H_


#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <I2Cdev.h>

class VL53L5CX 
{
    public: 

    static constexpr uint8_t DEFAULT_ADDRESS = 0x; 
    static constexpr uint8_t ADDRESS_ALT_HIGH = 0x;

    enum class Register : unint8_t
    {

    }

}

