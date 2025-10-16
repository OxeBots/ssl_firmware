/**
 * @file as5600_i2c.hpp
 * @brief I2C driver for the AS5600 magnetic rotary encoder.
 */
#ifndef DRIVER_AS5600_I2C_HPP
#define DRIVER_AS5600_I2C_HPP

#include <esp_err.h>

#include "driver/i2c_wrapper.hpp"

class AS5600_I2C
{
   public:
    enum class OutputStage : uint8_t
    {
        ANALOG_FULL = 0x00,
        ANALOG_REDUCED = 0x01,
        DIGITAL_PWM = 0x02,
    };
    enum class SlowFilter : uint8_t
    {
        FILTER_16X = 0x00,
        FILTER_8X = 0x01,
        FILTER_4X = 0x02,
        FILTER_2X = 0x03
    };
    enum class FastFilter : uint8_t
    {
        THRESH_SLOW_ONLY = 0x00,
        THRESH_6LSB = 0x01,
        THRESH_7LSB = 0x02,
        THRESH_9LSB = 0x03,
        THRESH_18LSB = 0x04,
        THRESH_21LSB = 0x05,
        THRESH_24LSB = 0x06,
        THRESH_10LSB = 0x07
    };

    AS5600_I2C();
    ~AS5600_I2C() = default;

    /**
     * @brief Initializes the AS5600 device using a shared I2C_Wrapper
     * @param i2c_wrapper Reference to a pre-initialized I2C_Wrapper
     * @return ESP_OK on success
     */
    esp_err_t init(I2C_Wrapper &i2c_wrapper);

    // --- Configuration Methods ---
    esp_err_t set_output_stage(OutputStage stage);
    esp_err_t set_slow_filter(SlowFilter filter);
    esp_err_t set_fast_filter(FastFilter threshold);
    esp_err_t burn_settings();

   private:
    enum class Register : uint8_t
    {
        CONF_H = 0x07,
        RAW_ANGLE_H = 0x0C,
        BURN = 0xFF
    };

    enum class Cmd : uint8_t
    {
        BURN = 0x40
    };

    static constexpr uint8_t AS5600_ADDR = 0x36;

    I2C_Wrapper* m_i2c_wrapper = nullptr;
    bool m_initialized = false;

    esp_err_t read_config_register(uint16_t * config);
    esp_err_t write_config_register(uint16_t config);
    esp_err_t read_register(Register reg_addr, uint8_t * data, size_t len);
    esp_err_t write_register(Register reg_addr, uint8_t * data, size_t len);
};

#endif  // DRIVER_AS5600_I2C_HPP