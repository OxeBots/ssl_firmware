/**
 * @file as5600_i2c.hpp
 * @brief I2C driver for the AS5600_I2C magnetic rotary encoder.
 */
#ifndef DRIVER_AS5600_I2C_HPP
#define DRIVER_AS5600_I2C_HPP

#include <driver/i2c_master.h>
#include <esp_err.h>

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
    ~AS5600_I2C();

    /**
     * @brief Initializes the I2C master bus and adds the AS5600_I2C device.
     * @param i2c_port I2C port number.
     * @param sda_pin GPIO for SDA.
     * @param scl_pin GPIO for SCL.
     * @return ESP_OK on success.
     */
    esp_err_t init(i2c_port_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin);

    // --- Configuration Methods ---
    esp_err_t set_output_stage(OutputStage stage);
    esp_err_t set_slow_filter(SlowFilter filter);
    esp_err_t set_fast_filter(FastFilter threshold);

    /**
     * @brief Permanently burns the current configuration to OTP memory.
     * @warning This is a permanent operation.
     * @return ESP_OK on success.
     */
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

    // --- I2C handles ---
    bool m_initialized = false;
    i2c_master_bus_handle_t m_bus_handle = nullptr;
    i2c_master_dev_handle_t m_dev_handle = nullptr;

    // --- Low-level I2C helpers ---
    esp_err_t read_register(Register reg_addr, uint8_t * data, size_t len);
    esp_err_t write_register(Register reg_addr, uint8_t * data, size_t len);
    esp_err_t read_config_register(uint16_t * config);
    esp_err_t write_config_register(uint16_t config);
};

#endif  // DRIVER_AS5600_I2C_HPP
