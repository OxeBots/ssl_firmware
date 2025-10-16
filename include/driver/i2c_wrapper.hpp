/**
 * @file i2c_wrapper.hpp
 * @brief Generic I2C device wrapper for ESP32
 */
#ifndef DRIVER_I2C_WRAPPER_HPP
#define DRIVER_I2C_WRAPPER_HPP

#include <driver/i2c_master.h>
#include <esp_err.h>

#include <map>

class I2C_Wrapper
{
   public:
    I2C_Wrapper();
    ~I2C_Wrapper();

    /**
     * @brief Initializes the I2C master bus
     * @param i2c_port I2C port number
     * @param sda_pin GPIO for SDA
     * @param scl_pin GPIO for SCL
     * @return ESP_OK on success
     */
    esp_err_t init_bus(i2c_port_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin);

    /**
     * @brief Adds a device to the I2C bus
     * @param device_address 7-bit I2C device address
     * @param scl_speed_hz I2C clock speed in Hz (default: 100000)
     * @return ESP_OK on success
     */
    esp_err_t add_device(uint8_t device_address, uint32_t scl_speed_hz = 100000);

    /**
     * @brief Reads from an I2C device register
     * @param device_address 7-bit I2C device address
     * @param reg_addr Register address to read from
     * @param data Buffer to store read data
     * @param len Number of bytes to read
     * @return ESP_OK on success
     */
    esp_err_t read_register(uint8_t device_address, uint8_t reg_addr, uint8_t * data, size_t len);

    /**
     * @brief Writes to an I2C device register
     * @param device_address 7-bit I2C device address
     * @param reg_addr Register address to write to
     * @param data Data to write
     * @param len Number of bytes to write
     * @return ESP_OK on success
     */
    esp_err_t write_register(uint8_t device_address, uint8_t reg_addr, uint8_t * data, size_t len);

    /**
     * @brief Checks if the I2C wrapper is initialized
     * @return true if initialized
     */
    bool is_initialized() const { return m_initialized; }

    /**
     * @brief Checks if a device is added to the bus
     * @param device_address 7-bit I2C device address
     * @return true if device is added
     */
    bool is_device_added(uint8_t device_address) const;

   private:
    bool m_initialized = false;
    i2c_master_bus_handle_t m_bus_handle = nullptr;
    std::map<uint8_t, i2c_master_dev_handle_t> m_devices;
};

#endif  // DRIVER_I2C_WRAPPER_HPP
