#include "driver/i2c_wrapper.hpp"

#include <cstring>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char * TAG = "I2C_Wrapper";
static constexpr int I2C_TIMEOUT_MS = 100;

I2C_Wrapper::I2C_Wrapper()
{
}

I2C_Wrapper::~I2C_Wrapper()
{
    if (m_initialized)
    {
        // Remove all devices
        for (auto & device : m_devices)
        {
            i2c_master_bus_rm_device(device.second);
        }
        m_devices.clear();

        // Delete the bus
        i2c_del_master_bus(m_bus_handle);
    }
}

esp_err_t I2C_Wrapper::init_bus(i2c_port_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin)
{
    if (m_initialized)
        return ESP_ERR_INVALID_STATE;

    i2c_master_bus_config_t i2c_mst_config = {
      .i2c_port = i2c_port,
      .sda_io_num = sda_pin,
      .scl_io_num = scl_pin,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags = {.enable_internal_pullup = true},
    };
    esp_err_t ret = i2c_new_master_bus(&i2c_mst_config, &m_bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }

    m_initialized = true;
    ESP_LOGI(TAG, "I2C wrapper initialized successfully.");
    return ESP_OK;
}

esp_err_t I2C_Wrapper::add_device(uint8_t device_address, uint32_t scl_speed_hz)
{
    if (!m_initialized)
        return ESP_ERR_INVALID_STATE;

    // Check if device already exists
    if (m_devices.find(device_address) != m_devices.end())
    {
        ESP_LOGW(TAG, "Device 0x%02X already added", device_address);
        return ESP_OK;
    }

    i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = device_address,
      .scl_speed_hz = scl_speed_hz,
    };

    i2c_master_dev_handle_t dev_handle;
    esp_err_t ret = i2c_master_bus_add_device(m_bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add I2C device 0x%02X: %s", device_address, esp_err_to_name(ret));
        return ret;
    }

    m_devices[device_address] = dev_handle;
    ESP_LOGI(TAG, "Added I2C device 0x%02X", device_address);
    return ESP_OK;
}

bool I2C_Wrapper::is_device_added(uint8_t device_address) const
{
    return m_devices.find(device_address) != m_devices.end();
}

esp_err_t I2C_Wrapper::read_register(uint8_t device_address, uint8_t reg_addr, uint8_t * data, size_t len)
{
    if (!m_initialized)
        return ESP_ERR_INVALID_STATE;

    auto it = m_devices.find(device_address);
    if (it == m_devices.end())
    {
        ESP_LOGE(TAG, "Device 0x%02X not found. Call add_device() first.", device_address);
        return ESP_ERR_NOT_FOUND;
    }

    return i2c_master_transmit_receive(it->second, &reg_addr, 1, data, len, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

esp_err_t I2C_Wrapper::write_register(uint8_t device_address, uint8_t reg_addr, uint8_t * data, size_t len)
{
    if (!m_initialized)
        return ESP_ERR_INVALID_STATE;

    auto it = m_devices.find(device_address);
    if (it == m_devices.end())
    {
        ESP_LOGE(TAG, "Device 0x%02X not found. Call add_device() first.", device_address);
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t write_buf[len + 1];
    write_buf[0] = reg_addr;
    memcpy(write_buf + 1, data, len);
    return i2c_master_transmit(it->second, write_buf, sizeof(write_buf), pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}
