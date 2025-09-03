#include "driver/as5600_i2c.hpp"

#include <cstring>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char * TAG = "AS5600_I2C";
static constexpr int I2C_TIMEOUT_MS = 100;
static constexpr uint8_t AS5600_ADDR = 0x36;

AS5600_I2C::AS5600_I2C()
{
}

AS5600_I2C::~AS5600_I2C()
{
    if (m_initialized)
    {
        i2c_master_bus_rm_device(m_dev_handle);
        i2c_del_master_bus(m_bus_handle);
    }
}

esp_err_t AS5600_I2C::init(i2c_port_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin)
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

    i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = AS5600_ADDR,
      .scl_speed_hz = 100000,  // 100 kHz
    };
    ret = i2c_master_bus_add_device(m_bus_handle, &dev_cfg, &m_dev_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add AS5600_I2C device: %s", esp_err_to_name(ret));
        i2c_del_master_bus(m_bus_handle);
        return ret;
    }

    // Test communication by reading a register
    uint16_t test_conf;
    ret = read_config_register(&test_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to communicate with AS5600_I2C sensor.");
        i2c_master_bus_rm_device(m_dev_handle);
        i2c_del_master_bus(m_bus_handle);
        return ret;
    }

    m_initialized = true;
    ESP_LOGI(TAG, "AS5600_I2C driver initialized successfully.");
    return ESP_OK;
}

esp_err_t AS5600_I2C::set_output_stage(OutputStage stage)
{
    if (!m_initialized)
        return ESP_ERR_INVALID_STATE;
    uint16_t config;
    esp_err_t ret = read_config_register(&config);
    if (ret != ESP_OK)
        return ret;

    config &= ~0x0030;  // Clear bits 4 and 5
    config |= (static_cast<uint8_t>(stage) << 4);

    return write_config_register(config);
}

esp_err_t AS5600_I2C::set_slow_filter(SlowFilter filter)
{
    if (!m_initialized)
        return ESP_ERR_INVALID_STATE;
    uint16_t config;
    esp_err_t ret = read_config_register(&config);
    if (ret != ESP_OK)
        return ret;

    config &= ~0x0300;  // Clear bits 8 and 9
    config |= (static_cast<uint8_t>(filter) << 8);

    return write_config_register(config);
}

esp_err_t AS5600_I2C::set_fast_filter(FastFilter threshold)
{
    if (!m_initialized)
        return ESP_ERR_INVALID_STATE;
    uint16_t config;
    esp_err_t ret = read_config_register(&config);
    if (ret != ESP_OK)
        return ret;

    config &= ~0x1C00;  // Clear bits 10, 11, and 12
    config |= (static_cast<uint8_t>(threshold) << 10);

    return write_config_register(config);
}

esp_err_t AS5600_I2C::burn_settings()
{
    if (!m_initialized)
        return ESP_ERR_INVALID_STATE;
    ESP_LOGW(TAG, "Burning settings to AS5600_I2C OTP memory. This is permanent!");
    uint8_t cmd = static_cast<uint8_t>(Cmd::BURN);
    esp_err_t ret = write_register(Register::BURN, &cmd, 1);
    vTaskDelay(pdMS_TO_TICKS(10));  // Wait for burn to complete
    return ret;
}

// --- Private I2C Helper Methods ---

esp_err_t AS5600_I2C::read_config_register(uint16_t * config)
{
    uint8_t buffer[2];
    esp_err_t ret = read_register(Register::CONF_H, buffer, 2);

    if (ret == ESP_OK)
        *config = (buffer[0] << 8) | buffer[1];

    return ret;
}

esp_err_t AS5600_I2C::write_config_register(uint16_t config)
{
    uint8_t buffer[2];
    buffer[0] = (config >> 8) & 0xFF;
    buffer[1] = config & 0xFF;
    return write_register(Register::CONF_H, buffer, 2);
}

esp_err_t AS5600_I2C::read_register(Register reg_addr, uint8_t * data, size_t len)
{
    uint8_t reg = static_cast<uint8_t>(reg_addr);
    return i2c_master_transmit_receive(m_dev_handle, &reg, 1, data, len, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

esp_err_t AS5600_I2C::write_register(Register reg_addr, uint8_t * data, size_t len)
{
    uint8_t write_buf[len + 1];
    write_buf[0] = static_cast<uint8_t>(reg_addr);
    memcpy(write_buf + 1, data, len);
    return i2c_master_transmit(m_dev_handle, write_buf, sizeof(write_buf), pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}
