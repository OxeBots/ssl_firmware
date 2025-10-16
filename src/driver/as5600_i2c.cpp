#include "driver/as5600_i2c.hpp"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char * TAG = "AS5600_I2C";

AS5600_I2C::AS5600_I2C()
{
}

esp_err_t AS5600_I2C::init(I2C_Wrapper & i2c_wrapper)
{
    if (m_initialized)
        return ESP_ERR_INVALID_STATE;

    if (!i2c_wrapper.is_initialized())
    {
        ESP_LOGE(TAG, "I2C_Wrapper not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    m_i2c_wrapper = &i2c_wrapper;

    // Add AS5600 device to the bus
    esp_err_t ret = m_i2c_wrapper->add_device(AS5600_ADDR);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add AS5600 device to I2C bus");
        return ret;
    }

    // Test communication by reading a register
    uint16_t test_conf;
    ret = read_config_register(&test_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to communicate with AS5600 sensor.");
        return ret;
    }

    m_initialized = true;
    ESP_LOGI(TAG, "AS5600 driver initialized successfully.");
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

    ESP_LOGW(TAG, "Burning settings to AS5600 OTP memory. This is permanent!");
    uint8_t cmd = static_cast<uint8_t>(Cmd::BURN);
    esp_err_t ret = write_register(Register::BURN, &cmd, 1);
    vTaskDelay(pdMS_TO_TICKS(10));  // Wait for burn to complete
    return ret;
}

// --- Private Methods ---

esp_err_t AS5600_I2C::read_config_register(uint16_t * config)
{
    uint8_t buffer[2];
    esp_err_t ret = m_i2c_wrapper->read_register(AS5600_ADDR, static_cast<uint8_t>(Register::CONF_H), buffer, 2);

    if (ret == ESP_OK)
        *config = (buffer[0] << 8) | buffer[1];

    return ret;
}

esp_err_t AS5600_I2C::write_config_register(uint16_t config)
{
    uint8_t buffer[2];
    buffer[0] = (config >> 8) & 0xFF;
    buffer[1] = config & 0xFF;
    return m_i2c_wrapper->write_register(AS5600_ADDR, static_cast<uint8_t>(Register::CONF_H), buffer, 2);
}
