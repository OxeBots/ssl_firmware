/**
 * @file DigitalOutputExpander.cpp
 * @brief Driver for the SPI-connected 74HC595 output shift register.
 */

#include "DigitalOutputExpander.h"

#include <driver/spi_master.h>
#include <esp_log.h>

#include "SPIdev.h"

static const char * TAG = "DigitalOutputExpander";

esp_err_t DigitalOutputExpander::init(const Config & config, uint8_t initial_value)
{
    if (m_initialized)
        return ESP_ERR_INVALID_STATE;

    if (config.latch_gpio < 0 || config.latch_gpio >= GPIO_NUM_MAX)
        return ESP_ERR_INVALID_ARG;

    spi_device_interface_config_t device_config = {};
    device_config.clock_speed_hz = static_cast<int>(config.clock_hz);
    device_config.mode = 0;
    device_config.spics_io_num = -1;
    device_config.queue_size = 1;
    device_config.flags = SPI_DEVICE_NO_DUMMY;

    if (!SPIdev::register_device(config.device_key, &device_config))
    {
        ESP_LOGE(TAG, "Failed to register 74HC595 on SPI device key %u", config.device_key);
        return ESP_FAIL;
    }

    gpio_reset_pin(config.latch_gpio);
    esp_err_t err = gpio_set_direction(config.latch_gpio, GPIO_MODE_OUTPUT);
    if (err != ESP_OK)
        return err;

    err = gpio_set_level(config.latch_gpio, 0);
    if (err != ESP_OK)
        return err;

    m_config = config;
    m_initialized = true;
    return write(initial_value);
}

esp_err_t DigitalOutputExpander::write(uint8_t value)
{
    if (!m_initialized)
        return ESP_ERR_INVALID_STATE;

    if (!SPIdev::write(m_config.device_key, &value, 1))
        return ESP_FAIL;

    // The storage register is updated on the rising latch edge. Keep the
    // pulse short so motor direction and mux selection change atomically.
    esp_err_t err = gpio_set_level(m_config.latch_gpio, 1);
    if (err != ESP_OK)
        return err;
    err = gpio_set_level(m_config.latch_gpio, 0);
    if (err == ESP_OK)
        m_output = value;
    return err;
}

esp_err_t DigitalOutputExpander::update_mask(uint8_t mask, uint8_t value)
{
    const uint8_t next = static_cast<uint8_t>((m_output & ~mask) | (value & mask));
    return write(next);
}

uint8_t DigitalOutputExpander::get_output() const
{
    return m_output;
}

bool DigitalOutputExpander::is_initialized() const
{
    return m_initialized;
}
