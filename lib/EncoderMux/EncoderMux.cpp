/**
 * @file EncoderMux.cpp
 * @brief 74HC4052 selector control through the 74HC595.
 */

#include "EncoderMux.h"

#include <esp_log.h>

static const char * TAG = "EncoderMux";

static bool valid_bit(uint8_t bit)
{
    return bit < 8;
}

esp_err_t EncoderMux::init(DigitalOutputExpander * output_expander, const Config & config)
{
    if (output_expander == nullptr || !output_expander->is_initialized() ||
        !valid_bit(config.select_a_bit) || !valid_bit(config.select_b_bit) ||
        !valid_bit(config.inhibit_bit) || config.select_a_bit == config.select_b_bit ||
        config.select_a_bit == config.inhibit_bit || config.select_b_bit == config.inhibit_bit)
        return ESP_ERR_INVALID_ARG;

    m_output_expander = output_expander;
    m_config = config;
    m_selected_channel = 0;
    m_enabled = false;
    return disable();
}

esp_err_t EncoderMux::select(uint8_t channel)
{
    if (m_output_expander == nullptr)
        return ESP_ERR_INVALID_STATE;
    if (channel >= 4)
        return ESP_ERR_INVALID_ARG;

    const uint8_t select_mask = static_cast<uint8_t>(
      (1U << m_config.select_a_bit) | (1U << m_config.select_b_bit) | (1U << m_config.inhibit_bit));
    uint8_t value = m_output_expander->get_output() & ~select_mask;
    if ((channel & 0x01U) != 0)
        value |= static_cast<uint8_t>(1U << m_config.select_a_bit);
    if ((channel & 0x02U) != 0)
        value |= static_cast<uint8_t>(1U << m_config.select_b_bit);

    // The 4052 inhibit input is active high. Include it in the same latch
    // update so the selected AS5600 is never exposed during address changes.
    esp_err_t err = m_output_expander->write(value);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to select encoder channel %u", channel);
        return err;
    }

    m_selected_channel = channel;
    m_enabled = true;
    return ESP_OK;
}

esp_err_t EncoderMux::disable()
{
    if (m_output_expander == nullptr)
        return ESP_ERR_INVALID_STATE;

    const uint8_t inhibit = static_cast<uint8_t>(1U << m_config.inhibit_bit);
    esp_err_t err = m_output_expander->update_mask(inhibit, inhibit);
    if (err == ESP_OK)
        m_enabled = false;
    return err;
}

uint8_t EncoderMux::selected_channel() const
{
    return m_selected_channel;
}

bool EncoderMux::is_enabled() const
{
    return m_enabled;
}
