/**
 * @file CD4052B.cpp
 * @brief CD4052B analog switch control through the SN74HC595.
 */

#include "CD4052B.h"

#include <esp_log.h>

static const char * TAG = "CD4052B";

static bool valid_bit(uint8_t bit)
{
    return bit < 8;
}

esp_err_t CD4052B::init(SN74HC595 * shift_register, const Config & config)
{
    if (shift_register == nullptr || !shift_register->is_initialized() ||
        !valid_bit(config.select_a_bit) || !valid_bit(config.select_b_bit) ||
        !valid_bit(config.inhibit_bit) || config.select_a_bit == config.select_b_bit ||
        config.select_a_bit == config.inhibit_bit || config.select_b_bit == config.inhibit_bit)
        return ESP_ERR_INVALID_ARG;

    m_shift_register = shift_register;
    m_config = config;
    m_selected_channel = 0;
    m_enabled = false;
    return disable();
}

esp_err_t CD4052B::select(uint8_t channel)
{
    if (m_shift_register == nullptr)
        return ESP_ERR_INVALID_STATE;
    if (channel >= 4)
        return ESP_ERR_INVALID_ARG;

    const uint8_t select_mask = static_cast<uint8_t>(
      (1U << m_config.select_a_bit) | (1U << m_config.select_b_bit) | (1U << m_config.inhibit_bit));
    uint8_t value = m_shift_register->get_output() & ~select_mask;
    if ((channel & 0x01U) != 0)
        value |= static_cast<uint8_t>(1U << m_config.select_a_bit);
    if ((channel & 0x02U) != 0)
        value |= static_cast<uint8_t>(1U << m_config.select_b_bit);

    // The CD4052B inhibit input is active high. Include it in the same latch
    // update so the selected AS5600 is never exposed during address changes.
    esp_err_t err = m_shift_register->write(value);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to select encoder channel %u", channel);
        return err;
    }

    m_selected_channel = channel;
    m_enabled = true;
    return ESP_OK;
}

esp_err_t CD4052B::disable()
{
    if (m_shift_register == nullptr)
        return ESP_ERR_INVALID_STATE;

    const uint8_t inhibit = static_cast<uint8_t>(1U << m_config.inhibit_bit);
    esp_err_t err = m_shift_register->update_mask(inhibit, inhibit);
    if (err == ESP_OK)
        m_enabled = false;
    return err;
}

uint8_t CD4052B::selected_channel() const
{
    return m_selected_channel;
}

bool CD4052B::is_enabled() const
{
    return m_enabled;
}
