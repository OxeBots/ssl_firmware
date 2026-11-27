#include "RobotState.h"

#include <sdkconfig.h>

#include "NVSManager.h"
#include "RadioAddressing.h"
#include "ssl_robot_protocol_bp.h"

static const char * TAG = "RobotState";

RobotState & RobotState::get_instance()
{
    static RobotState instance;
    return instance;
}

esp_err_t RobotState::init()
{
    int32_t stored_id = 0;
    esp_err_t err = NVSManager::load_i32(NVS_NS, NVS_KEY, &stored_id);

    if (err == ESP_OK && (stored_id >= 0 && stored_id <= ROBOT_ID_MAX))
    {
        m_robot_id = static_cast<uint8_t>(stored_id);
        ESP_LOGI(TAG, "Loaded robot ID from NVS: %d", m_robot_id);
    }
    else
    {
        m_robot_id = 0;
        ESP_LOGW(TAG, "No robot ID in NVS (err=%s) — defaulting to 0.", esp_err_to_name(err));
    }

    int32_t stored_ch = CONFIG_RADIO_CHANNEL;
    err = NVSManager::load_i32(NVS_NS, NVS_KEY_CHANNEL, &stored_ch);
    if (err == ESP_OK && stored_ch >= 0 &&
        radio_addressing::is_valid_channel(static_cast<uint8_t>(stored_ch)))
    {
        m_radio_channel = static_cast<uint8_t>(stored_ch);
        ESP_LOGI(TAG, "Loaded radio RX channel from NVS: %u", m_radio_channel);
    }
    else
    {
        m_radio_channel = CONFIG_RADIO_CHANNEL;
        ESP_LOGI(TAG, "Using default radio RX channel: %u", m_radio_channel);
    }

    int32_t stored_tx_ch =
      (m_radio_channel == radio_addressing::DEFAULT_CHANNEL)
        ? radio_addressing::DEFAULT_RX_CHANNEL
        : m_radio_channel;
    err = NVSManager::load_i32(NVS_NS, NVS_KEY_TX_CHANNEL, &stored_tx_ch);
    if (err == ESP_OK && stored_tx_ch >= 0 &&
        radio_addressing::is_valid_channel(static_cast<uint8_t>(stored_tx_ch)))
    {
        m_radio_tx_channel = static_cast<uint8_t>(stored_tx_ch);
        ESP_LOGI(TAG, "Loaded radio TX (uplink) channel from NVS: %u", m_radio_tx_channel);
    }
    else
    {
        m_radio_tx_channel =
          (m_radio_channel == radio_addressing::DEFAULT_CHANNEL)
            ? radio_addressing::DEFAULT_RX_CHANNEL
            : m_radio_channel;
        ESP_LOGI(TAG, "Radio TX (uplink) channel defaulting to: %u", m_radio_tx_channel);
    }

    return ESP_OK;
}

uint8_t RobotState::get_id() const
{
    return m_robot_id;
}

void RobotState::set_id(uint8_t id)
{
    if (id > ROBOT_ID_MAX && id != ROBOT_ID_BROADCAST)
    {
        ESP_LOGE(TAG,
                 "Invalid robot ID %d (max=%d, broadcast=%d). Rejected.",
                 id,
                 ROBOT_ID_MAX,
                 ROBOT_ID_BROADCAST);
        return;
    }

    m_robot_id = id;
    NVSManager::save_i32(NVS_NS, NVS_KEY, static_cast<int32_t>(id));
    ESP_LOGI(TAG, "Robot ID set to %d (NVS write queued).", id);
}

uint8_t RobotState::get_radio_channel() const
{
    return m_radio_channel;
}

uint8_t RobotState::get_radio_tx_channel() const
{
    return m_radio_tx_channel;
}

void RobotState::set_radio_channel(uint8_t channel)
{
    set_radio_channels(channel, channel);
}

void RobotState::set_radio_channels(uint8_t rx_channel, uint8_t tx_channel)
{
    if (!radio_addressing::is_valid_channel(rx_channel) ||
        !radio_addressing::is_valid_channel(tx_channel))
    {
        ESP_LOGE(TAG,
                 "Invalid radio channels rx=%u tx=%u (max=%u). Rejected.",
                 rx_channel,
                 tx_channel,
                 radio_addressing::MAX_CHANNEL);
        return;
    }

    m_radio_channel = rx_channel;
    m_radio_tx_channel = tx_channel;
    NVSManager::save_i32(NVS_NS, NVS_KEY_CHANNEL, static_cast<int32_t>(rx_channel));
    NVSManager::save_i32(NVS_NS, NVS_KEY_TX_CHANNEL, static_cast<int32_t>(tx_channel));
    ESP_LOGI(TAG,
             "Radio channels set to rx=%u tx=%u (NVS write queued).",
             rx_channel,
             tx_channel);
}

void RobotState::set_error(uint16_t flag)
{
    m_error_flags |= flag;
}

void RobotState::clear_error(uint16_t flag)
{
    m_error_flags &= static_cast<uint16_t>(~flag);
}

uint16_t RobotState::get_error_flags() const
{
    return m_error_flags;
}
