#include "RobotState.h"

#include "NVSManager.h"
#include "ssl_robot_protocol_bp.h"  // for ROBOT_ID_MAX

static const char * TAG = "RobotState";

RobotState & RobotState::get_instance()
{
    static RobotState instance;
    return instance;
}

/**
 * @brief Initialize robot state — load robot ID from NVS.
 *
 * Attempts to load stored robot ID from NVS. If not found or error,
 * defaults to 0. Always returns ESP_OK (missing key is not an error).
 *
 * @return ESP_OK on success
 */
esp_err_t RobotState::init()
{
    int32_t stored_id = 0;
    esp_err_t err = NVSManager::load_i32(NVS_NS, NVS_KEY, &stored_id);

    if (err == ESP_OK)
    {
        m_robot_id = static_cast<uint8_t>(stored_id);
        ESP_LOGI(TAG, "Loaded robot ID from NVS: %d", m_robot_id);
    }
    else
    {
        m_robot_id = 0;
        ESP_LOGW(TAG, "No robot ID in NVS (err=%s) — defaulting to 0.", esp_err_to_name(err));
    }

    return ESP_OK;  // Always succeed; missing key is not an error.
}

/**
 * @brief Get current robot ID.
 * @return Robot ID (0-10 for valid robots, 15 for broadcast)
 */
uint8_t RobotState::get_id() const
{
    return m_robot_id;
}

/**
 * @brief Set robot ID and persist to NVS.
 *
 * Validates ID is within valid range (0-10 or 15 for broadcast).
 * Invalid IDs are rejected and not written to NVS.
 *
 * @param id Robot ID to set
 */
void RobotState::set_id(uint8_t id)
{
    if (id > ROBOT_ID_MAX && id != ROBOT_ID_BROADCAST)
    {
        ESP_LOGE(TAG, "Invalid robot ID %d (max=%d, broadcast=%d). Rejected.", id, ROBOT_ID_MAX, ROBOT_ID_BROADCAST);
        return;
    }

    m_robot_id = id;
    NVSManager::save_i32(NVS_NS, NVS_KEY, static_cast<int32_t>(id));
    ESP_LOGI(TAG, "Robot ID set to %d (NVS write queued).", id);
}

/**
 * @brief Set error flag in error_flags bitmask.
 * @param flag Error flag bit to set
 */
void RobotState::set_error(uint16_t flag)
{
    m_error_flags |= flag;
}

/**
 * @brief Clear error flag from error_flags bitmask.
 * @param flag Error flag bit to clear
 */
void RobotState::clear_error(uint16_t flag)
{
    m_error_flags &= static_cast<uint16_t>(~flag);
}

/**
 * @brief Get current error flags bitmask.
 * @return 16-bit error flags bitmask
 */
uint16_t RobotState::get_error_flags() const
{
    return m_error_flags;
}
