#include "Telemetry.h"

#include <cstring>

#include "OrientationHandler.h"
#include "RobotState.h"
#include "ssl_robot_protocol_bp.h"

static const char * TAG = "Telemetry";

Telemetry & Telemetry::get_instance()
{
    static Telemetry instance;
    return instance;
}

/**
 * @brief Initialize telemetry service with radio instance.
 *
 * @param radio Pointer to initialized NRF24L01 radio instance
 */
void Telemetry::init(NRF24L01 * radio)
{
    configASSERT(radio != nullptr);
    m_radio = radio;
    ESP_LOGI(TAG, "Telemetry initialized.");
}

/**
 * @brief Send robot telemetry frame to base station.
 *
 * Encodes RobotTelemetry message with current robot ID, yaw from IMU,
 * battery/kicker stub values, and error flags. Sends via NRF24L01 radio.
 *
 * @param echo_timestamp Timestamp from command to echo in telemetry reply
 */
void Telemetry::send(uint32_t echo_timestamp)
{
    if (m_radio == nullptr)
    {
        ESP_LOGE(TAG, "send() called before init() — dropping telemetry frame.");
        return;
    }

    // Always zero-initialize before encoding (bitproto uses bitwise OR).
    RobotTelemetry tel;
    memset(&tel, 0, sizeof(RobotTelemetry));

    const RobotState & state = RobotState::get_instance();

    tel.header.msg_type = MSG_TYPE_TELEMETRY;
    tel.header.robot_id = state.get_id();
    tel.header.timestamp = echo_timestamp;

    // Pose: x and y are unknown without position tracking — send 0.
    // Yaw is provided by the IMU (scaled: 1 unit = 0.01°).
    tel.robot_pose.x = 0;
    tel.robot_pose.y = 0;
    tel.robot_pose.angle =
      static_cast<int16_t>(OrientationHandler::get_instance().get_yaw() * 100.0);

    tel.battery_percentage = STUB_BATTERY_PCT;
    tel.kicker_voltage = STUB_KICKER_VOLTAGE;
    tel.error_flags = state.get_error_flags();

    uint8_t buf[BYTES_LENGTH_ROBOT_TELEMETRY] = {0};
    EncodeRobotTelemetry(&tel, buf);

    esp_err_t err = m_radio->send_raw(buf, BYTES_LENGTH_ROBOT_TELEMETRY);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "send_raw failed: %s", esp_err_to_name(err));
}
