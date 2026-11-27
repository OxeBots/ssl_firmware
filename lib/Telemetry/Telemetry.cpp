#include "Telemetry.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>

#include "ESP_NOW_Dev.h"
#include "OrientationHandler.h"
#include "RobotState.h"
#include "ssl_robot_protocol_bp.h"
#include "kicker_controller.h"
#include "PowerMonitor.h"

static const char * TAG = "Telemetry";

Telemetry & Telemetry::get_instance()
{
    static Telemetry instance;
    return instance;
}

/**
 * @brief Initialize telemetry service with radio instance.
 *
 * @param radio Pointer to initialized ESP-NOW link
 */
void Telemetry::init(ESP_NOW_Dev * radio)
{
    configASSERT(radio != nullptr);
    m_radio = radio;
    ESP_LOGI(TAG, "Telemetry initialized.");
}

/**
 * @brief Send robot telemetry frame to base station.
 *
 * Encodes RobotTelemetry message with current robot ID, yaw from IMU,
 * battery and kicker voltage values, and error flags.
 *
 * @param echo_timestamp Timestamp from the telemetry request
 * @param destination Address of the gateway that requested telemetry
 */
void Telemetry::send(uint32_t echo_timestamp, const uint8_t * destination)
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

    // conversão da tensão da bat em %
    const PowerMonitor & power_mon = PowerMonitor::get_instance();
    uint8_t bat_per = 0;
    if(power_mon.is_valid()){
        const float bat_empty = 19.8f; // CONFIG_VMON_UNDERVOLTAGE_MV = 19800
        const float bat_max = 25.2f; // CONFIG_VMON_OVERVOLTAGE_MV = 26500
        const float per = (power_mon.get_supply_voltage_v() - bat_empty ) / (bat_max - bat_empty) * 100.0f;
        bat_per = static_cast<uint8_t>(std::lround(std::clamp(per, 0.0f, 100.0f)));
    }


    tel.battery_percentage = bat_per;
    tel.kicker_voltage = static_cast<uint16_t>(
      static_cast<uint16_t>(KickerController::get_instance().get_voltage()) * 100U);
    tel.error_flags = state.get_error_flags();

    uint8_t buf[BYTES_LENGTH_ROBOT_TELEMETRY] = {0};
    EncodeRobotTelemetry(&tel, buf);

    const esp_err_t err = m_radio->send(destination, buf, sizeof(buf));
    if (err != ESP_OK)
        ESP_LOGW(TAG, "ESP-NOW send failed: %s", esp_err_to_name(err));
}
