#include "ProtocolHandler.h"

#include <algorithm>
#include <cstring>

#include "OrientationHandler.h"
#include "RobotState.h"
#include "TelemetryService.h"
#include "wheel_state_estimator.h"

static const char * TAG = "ProtocolHandler";

ProtocolHandler & ProtocolHandler::get_instance()
{
    static ProtocolHandler instance;
    return instance;
}

/**
 * @brief Initialize the protocol handler with radio, wheel controller, and kinematics.
 *
 * @param radio Pointer to initialized NRF24L01 radio instance
 * @param wheel_ctrl Pointer to initialized WheelController instance
 * @param kinematics Pointer to initialized OmnidirectionalRobot kinematics instance
 */
void ProtocolHandler::init(NRF24L01 * radio, WheelController * wheel_ctrl, OmnidirectionalRobot * kinematics)
{
    configASSERT(wheel_ctrl != nullptr);
    configASSERT(kinematics != nullptr);

    m_radio = radio;
    m_wheel_ctrl = wheel_ctrl;
    m_kinematics = kinematics;

    ESP_LOGI(TAG, "ProtocolHandler initialized.");
}

/**
 * @brief Process incoming radio packet — decode header and dispatch to handler.
 *
 * Validates packet length, decodes header, and routes to handle_command() or
 * handle_config() based on message type. Commands addressed to other robots
 * (or broadcast) are filtered. Telemetry replies from other robots are ignored.
 *
 * @param payload Raw packet bytes
 * @param len Packet length in bytes
 */
void ProtocolHandler::on_packet(const uint8_t * payload, uint8_t len)
{
    if (len < BYTES_LENGTH_HEADER)
    {
        ESP_LOGW(TAG, "Packet too short for header (%d bytes). Discarding.", len);
        return;
    }

    // IMPORTANT: bitproto decoders use bitwise OR — always decode into a
    // zero-initialised struct to prevent stale bits bleeding into fields.
    Header hdr;
    memset(&hdr, 0, sizeof(Header));
    DecodeHeader(&hdr, const_cast<uint8_t *>(payload));

    switch (hdr.msg_type)
    {
        case MSG_TYPE_COMMAND:
        {
            if (len < BYTES_LENGTH_ROBOT_COMMAND)
            {
                ESP_LOGW(TAG, "MSG_TYPE_COMMAND: too short (%d / %d bytes).", len, BYTES_LENGTH_ROBOT_COMMAND);
                return;
            }

            const uint8_t my_id = RobotState::get_instance().get_id();

            if (hdr.robot_id != my_id && hdr.robot_id != ROBOT_ID_BROADCAST)
            {
                ESP_LOGD(TAG, "Command ignored — for robot %d, we are %d.", hdr.robot_id, my_id);
                return;
            }

            RobotCommand cmd;
            memset(&cmd, 0, sizeof(RobotCommand));
            DecodeRobotCommand(&cmd, const_cast<uint8_t *>(payload));
            handle_command(cmd);
            break;
        }

        case MSG_TYPE_CONFIG:
        {
            if (len < BYTES_LENGTH_ROBOT_CONFIG)
            {
                ESP_LOGW(TAG, "MSG_TYPE_CONFIG: too short (%d / %d bytes).", len, BYTES_LENGTH_ROBOT_CONFIG);
                return;
            }

            RobotConfig cfg;
            memset(&cfg, 0, sizeof(RobotConfig));
            DecodeRobotConfig(&cfg, const_cast<uint8_t *>(payload));
            handle_config(cfg);
            break;
        }

        case MSG_TYPE_TELEMETRY:
            // Another robot's telemetry reply on the shared channel — ignore.
            ESP_LOGD(TAG, "Received telemetry from robot %d — ignoring.", hdr.robot_id);
            break;

        default:
            ESP_LOGW(TAG, "Unknown msg_type=0x%02X — discarding.", hdr.msg_type);
            break;
    }
}

/**
 * @brief Handle RobotCommand — convert body-frame velocities to wheel speeds.
 *
 * Extracts target velocities from command, converts mm/s and 0.01 rad/s units
 * to m/s and rad/s, computes wheel velocities via inverse kinematics, and
 * sets wheel controller targets. For non-broadcast commands, sends telemetry
 * reply with echo timestamp.
 *
 * @param cmd Decoded RobotCommand message
 */
void ProtocolHandler::handle_command(const RobotCommand & cmd)
{
    ESP_LOGI(TAG, "CMD robot=%d ts=%lu vx=%d vy=%d w=%d kick=%d", cmd.header.robot_id,
             (unsigned long)cmd.header.timestamp, cmd.target_pose.x_v, cmd.target_pose.y_v, cmd.target_pose.angular_vel,
             cmd.kick_velocity);

    // Convert body-frame velocities to wheel velocities via omni kinematics.
    vt::numeric_vector<3> body_vel;
    body_vel(0) = cmd.target_pose.x_v / 1000.0f;         // mm/s  → m/s
    body_vel(1) = cmd.target_pose.y_v / 1000.0f;         // mm/s  → m/s
    body_vel(2) = cmd.target_pose.angular_vel / 100.0f;  // 0.01 rad/s → rad/s

    vt::numeric_vector<4> wheel_vels = m_kinematics->compute_wheel_velocities(body_vel);

    std::array<float, 4> wheel_arr = {
      static_cast<float>(wheel_vels(0)),
      static_cast<float>(wheel_vels(1)),
      static_cast<float>(wheel_vels(2)),
      static_cast<float>(wheel_vels(3)),
    };
    m_wheel_ctrl->set_target_velocities(wheel_arr);

    // Broadcast commands: execute without reply.
    if (cmd.header.robot_id == ROBOT_ID_BROADCAST)
        return;

    TelemetryService::get_instance().send(cmd.header.timestamp);
}

/**
 * @brief Handle RobotConfig — process ID changes, calibration triggers, NVS resets.
 *
 * Validates target robot ID (filters if not for us or broadcast). Processes:
 * - CONFIG_FLAG_SET_ID: Set robot ID (validates against ROBOT_ID_MAX)
 * - CONFIG_FLAG_RUN_CALIBRATION: Run interactive sensor calibration routines
 * - CONFIG_FLAG_RESET_NVS: Reset robot ID to 0 (TODO: full NVS erase)
 *
 * @param cfg Decoded RobotConfig message
 */
void ProtocolHandler::handle_config(const RobotConfig & cfg)
{
    RobotState & state = RobotState::get_instance();
    const uint8_t my_id = state.get_id();
    const uint8_t target = cfg.header.robot_id;

    if (target != my_id && target != ROBOT_ID_BROADCAST)
    {
        ESP_LOGD(TAG, "Config ignored — for robot %d, we are %d.", target, my_id);
        return;
    }

    ESP_LOGI(TAG, "Handling config flags=0x%02X param=%d", cfg.config_flags, cfg.param);

    // SET robot ID
    if (cfg.config_flags & CONFIG_FLAG_SET_ID)
    {
        const uint8_t new_id = static_cast<uint8_t>(cfg.param & 0xFF);

        if (new_id > ROBOT_ID_MAX)
        {
            ESP_LOGE(TAG, "CONFIG_FLAG_SET_ID: ID %d exceeds ROBOT_ID_MAX (%d). Rejected.", new_id, ROBOT_ID_MAX);
        }
        else
        {
            ESP_LOGI(TAG, "Robot ID: %d → %d", my_id, new_id);
            state.set_id(new_id);
        }
    }

    // RUN calibration routines (blocking)
    if (cfg.config_flags & CONFIG_FLAG_RUN_CALIBRATION)
    {
        ESP_LOGI(TAG, "CONFIG_FLAG_RUN_CALIBRATION: triggering sensor calibration.");
        // Calibrate IMU sensors (blocking, runs interactive routine)
        OrientationHandler::get_instance().calibrate_mag(10);
        OrientationHandler::get_instance().calibrate_accel(10);
        OrientationHandler::get_instance().calibrate_gyro(10);
        // Calibrate wheel encoders
        WheelStateEstimator::get_instance().calibrate_encoders(5000, true);
    }

    // RESET NVS (e.g. on factory reset) — currently only resets robot ID, but should ideally erase all keys.
    if (cfg.config_flags & CONFIG_FLAG_RESET_NVS)
    {
        ESP_LOGW(TAG, "CONFIG_FLAG_RESET_NVS: resetting robot ID to 0.");
        state.set_id(0);
        // TODO: erase all NVS keys, not just robot_id.
        ESP_LOGW(TAG, "NOTE: Full NVS erase not yet implemented. Only robot_id was reset.");
    }
}
