#include "ProtocolHandler.h"

#include <esp_system.h>
#include <esp_timer.h>

#include <cstring>

#include "BL48250.h"
#include "NVSManager.h"
#include "OrientationHandler.h"
#include "PowerMonitor.h"
#include "RobotState.h"
#include "Telemetry.h"
#include "WheelTuning.h"
#include "wheel_state_estimator.h"

static const char * TAG = "ProtocolHandler";

ProtocolHandler & ProtocolHandler::get_instance()
{
    static ProtocolHandler instance;
    return instance;
}

ProtocolHandler::ProtocolHandler() : m_mutex(xSemaphoreCreateMutex())
{
    configASSERT(m_mutex != nullptr);
}

/**
 * @brief Initialize the protocol handler with radio, wheel controller, and kinematics.
 *
 * @param radio Pointer to initialized NRF24L01 radio instance
 * @param wheel_ctrl Pointer to initialized WheelController instance
 * @param kinematics Pointer to initialized OmnidirectionalRobot kinematics instance
 */
void ProtocolHandler::init(NRF24L01 * radio,
                           WheelController * wheel_ctrl,
                           OmnidirectionalRobot * kinematics)
{
    configASSERT(wheel_ctrl != nullptr);
    configASSERT(kinematics != nullptr);

    m_radio = radio;
    m_wheel_ctrl = wheel_ctrl;
    configASSERT(m_command_inbox.init());
    m_velocity_controller.init(*kinematics);
    m_safety_supervisor.mark_self_test_complete(true);
    m_health_state = {};
    m_health_state.severity = HealthSeverity::HEALTHY;
    m_health_state.kick_inhibited = true;

    ESP_LOGI(TAG, "ProtocolHandler initialized.");
}

void ProtocolHandler::force_safe_stop()
{
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    m_safety_supervisor.enter_safe_stop();
    xSemaphoreGive(m_mutex);
    ESP_LOGE(TAG, "SAFE_STOP latched until reboot.");
}

LifecycleState ProtocolHandler::get_lifecycle() const
{
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    const LifecycleState state = m_safety_supervisor.get_state();
    xSemaphoreGive(m_mutex);
    return state;
}

HealthState ProtocolHandler::get_health_state() const
{
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    const HealthState state = m_health_state;
    xSemaphoreGive(m_mutex);
    return state;
}

/**
 * @brief Report whether the NRF24L01 module is connected and healthy.
 *
 * init() only receives a radio whose SPI register read-back succeeded, so a
 * non-null radio pointer means the module answered on the bus at init.
 *
 * @return true when the radio module is initialized.
 */
bool ProtocolHandler::is_radio_ready() const
{
    return m_radio != nullptr;
}

/**
 * @brief Report whether the base station link is currently up.
 *
 * The link counts as up while a COMMAND or CONFIG packet arrived within the
 * link timeout window; a null radio or a silent base station reports false.
 *
 * @return true while base station packets are fresh.
 */
bool ProtocolHandler::is_base_station_connected() const
{
    const uint64_t last_rx_us = m_last_base_station_rx_us.load(std::memory_order_relaxed);
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    return m_radio != nullptr && last_rx_us != 0 &&
           (now_us - last_rx_us) <= BASE_STATION_LINK_TIMEOUT_US;
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

    // COMMAND, CONFIG and TELEMETRY_REQ messages only originate from the base
    // station, so a decoded header of any of these types proves the base
    // station is transmitting, even when the packet targets another robot.
    // Clearing ERROR_COMM_LOSS here (not just in update()) means a telemetry
    // reply reports the live link instead of the stale pre-packet state;
    // update() re-arms the flag when the link goes quiet.
    if (hdr.msg_type == MSG_TYPE_COMMAND || hdr.msg_type == MSG_TYPE_CONFIG ||
        hdr.msg_type == MSG_TYPE_TELEMETRY_REQ)
    {
        m_last_base_station_rx_us.store(static_cast<uint64_t>(esp_timer_get_time()),
                                        std::memory_order_relaxed);
        RobotState::get_instance().clear_error(ERROR_COMM_LOSS);
    }

    switch (hdr.msg_type)
    {
        case MSG_TYPE_COMMAND:
        {
            if (len < BYTES_LENGTH_ROBOT_COMMAND)
            {
                ESP_LOGW(TAG,
                         "MSG_TYPE_COMMAND: too short (%d / %d bytes).",
                         len,
                         BYTES_LENGTH_ROBOT_COMMAND);
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
            update(static_cast<uint64_t>(esp_timer_get_time()));
            break;
        }

        case MSG_TYPE_CONFIG:
        {
            if (len < BYTES_LENGTH_ROBOT_CONFIG)
            {
                ESP_LOGW(TAG,
                         "MSG_TYPE_CONFIG: too short (%d / %d bytes).",
                         len,
                         BYTES_LENGTH_ROBOT_CONFIG);
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

        case MSG_TYPE_TELEMETRY_REQ:
        {
            if (len < BYTES_LENGTH_ROBOT_TELEMETRY_REQUEST)
            {
                ESP_LOGW(TAG,
                         "MSG_TYPE_TELEMETRY_REQ: too short (%d / %d bytes).",
                         len,
                         BYTES_LENGTH_ROBOT_TELEMETRY_REQUEST);
                return;
            }

            const uint8_t my_id = RobotState::get_instance().get_id();

            // Telemetry requests are strictly unicast to avoid radio collisions
            // from multiple robots replying simultaneously.
            if (hdr.robot_id != my_id)
            {
                ESP_LOGD(
                  TAG, "Telemetry request ignored — for robot %d, we are %d.", hdr.robot_id, my_id);
                return;
            }

            RobotTelemetryRequest telreq;
            memset(&telreq, 0, sizeof(RobotTelemetryRequest));
            DecodeRobotTelemetryRequest(&telreq, const_cast<uint8_t *>(payload));
            handle_telemetry_req(telreq);
            break;
        }

        default:
            ESP_LOGW(TAG, "Unknown msg_type=0x%02X — discarding.", hdr.msg_type);
            break;
    }
}

/**
 * @brief Handle RobotCommand — enqueue target pose and velocity into the command inbox.
 *
 * RobotCommand is strictly fire-and-forget; telemetry replies are only sent
 * when an explicit RobotTelemetryRequest is received.
 *
 * @param cmd Decoded RobotCommand message
 */
void ProtocolHandler::handle_command(const RobotCommand & cmd)
{
    ESP_LOGI(TAG,
             "CMD robot=%d ts=%lu x=%d y=%d vx=%d vy=%d w=%d kick=%d",
             cmd.header.robot_id,
             (unsigned long)cmd.header.timestamp,
             cmd.target_pose.x,
             cmd.target_pose.y,
             cmd.target_pose.x_v,
             cmd.target_pose.y_v,
             cmd.target_pose.angular_vel,
             cmd.kick_velocity);

    if (!m_command_inbox.push(cmd, static_cast<uint64_t>(esp_timer_get_time())))
    {
        ESP_LOGW(TAG, "Command inbox full — command was not accepted.");
    }
}

/**
 * @brief Consume queued commands and publish the current safe wheel target.
 */
void ProtocolHandler::update(uint64_t now_us)
{
    if (m_wheel_ctrl == nullptr)
        return;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    // Classify the live observations into the health state the supervisor
    // and the status consumers act on. The supply status comes from the
    // continuously sampled VMON channel; its electrical faults are critical
    // and therefore latch SAFE_STOP below.
    HealthObservationFrame observation = {};
    PowerMonitor::get_instance().fill_observation(&observation);
    const uint16_t error_flags = RobotState::get_instance().get_error_flags();
    observation.imu_available = (error_flags & ERROR_IMU_FAIL) == 0U;
    observation.tof_available = (error_flags & ERROR_TOF_FAIL) == 0U;
    // Kicker sensing does not exist on this robot yet: treat the kicker as
    // ready so only real observations inhibit.
    observation.kicker_ready = true;
    auto & estimator = WheelStateEstimator::get_instance();
    for (size_t wheel = 0; wheel < ROBOT_WHEEL_COUNT; ++wheel)
    {
        if (estimator.is_channel_calibrated(wheel))
            observation.wheel_valid_mask |= static_cast<uint8_t>(1U << wheel);
    }
    m_health_state = m_health_monitor.update(observation);

    // Mirror the battery faults into the robot error flags so the telemetry
    // frame and the dashboard error list report them.
    RobotState & robot_state = RobotState::get_instance();
    if ((m_health_state.active_faults & ERROR_BATT_UNDERVOLT) != 0U)
        robot_state.set_error(ERROR_BATT_UNDERVOLT);
    else
        robot_state.clear_error(ERROR_BATT_UNDERVOLT);
    if ((m_health_state.active_faults & ERROR_BATT_OVERVOLT) != 0U)
        robot_state.set_error(ERROR_BATT_OVERVOLT);
    else
        robot_state.clear_error(ERROR_BATT_OVERVOLT);

    // Keep ERROR_COMM_LOSS in step with the live link so telemetry carries
    // the protocol meaning of bit 0: communication loss with the base station.
    if (is_base_station_connected())
        RobotState::get_instance().clear_error(ERROR_COMM_LOSS);
    else
        RobotState::get_instance().set_error(ERROR_COMM_LOSS);

    CommandEnvelope envelope = {};
    while (m_command_inbox.pop(&envelope))
    {
        m_latest_command = envelope;
        m_has_latest_command = true;
    }

    const CommandEnvelope * command = m_has_latest_command ? &m_latest_command : nullptr;
    const SafetyDecision decision = m_safety_supervisor.update(now_us, command, m_health_state, {});
    RobotStateEstimate state = {};
    state.health = m_health_state;
    const BodyVelocity body_target = m_position_controller.compute(state, decision.goal);
    const ActuationSetpoint setpoint =
      m_velocity_controller.compute(body_target, decision.goal, state.health);
    // Until the first radio command arrives the wheel targets belong to the
    // bench dashboard (SerialDebug); publishing the zero setpoint here would
    // overwrite the dashboard targets every cycle. Once the radio path has
    // delivered a command, this loop owns the targets, including the zero
    // setpoint after a command timeout.
    if (m_has_latest_command)
        m_wheel_ctrl->set_target_velocities(setpoint.wheel_velocity_rad_s);

    xSemaphoreGive(m_mutex);
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
            ESP_LOGE(TAG,
                     "CONFIG_FLAG_SET_ID: ID %d exceeds ROBOT_ID_MAX (%d). Rejected.",
                     new_id,
                     ROBOT_ID_MAX);
        }
        else
        {
            ESP_LOGI(TAG, "Robot ID: %d → %d", my_id, new_id);
            state.set_id(new_id);
        }
    }

    // RUN calibration routines
    if (cfg.config_flags & CONFIG_FLAG_RUN_CALIBRATION)
    {
        ESP_LOGI(TAG,
                 "CONFIG_FLAG_RUN_CALIBRATION: triggering sensor + wheel control calibration.");

        WheelTuning & tuning = WheelTuning::get_instance();
        if (tuning.is_running())
        {
            ESP_LOGW(TAG, "Wheel control calibration already active — ignoring request.");
        }
        else if (tuning.start() != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to start the wheel control calibration.");
        }
    }

    // CALIBRATE magnetometer only
    if (cfg.config_flags & CONFIG_FLAG_CALIBRATE_MAG)
    {
        ESP_LOGI(TAG, "CONFIG_FLAG_CALIBRATE_MAG: triggering magnetometer calibration.");
        OrientationHandler::get_instance().calibrate_mag(10);
    }

    // RESET NVS — reset robot ID to 0 (legacy, superseded by FACTORY_RESET)
    if (cfg.config_flags & CONFIG_FLAG_RESET_NVS)
    {
        ESP_LOGW(TAG, "CONFIG_FLAG_RESET_NVS: resetting robot ID to 0.");
        state.set_id(0);
    }

    // FULL FACTORY RESET — erase all NVS namespaces and keys
    if (cfg.config_flags & CONFIG_FLAG_FACTORY_RESET)
    {
        ESP_LOGW(TAG, "CONFIG_FLAG_FACTORY_RESET: erasing all NVS data.");
        NVSManager::erase_all();
        ESP_LOGI(TAG, "NVS erased. Rebooting to apply factory reset.");
        esp_restart();
    }
}

/**
 * @brief Handle RobotTelemetryRequest — send RobotTelemetry back to the base station.
 *
 * Echoes the request timestamp so the base station can measure round-trip time.
 *
 * @param telreq Decoded RobotTelemetryRequest message
 */
void ProtocolHandler::handle_telemetry_req(const RobotTelemetryRequest & telreq)
{
    ESP_LOGI(TAG, "Answering telemetry request, robot: %d", telreq.header.robot_id);

    Telemetry::get_instance().send(telreq.header.timestamp);
}
