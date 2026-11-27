#include "ProtocolHandler.h"

#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>

#include <cstring>

#include "ESP_NOW_Dev.h"
#include "MotorCalibration.h"
#include "NVSManager.h"
#include "OrientationHandler.h"
#include "PowerMonitor.h"
#include "RobotState.h"
#include "Telemetry.h"
#include "kicker_controller.h"
#include "ssl_robot_protocol_bp.h"
#include "wheel_state_estimator.h"

static const char * TAG = "ProtocolHandler";

// An all-zero pose and velocity is a radio stop command.
static bool command_has_motion(const RobotCommand & command)
{
    const Pose & pose = command.target_pose;
    return pose.x != 0 || pose.y != 0 || pose.angle != 0 || pose.x_v != 0 || pose.y_v != 0 ||
           pose.angular_vel != 0;
}

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
 * @param radio Pointer to initialized ESP-NOW link
 * @param wheel_ctrl Pointer to initialized WheelController instance
 * @param kinematics Pointer to initialized OmnidirectionalRobot kinematics instance
 */
void ProtocolHandler::init(ESP_NOW_Dev * radio,
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
    ESP_LOGI(TAG, "ProtocolHandler initialized.");
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
 * @brief Report whether ESP-NOW initialized successfully.
 *
 * @return true when the radio module is initialized.
 */
bool ProtocolHandler::is_radio_ready() const
{
    return m_radio != nullptr && m_radio->is_receiving();
}

/**
 * @brief Report whether the base station link is currently up.
 *
 * The link counts as up while a base station packet arrived within the
 * link timeout window; a null radio or a silent base station reports false.
 *
 * @return true while base station packets are fresh.
 */
bool ProtocolHandler::is_base_station_connected() const
{
    const uint64_t last_rx_us = m_last_base_station_rx_us.load(std::memory_order_relaxed);
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    return is_radio_ready() && last_rx_us != 0 &&
           (now_us - last_rx_us) <= BASE_STATION_LINK_TIMEOUT_US;
}

bool ProtocolHandler::set_bench_targets(const std::array<float, 4> & targets)
{
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    const bool accepted = !m_radio_owns_wheels &&
                          !SafetySupervisor::health_inhibits_motion(m_health_state) &&
                          !MotorCalibration::get_instance().is_running();
    if (accepted)
        m_wheel_ctrl->set_target_velocities(targets);
    xSemaphoreGive(m_mutex);
    return accepted;
}

esp_err_t ProtocolHandler::apply_bench_wheel_tuning(int wheel,
                                                    uint32_t fields,
                                                    const WheelController::TuningValues & values)
{
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    esp_err_t result = ESP_ERR_INVALID_STATE;
    if (m_wheel_ctrl && !m_radio_owns_wheels &&
        !SafetySupervisor::health_inhibits_motion(m_health_state) &&
        !MotorCalibration::get_instance().is_running())
        result = m_wheel_ctrl->apply_live_tuning(wheel, fields, values);
    xSemaphoreGive(m_mutex);
    return result;
}

esp_err_t ProtocolHandler::start_bench_position_move(uint8_t wheel_mask,
                                                     const std::array<float, 4> & angles,
                                                     bool relative,
                                                     float max_velocity)
{
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    esp_err_t result = ESP_ERR_INVALID_STATE;
    if (!m_radio_owns_wheels && !SafetySupervisor::health_inhibits_motion(m_health_state) &&
        !MotorCalibration::get_instance().is_running())
        result = m_wheel_ctrl->start_position_move(wheel_mask, angles, relative, max_velocity);
    xSemaphoreGive(m_mutex);
    return result;
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
 * @param source Sender's MAC address for telemetry replies
 * @param received_at_us Radio arrival time used for command freshness
 */
void ProtocolHandler::on_packet(const uint8_t * payload,
                                uint8_t len,
                                const uint8_t * source,
                                uint64_t received_at_us)
{
    if (len < BYTES_LENGTH_HEADER)
    {
        ESP_LOGW(TAG, "Packet too short for header (%d bytes). Discarding.", len);
        return;
    }

    if (len <= sizeof(m_last_rx_payload) && len == m_last_rx_len && m_last_rx_us != 0 &&
        (received_at_us - m_last_rx_us) < RX_DEDUP_WINDOW_US &&
        memcmp(payload, m_last_rx_payload, len) == 0)
    {
        ESP_LOGD(TAG, "Duplicate packet ignored (%u bytes).", len);
        return;
    }
    if (len <= sizeof(m_last_rx_payload))
    {
        memcpy(m_last_rx_payload, payload, len);
        m_last_rx_len = len;
        m_last_rx_us = received_at_us;
    }

    // IMPORTANT: bitproto decoders use bitwise OR — always decode into a
    // zero-initialised struct to prevent stale bits bleeding into fields.
    Header hdr;
    memset(&hdr, 0, sizeof(Header));
    DecodeHeader(&hdr, const_cast<uint8_t *>(payload));

    // These message types originate from the base station. The header confirms
    // the link even when the packet targets another robot.
    // Clearing ERROR_COMM_LOSS here (not just in update()) means a telemetry
    // reply reports the live link instead of the stale pre-packet state;
    // update() re-arms the flag when the link goes quiet.
    if (hdr.msg_type == MSG_TYPE_COMMAND || hdr.msg_type == MSG_TYPE_CONFIG ||
        hdr.msg_type == MSG_TYPE_TELEMETRY_REQ || hdr.msg_type == MSG_TYPE_TEAM_VEL_CMD)
    {
        m_last_base_station_rx_us.store(received_at_us, std::memory_order_relaxed);
        if (is_base_station_connected())
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
            handle_command(cmd, received_at_us);
            update();
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
            handle_telemetry_req(telreq, source);
            break;
        }

        case MSG_TYPE_TEAM_VEL_CMD:
        {
            const uint8_t qtd_robots = hdr.robot_id;
            const size_t expected_lenght =
              BYTES_LENGTH_HEADER +
              static_cast<size_t>(qtd_robots) * BYTES_LENGTH_SINGLE_ROBOT_VEL_CMD;

            if (qtd_robots > TEAM_VEL_CMD_MAX_ROBOTS || len < expected_lenght || qtd_robots == 0)
            {
                ESP_LOGD(TAG,
                         "MSG_TYPE_TEAM_VEL_CMD: invalid message (%d/ %zu bytes)",
                         len,
                         expected_lenght);
                return;
            }

            const uint8_t my_id = RobotState::get_instance().get_id();

            for (size_t i = 0; i < qtd_robots; i++)
            {
                const uint8_t * slot_atual =
                  payload + BYTES_LENGTH_HEADER + i * BYTES_LENGTH_SINGLE_ROBOT_VEL_CMD;

                const uint8_t slot_id = slot_atual[0] & 0x0F;  // 0000 1111

                if (slot_id == my_id)
                {
                    SingleRobotVelCmd t_comand;
                    memset(&t_comand, 0, sizeof(SingleRobotVelCmd));
                    DecodeSingleRobotVelCmd(&t_comand, const_cast<uint8_t *>(slot_atual));
                    handle_TeamCommand(t_comand, hdr, received_at_us);
                    update();
                    break;
                }
            }
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
 * @param received_at_us Radio arrival time
 */
void ProtocolHandler::handle_command(const RobotCommand & cmd, uint64_t received_at_us)
{
    ESP_LOGD(TAG,
             "CMD robot=%d ts=%lu x=%d y=%d vx=%d vy=%d w=%d kick=%d",
             cmd.header.robot_id,
             (unsigned long)cmd.header.timestamp,
             cmd.target_pose.x,
             cmd.target_pose.y,
             cmd.target_pose.x_v,
             cmd.target_pose.y_v,
             cmd.target_pose.angular_vel,
             cmd.kick_velocity);

    if (!m_command_inbox.push(cmd, received_at_us))
    {
        ESP_LOGW(TAG, "Command inbox full — command was not accepted.");
    }
}

void ProtocolHandler::handle_TeamCommand(const SingleRobotVelCmd & r_comand,
                                         const Header & hdr,
                                         uint64_t received_at_us)
{
    ESP_LOGD(TAG,
             "r_command vx=%d vy=%d vw=%d kick=%d",
             r_comand.vx,
             r_comand.vy,
             r_comand.vw,
             r_comand.kick_velocity);

    const uint8_t clamped_kick = (r_comand.kick_velocity > 100U) ? 100U : r_comand.kick_velocity;

    RobotCommand cmd = {};
    cmd.header = hdr;
    cmd.header.robot_id = r_comand.robot_id;
    cmd.kick_velocity =
      (clamped_kick > 0U)
        ? static_cast<uint8_t>((static_cast<uint16_t>(clamped_kick) * 255U + 50U) / 100U)
        : 0U;  // conversão 7bits (0-100) -> (0-256)
    cmd.target_pose.angle = 0;
    cmd.target_pose.angular_vel = r_comand.vw;
    cmd.target_pose.x = 0;
    cmd.target_pose.x_v = r_comand.vx;
    cmd.target_pose.y = 0;
    cmd.target_pose.y_v = r_comand.vy;

    if (!m_command_inbox.push(cmd, received_at_us))
    {
        ESP_LOGW(TAG, "Command inbox full — command was not accepted.");
    }
}

/**
 * @brief Consume queued commands and publish the current safe wheel target.
 */
void ProtocolHandler::update()
{
    if (m_wheel_ctrl == nullptr)
        return;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    HealthObservationFrame observation = {};
    PowerMonitor::get_instance().fill_observation(&observation);
    RobotState & robot_state = RobotState::get_instance();
    const uint16_t error_flags = robot_state.get_error_flags();
    observation.imu_available = (error_flags & ERROR_IMU_FAIL) == 0U;
    observation.tof_available = (error_flags & ERROR_TOF_FAIL) == 0U;
    observation.temperature_warning = (error_flags & ERROR_TEMPERATURE_WARN) != 0U;
    auto & kicker = KickerController::get_instance();
    observation.kicker_ready = kicker.is_ready();
    observation.kicker_fault = kicker.has_fault();
    auto & estimator = WheelStateEstimator::get_instance();
    for (size_t wheel = 0; wheel < ROBOT_WHEEL_COUNT; ++wheel)
    {
        const uint8_t bit = static_cast<uint8_t>(1U << wheel);
        if (estimator.is_channel_calibrated(wheel))
            observation.wheel_valid_mask |= bit;
        if ((error_flags & (ERROR_MOTOR0_FAULT << wheel)) != 0U)
            observation.motor_fault_mask |= bit;
    }
    m_health_state = m_health_monitor.update(observation);

    // Publish the warnings detected here through the radio telemetry as well.
    constexpr uint16_t MONITORED_FLAGS =
      ERROR_BATT_UNDERVOLT | ERROR_BATT_OVERVOLT | ERROR_ENCODER_INVALID;
    robot_state.set_error(MONITORED_FLAGS & m_health_state.active_faults);
    robot_state.clear_error(MONITORED_FLAGS & ~m_health_state.active_faults);

    // Keep ERROR_COMM_LOSS in step with the live link so telemetry carries
    // the protocol meaning of bit 0: communication loss with the base station.
    if (is_base_station_connected())
        robot_state.clear_error(ERROR_COMM_LOSS);
    else
        robot_state.set_error(ERROR_COMM_LOSS);

    const bool calibrating_motors = MotorCalibration::get_instance().is_running();
    const bool health_stop = SafetySupervisor::health_inhibits_motion(m_health_state);
    m_safety_supervisor.set_calibration_active(calibrating_motors);
    CommandEnvelope envelope = {};
    while (m_command_inbox.pop(&envelope))
    {
        if (!calibrating_motors && !health_stop)
        {
            m_latest_command = envelope;
            m_has_latest_command = true;
        }
    }
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    if (calibrating_motors)
    {
        // Calibration owns the wheel targets until it leaves the motors stopped.
        // A radio command received during a run must not start them afterward.
        m_has_latest_command = false;
        m_radio_owns_wheels = false;
        m_safety_supervisor.update(now_us, nullptr, m_health_state, {});
        xSemaphoreGive(m_mutex);
        return;
    }

    // Recovery needs a new command instead of resuming a target from before the fault.
    if (health_stop)
        m_has_latest_command = false;

    const CommandEnvelope * command = m_has_latest_command ? &m_latest_command : nullptr;
    const SafetyDecision decision = m_safety_supervisor.update(now_us, command, m_health_state, {});
    const bool radio_owns_wheels =
      decision.command_fresh && command_has_motion(m_latest_command.command);
    const bool kick_requested = decision.goal.kick_velocity > 0U;
    if (health_stop || (!radio_owns_wheels && m_radio_owns_wheels))
        m_wheel_ctrl->set_target_velocities({});
    if (!health_stop && (radio_owns_wheels || kick_requested))
    {
        const BodyVelocity body_target = m_position_controller.compute({}, decision.goal);
        const ActuationSetpoint setpoint =
          m_velocity_controller.compute(body_target, decision.goal, m_health_state);
        if (radio_owns_wheels)
            m_wheel_ctrl->set_target_velocities(setpoint.wheel_velocity_rad_s);
        if (kick_requested)
        {
            ESP_LOGD(TAG,
                     "[KICK-FLOW] cmd_kick=%u seq=%lu | ready=%d fault=%d V=%u | "
                     "lifecycle=%d sev=%d | goal_inh=%d sp_inh=%d sp_kick=%u",
                     m_latest_command.command.kick_velocity,
                     (unsigned long)m_latest_command.sequence,
                     kicker.is_ready(),
                     kicker.has_fault(),
                     kicker.get_voltage(),
                     static_cast<int>(decision.lifecycle),
                     static_cast<int>(m_health_state.severity),
                     decision.goal.kick_inhibited,
                     setpoint.kick_inhibited,
                     setpoint.kick_velocity);
        }
        if (!setpoint.kick_inhibited && setpoint.kick_velocity > 0U)
            kicker.request_kick(setpoint.kick_velocity, setpoint.command_sequence);
    }
    m_radio_owns_wheels = radio_owns_wheels;

    xSemaphoreGive(m_mutex);
}

/**
 * @brief Handle RobotConfig — process ID changes, calibration triggers, NVS resets.
 *
 * Validates target robot ID (filters if not for us or broadcast). Processes:
 * - CONFIG_FLAG_SET_ID: Set robot ID (validates against ROBOT_ID_MAX)
 * - CONFIG_FLAG_RUN_CALIBRATION: Start wheel controller tuning
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

    if (cfg.config_flags & CONFIG_FLAG_RUN_CALIBRATION)
    {
        xSemaphoreTake(m_mutex, portMAX_DELAY);
        const esp_err_t result = MotorCalibration::get_instance().start_controller_tuning();
        xSemaphoreGive(m_mutex);
        if (result == ESP_ERR_INVALID_STATE)
            ESP_LOGW(TAG, "Controller tuning start ignored: a motor calibration is running.");
        else if (result != ESP_OK)
            ESP_LOGE(TAG, "Controller tuning start failed: %s.", esp_err_to_name(result));
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
 * @param source Address of the gateway that requested telemetry
 */
void ProtocolHandler::handle_telemetry_req(const RobotTelemetryRequest & telreq,
                                           const uint8_t * source)
{
    ESP_LOGD(TAG, "Answering telemetry request, robot: %d", telreq.header.robot_id);

    Telemetry::get_instance().send(telreq.header.timestamp, source);
}
