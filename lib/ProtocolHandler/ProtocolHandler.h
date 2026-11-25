#ifndef _PROTOCOL_HANDLER_H_
#define _PROTOCOL_HANDLER_H_

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdint.h>

#include <atomic>

#include "CommandInbox.h"
#include "HealthMonitor.h"
#include "NRF24L01.h"
#include "RobotPositionController.h"
#include "RobotVelocityController.h"
#include "RuntimeTypes.h"
#include "SafetySupervisor.h"
#include "WheelController.h"
#include "kicker_controller.h"
#include "omni_robot.h"
#include "ssl_robot_protocol_bp.h"

class ProtocolHandler
{
   public:
    static ProtocolHandler & get_instance();

    void init(NRF24L01 * radio, WheelController * wheel_ctrl, OmnidirectionalRobot * kinematics);

    void on_packet(const uint8_t * payload, uint8_t len);
    void update();

    // Current safety lifecycle for status consumers (serial debug).
    LifecycleState get_lifecycle() const;

    // Copy of the latest health snapshot for status consumers (serial debug).
    HealthState get_health_state() const;

    // True when the NRF24L01 module answered SPI during init.
    bool is_radio_ready() const;

    // True while a base station packet arrived within the link window.
    bool is_base_station_connected() const;

    bool set_bench_targets(const std::array<float, 4> & targets);
    esp_err_t apply_bench_wheel_tuning(int wheel,
                                       uint32_t fields,
                                       const WheelController::TuningValues & values);
    esp_err_t start_bench_position_move(uint8_t wheel_mask,
                                        const std::array<float, 4> & angles,
                                        bool relative,
                                        float max_velocity);

   private:
    ProtocolHandler();
    ~ProtocolHandler() = default;
    ProtocolHandler(const ProtocolHandler &) = delete;
    ProtocolHandler & operator=(const ProtocolHandler &) = delete;

    void handle_command(const RobotCommand & cmd);
    void handle_config(const RobotConfig & cfg);
    void handle_telemetry_req(const RobotTelemetryRequest & telreq);
    void handle_TeamCommand(const SingleRobotVelCmd & r_comand, const Header & hdr);

    // Serializes radio and bench commands and status reads.
    SemaphoreHandle_t m_mutex = nullptr;

    NRF24L01 * m_radio = nullptr;
    WheelController * m_wheel_ctrl = nullptr;
    // Base station link: receive time of the latest COMMAND/CONFIG packet.
    // Atomic because on_packet runs in the radio receiver task while the
    // control loop and status consumers read it from other tasks.
    static constexpr uint64_t BASE_STATION_LINK_TIMEOUT_US = 2000000;
    std::atomic<uint64_t> m_last_base_station_rx_us = 0;
    // Drop identical back-to-back downlink retransmissions from the base
    // station within this short window when Pipe 1 acknowledgement is off.
    static constexpr uint64_t RX_DEDUP_WINDOW_US = 50000;
    uint8_t m_last_rx_payload[31] = {};
    uint8_t m_last_rx_len = 0;
    uint64_t m_last_rx_us = 0;
    CommandInbox m_command_inbox;
    CommandEnvelope m_latest_command = {};
    bool m_has_latest_command = false;
    bool m_radio_owns_wheels = false;
    HealthState m_health_state = {};
    HealthMonitor m_health_monitor;
    SafetySupervisor m_safety_supervisor;
    RobotPositionController m_position_controller;
    RobotVelocityController m_velocity_controller;
};

#endif  // _PROTOCOL_HANDLER_H_
