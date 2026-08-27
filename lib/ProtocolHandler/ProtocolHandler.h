#ifndef PROTOCOL_HANDLER_H
#define PROTOCOL_HANDLER_H

#include <esp_log.h>
#include <stdint.h>

#include "CascadedMotionController.h"
#include "CommandInbox.h"
#include "NRF24L01.h"
#include "RuntimeTypes.h"
#include "SafetySupervisor.h"
#include "WheelController.h"
#include "omni_robot.h"
#include "ssl_robot_protocol_bp.h"

class ProtocolHandler
{
   public:
    static ProtocolHandler & get_instance();

    void init(NRF24L01 * radio, WheelController * wheel_ctrl, OmnidirectionalRobot * kinematics);

    void on_packet(const uint8_t * payload, uint8_t len);
    void update(uint64_t now_us);

   private:
    ProtocolHandler() = default;
    ~ProtocolHandler() = default;
    ProtocolHandler(const ProtocolHandler &) = delete;
    ProtocolHandler & operator=(const ProtocolHandler &) = delete;

    void handle_command(const RobotCommand & cmd);
    void handle_config(const RobotConfig & cfg);

    NRF24L01 * m_radio = nullptr;
    WheelController * m_wheel_ctrl = nullptr;
    CommandInbox m_command_inbox;
    CommandEnvelope m_latest_command = {};
    bool m_has_latest_command = false;
    HealthState m_health_state = {};
    SafetySupervisor m_safety_supervisor;
    CascadedMotionController m_motion_controller;
};

#endif  // PROTOCOL_HANDLER_H
