#ifndef PROTOCOL_HANDLER_H
#define PROTOCOL_HANDLER_H

#include <esp_log.h>
#include <stdint.h>

#include "NRF24L01.h"
#include "WheelController.h"
#include "omni_robot.h"
#include "ssl_robot_protocol_bp.h"

class ProtocolHandler
{
   public:
    static ProtocolHandler & get_instance();

    void init(NRF24L01 * radio, WheelController * wheel_ctrl, OmnidirectionalRobot * kinematics);

    void on_packet(const uint8_t * payload, uint8_t len);

   private:
    ProtocolHandler() = default;
    ~ProtocolHandler() = default;
    ProtocolHandler(const ProtocolHandler &) = delete;
    ProtocolHandler & operator=(const ProtocolHandler &) = delete;

    void handle_command(const RobotCommand & cmd);
    void handle_config(const RobotConfig & cfg);

    NRF24L01 * m_radio = nullptr;
    WheelController * m_wheel_ctrl = nullptr;
    OmnidirectionalRobot * m_kinematics = nullptr;
};

#endif  // PROTOCOL_HANDLER_H
