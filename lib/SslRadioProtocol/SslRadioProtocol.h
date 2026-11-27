#ifndef _SSL_RADIO_PROTOCOL_H_
#define _SSL_RADIO_PROTOCOL_H_

#include "ssl_robot_protocol_bp.h"

namespace ssl_radio
{

inline bool valid_downlink(const uint8_t * payload, size_t length)
{
    if (length < BYTES_LENGTH_HEADER)
        return false;

    const uint8_t message_type = payload[0] & 0x0F;
    const uint8_t robot_id = payload[0] >> 4;
    const bool robot_address = robot_id <= ROBOT_ID_MAX || robot_id == ROBOT_ID_BROADCAST;

    switch (message_type)
    {
        case MSG_TYPE_COMMAND:
            return robot_address && length == BYTES_LENGTH_ROBOT_COMMAND;
        case MSG_TYPE_CONFIG:
            return robot_address && length == BYTES_LENGTH_ROBOT_CONFIG;
        case MSG_TYPE_TELEMETRY_REQ:
            return robot_id <= ROBOT_ID_MAX && length == BYTES_LENGTH_ROBOT_TELEMETRY_REQUEST;
        case MSG_TYPE_TEAM_VEL_CMD:
            return robot_id > 0 && robot_id <= TEAM_VEL_CMD_MAX_ROBOTS &&
                   length == BYTES_LENGTH_HEADER + robot_id * BYTES_LENGTH_SINGLE_ROBOT_VEL_CMD;
        default:
            return false;
    }
}

}  // namespace ssl_radio

#endif  // _SSL_RADIO_PROTOCOL_H_
