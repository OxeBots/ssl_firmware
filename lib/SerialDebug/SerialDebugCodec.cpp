#include "SerialDebugCodec.h"

namespace serial_debug
{

uint8_t crc8(const uint8_t * data, size_t len)
{
    return serial_framing::crc8(data, len);
}

size_t encode_frame(
  uint8_t msg_type, const uint8_t * message, size_t message_len, uint8_t * out, size_t out_cap)
{
    if (message_len + 2 > MAX_MESSAGE_BYTES)
        return 0;
    return serial_framing::encode_frame(msg_type, message, message_len, out, out_cap);
}

}  // namespace serial_debug
