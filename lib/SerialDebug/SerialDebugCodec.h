#ifndef _SERIAL_DEBUG_CODEC_H_
#define _SERIAL_DEBUG_CODEC_H_

#include <stddef.h>
#include <stdint.h>

#include "SerialFraming.h"
#include "serial_debug_protocol_bp.h"

namespace serial_debug
{

// Largest message includes its type and checksum around the range payload.
static constexpr size_t MAX_MESSAGE_BYTES = BYTES_LENGTH_DEBUG_TOF + 2;
// Include the encoding overhead and final frame delimiter.
static constexpr size_t MAX_FRAME_BYTES = MAX_MESSAGE_BYTES + MAX_MESSAGE_BYTES / 254 + 2;

// Encode one wire frame: COBS(msg_type | message | crc8) followed by 0x00.
size_t encode_frame(
  uint8_t msg_type, const uint8_t * message, size_t message_len, uint8_t * out, size_t out_cap);

// Checksum with polynomial 0x07, initial value 0x00, no reflection.
uint8_t crc8(const uint8_t * data, size_t len);

// Streaming frame decoder: delegates to the shared serial_framing implementation.
class Decoder : public serial_framing::Decoder<MAX_MESSAGE_BYTES>
{
};

}  // namespace serial_debug

#endif  // _SERIAL_DEBUG_CODEC_H_
