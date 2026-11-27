#ifndef _SERIAL_DEBUG_CODEC_H_
#define _SERIAL_DEBUG_CODEC_H_

#include <stddef.h>
#include <stdint.h>

#include "serial_debug_protocol_bp.h"

namespace serial_debug
{

// Largest message includes its type and checksum around the range payload.
static constexpr size_t MAX_MESSAGE_BYTES = BYTES_LENGTH_DEBUG_TOF + 2;
// Include the encoding overhead and final frame delimiter.
static constexpr size_t MAX_FRAME_BYTES = MAX_MESSAGE_BYTES + MAX_MESSAGE_BYTES / 254 + 2;

// Encode one wire frame: COBS(msg_type | message | crc8) followed by 0x00.
// msg_type: message type for the serial protocol using the frame.
// message: serialized bitproto message bytes.
// message_len: message length in bytes.
// out: output buffer, receives the complete frame including delimiter.
// out_cap: capacity of the output buffer.
// Returns the frame length in bytes, or 0 when the output buffer is too small.
size_t encode_frame(
  uint8_t msg_type, const uint8_t * message, size_t message_len, uint8_t * out, size_t out_cap);

// CRC-8 (poly 0x07, init 0x00, no reflection).
uint8_t crc8(const uint8_t * data, size_t len);

// Streaming frame decoder: feed serial bytes, collect complete frames.
// Garbage between valid frames (e.g. stray boot noise) fails the CRC and is
// dropped, so the decoder self-synchronizes on every 0x00 delimiter.
class Decoder
{
   public:
    // Push one received byte into the decoder.
    void feed(uint8_t byte);

    // A complete, CRC-valid frame is waiting to be popped.
    bool has_frame() const;

    // Message type of the pending frame (valid while has_frame()).
    uint8_t message_type() const;

    // Serialized message bytes of the pending frame (after the type byte).
    // The pointer stays valid until the next completed frame overwrites it.
    const uint8_t * message() const;

    // Message length in bytes of the pending frame.
    size_t message_length() const;

    // Discard the pending frame.
    void pop();

    // Drop all decoder state (used when the link is re-established).
    void reset();

   private:
    void close_frame();

    uint8_t m_accum[MAX_FRAME_BYTES] = {};
    size_t m_accum_len = 0;
    bool m_overflow = false;
    bool m_ready = false;
    uint8_t m_type = 0;
    uint8_t m_message[MAX_MESSAGE_BYTES] = {};
    size_t m_message_len = 0;
};

}  // namespace serial_debug

#endif  // _SERIAL_DEBUG_CODEC_H_
