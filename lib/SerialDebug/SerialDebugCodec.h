#ifndef SERIAL_DEBUG_CODEC_H
#define SERIAL_DEBUG_CODEC_H

#include <stddef.h>
#include <stdint.h>

namespace serial_debug
{

// Largest serialized message: 1 (msg_type) + DebugTof (132)
static constexpr size_t MAX_MESSAGE_BYTES = 160;
// COBS adds one byte per 254 payload bytes, plus the 0x00 frame delimiter
static constexpr size_t MAX_FRAME_BYTES = MAX_MESSAGE_BYTES + 4;

/** Encode one wire frame: COBS(msg_type | message | crc8) followed by 0x00.
 * @param msg_type One of the DBG_MSG_* message type constants.
 * @param message Serialized bitproto message bytes.
 * @param message_len Message length in bytes.
 * @param out Output buffer, receives the complete frame including delimiter.
 * @param out_cap Capacity of the output buffer.
 * @return Frame length in bytes, or 0 when the output buffer is too small.
 */
size_t encode_frame(
  uint8_t msg_type, const uint8_t * message, size_t message_len, uint8_t * out, size_t out_cap);

/** CRC-8 (poly 0x07, init 0x00, no reflection). */
uint8_t crc8(const uint8_t * data, size_t len);

/** Streaming frame decoder: feed serial bytes, collect complete frames.
 * Garbage between valid frames (e.g. stray boot noise) fails the CRC and is
 * dropped, so the decoder self-synchronizes on every 0x00 delimiter.
 */
class Decoder
{
   public:
    /** Push one received byte into the decoder. */
    void feed(uint8_t byte);

    /** A complete, CRC-valid frame is waiting to be popped. */
    bool has_frame() const;

    /** Message type of the pending frame (valid while has_frame()). */
    uint8_t message_type() const;

    /** Serialized message bytes of the pending frame (after the type byte).
     * The pointer stays valid until the next completed frame overwrites it.
     */
    const uint8_t * message() const;

    /** Message length in bytes of the pending frame. */
    size_t message_length() const;

    /** Discard the pending frame. */
    void pop();

    /** Drop all decoder state (used when the link is re-established). */
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

#endif  // SERIAL_DEBUG_CODEC_H
