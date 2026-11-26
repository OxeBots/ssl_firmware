#ifndef _SERIAL_FRAMING_H_
#define _SERIAL_FRAMING_H_

#include <stddef.h>
#include <stdint.h>

#include <cstring>

namespace serial_framing
{

// Compute CRC-8 (polynomial 0x07, initial value 0x00, no reflection).
uint8_t crc8(const uint8_t * data, size_t len);

// Consistent Overhead Byte Stuffing encoder (does not append the trailing 0x00 delimiter).
// Output buffer must hold at least len + len / 254 + 1 bytes.
size_t cobs_encode(const uint8_t * in, size_t len, uint8_t * out);

// Consistent Overhead Byte Stuffing decoder (delimiter excluded from input).
// Returns decoded byte count, or 0 on malformed or overflowing input.
size_t cobs_decode(const uint8_t * in, size_t len, uint8_t * out, size_t out_cap);

// Maximum encoded wire length (including trailing 0x00 delimiter) for a payload of size
// message_bytes plus 1-byte type and 1-byte checksum.
constexpr size_t max_frame_bytes_for(size_t max_payload_bytes)
{
    const size_t staged = max_payload_bytes + 2;
    return staged + staged / 254 + 2;
}

// Encode one framed message: COBS(msg_type | message | crc8) followed by 0x00.
// Returns total frame length in bytes, or 0 if invalid or buffer too small.
size_t encode_frame(
  uint8_t msg_type, const uint8_t * message, size_t message_len, uint8_t * out, size_t out_cap);

// Streaming frame decoder for COBS(msg_type | message | crc8) + 0x00 frames.
// Self-synchronizes after any stray boot noise or corrupted byte sequence.
template <size_t MaxPayloadBytes>
class Decoder
{
   public:
    static constexpr size_t MAX_STAGED_BYTES = MaxPayloadBytes + 2;
    static constexpr size_t MAX_FRAME_BYTES = max_frame_bytes_for(MaxPayloadBytes);

    void feed(uint8_t byte)
    {
        if (byte == 0x00)
        {
            if (m_overflow)
            {
                m_overflow = false;
                m_accum_len = 0;
            }
            else if (m_accum_len > 0)
            {
                close_frame();
                m_accum_len = 0;
            }
            return;
        }
        if (m_overflow)
            return;
        if (m_accum_len >= sizeof(m_accum))
        {
            m_accum_len = 0;
            m_overflow = true;
            return;
        }
        m_accum[m_accum_len++] = byte;
    }

    bool has_frame() const
    {
        return m_ready;
    }

    uint8_t message_type() const
    {
        return m_type;
    }

    const uint8_t * message() const
    {
        return m_message;
    }

    size_t message_length() const
    {
        return m_message_len;
    }

    void pop()
    {
        m_ready = false;
        m_message_len = 0;
    }

    void reset()
    {
        m_accum_len = 0;
        m_overflow = false;
        m_ready = false;
        m_message_len = 0;
    }

   private:
    void close_frame()
    {
        uint8_t decoded[MAX_STAGED_BYTES];
        const size_t decoded_len = cobs_decode(m_accum, m_accum_len, decoded, sizeof(decoded));
        if (decoded_len < 3)
            return;
        if (crc8(decoded, decoded_len - 1) != decoded[decoded_len - 1])
            return;
        m_type = decoded[0];
        m_message_len = decoded_len - 2;
        memcpy(m_message, decoded + 1, m_message_len);
        m_ready = true;
    }

    uint8_t m_accum[MAX_FRAME_BYTES] = {};
    size_t m_accum_len = 0;
    bool m_overflow = false;
    bool m_ready = false;
    uint8_t m_type = 0;
    uint8_t m_message[MaxPayloadBytes] = {};
    size_t m_message_len = 0;
};

}  // namespace serial_framing

#endif  // _SERIAL_FRAMING_H_
