/**
 * @file SerialDebugCodec.cpp
 * @brief COBS + CRC8 wire codec for serial Bitproto messages.
 *
 * Frame layout on the wire:
 *
 *     COBS_encode( msg_type (1 B) | bitproto message | crc8 (1 B) ) 0x00
 *
 * COBS (Consistent Overhead Byte Stuffing) guarantees the encoded frame
 * never contains 0x00, so the delimiter both frames and re-synchronizes the
 * stream: after any garbage burst the decoder simply skips bytes until the
 * next 0x00. The CRC-8 (poly 0x07) covers the type byte and the message.
 */

#include "SerialDebugCodec.h"

#include <cstring>

namespace serial_debug
{
namespace
{
/**
 * @brief COBS-encode a payload without the trailing 0x00 delimiter.
 * @param in Payload bytes.
 * @param len Payload length in bytes.
 * @param out Output buffer, needs len + ceil(len / 254) + 1 bytes.
 * @return Encoded length in bytes.
 */
size_t cobs_encode(const uint8_t * in, size_t len, uint8_t * out)
{
    size_t written = 0;
    size_t code_index = 0;
    uint8_t code = 1;
    ++written;  // reserve the first code byte
    for (size_t i = 0; i < len; ++i)
    {
        if (in[i] != 0)
        {
            out[written++] = in[i];
            if (++code == 0xFF)  // Block full: emit 0xFF and start a new one
            {
                out[code_index] = 0xFF;
                code = 1;
                code_index = written;
                ++written;
            }
        }
        else
        {
            out[code_index] = code;
            code = 1;
            code_index = written;
            ++written;
        }
    }
    out[code_index] = code;
    return written;
}

/**
 * @brief COBS-decode one encoded frame.
 * @param in Encoded frame bytes (delimiter not included).
 * @param len Encoded length in bytes.
 * @param out Output buffer.
 * @param out_cap Output buffer capacity.
 * @return Decoded length in bytes, or 0 on any malformed input.
 */
size_t cobs_decode(const uint8_t * in, size_t len, uint8_t * out, size_t out_cap)
{
    size_t written = 0;
    size_t i = 0;
    while (i < len)
    {
        const uint8_t code = in[i++];
        if (code == 0)  // Encoders never emit a zero code byte
            return 0;
        for (uint8_t j = 1; j < code; ++j)
        {
            if (i >= len || written >= out_cap)  // Truncated or oversized
                return 0;
            out[written++] = in[i++];
        }
        // A non-final, non-overrun block ends with an implicit zero byte.
        if (code < 0xFF && i < len)
        {
            if (written >= out_cap)
                return 0;
            out[written++] = 0;
        }
    }
    return written;
}

}  // namespace

uint8_t crc8(const uint8_t * data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit)
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07)
                               : static_cast<uint8_t>(crc << 1);
    }
    return crc;
}

size_t encode_frame(
  uint8_t msg_type, const uint8_t * message, size_t message_len, uint8_t * out, size_t out_cap)
{
    if (message == nullptr || out == nullptr || message_len == 0)
        return 0;
    if (message_len + 2 > MAX_MESSAGE_BYTES || out_cap < message_len + 4)
        return 0;

    // Stage: msg_type | message | crc8
    uint8_t staged[MAX_MESSAGE_BYTES];
    staged[0] = msg_type;
    memcpy(staged + 1, message, message_len);
    staged[message_len + 1] = crc8(staged, message_len + 1);
    const size_t staged_len = message_len + 2;

    const size_t frame_len = cobs_encode(staged, staged_len, out) + 1;
    out[frame_len - 1] = 0x00;
    return frame_len;
}

void Decoder::feed(uint8_t byte)
{
    if (byte == 0x00)
    {
        if (m_overflow)
        {
            // The runaway block is over; resume collecting at the next byte.
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
        // No delimiter in sight: drop the block and skip until the next one.
        m_accum_len = 0;
        m_overflow = true;
        return;
    }
    m_accum[m_accum_len++] = byte;
}

void Decoder::close_frame()
{
    uint8_t decoded[MAX_MESSAGE_BYTES];
    const size_t decoded_len = cobs_decode(m_accum, m_accum_len, decoded, sizeof(decoded));
    if (decoded_len < 3)  // Minimum frame: msg_type + 1 message byte + crc8
        return;
    if (crc8(decoded, decoded_len - 1) != decoded[decoded_len - 1])
        return;
    m_type = decoded[0];
    m_message_len = decoded_len - 2;
    memcpy(m_message, decoded + 1, m_message_len);
    m_ready = true;
}

bool Decoder::has_frame() const
{
    return m_ready;
}

uint8_t Decoder::message_type() const
{
    return m_type;
}

const uint8_t * Decoder::message() const
{
    return m_message;
}

size_t Decoder::message_length() const
{
    return m_message_len;
}

void Decoder::pop()
{
    m_ready = false;
    m_message_len = 0;
}

void Decoder::reset()
{
    m_accum_len = 0;
    m_overflow = false;
    m_ready = false;
    m_message_len = 0;
}

}  // namespace serial_debug
