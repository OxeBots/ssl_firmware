#include "SerialFraming.h"

#include <cstring>

namespace serial_framing
{

uint8_t crc8(const uint8_t * data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07)
                               : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

size_t cobs_encode(const uint8_t * in, size_t len, uint8_t * out)
{
    size_t written = 0;
    size_t code_index = 0;
    uint8_t code = 1;
    ++written;
    for (size_t i = 0; i < len; ++i)
    {
        if (in[i] != 0)
        {
            out[written++] = in[i];
            if (++code == 0xFF)
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

size_t cobs_decode(const uint8_t * in, size_t len, uint8_t * out, size_t out_cap)
{
    size_t written = 0;
    size_t i = 0;
    while (i < len)
    {
        const uint8_t code = in[i++];
        if (code == 0)
            return 0;
        for (uint8_t j = 1; j < code; ++j)
        {
            if (i >= len || written >= out_cap)
                return 0;
            out[written++] = in[i++];
        }
        if (code < 0xFF && i < len)
        {
            if (written >= out_cap)
                return 0;
            out[written++] = 0;
        }
    }
    return written;
}

size_t encode_frame(
  uint8_t msg_type, const uint8_t * message, size_t message_len, uint8_t * out, size_t out_cap)
{
    if (message == nullptr || out == nullptr || message_len == 0)
        return 0;

    static constexpr size_t MAX_INLINE_STAGED = 256;
    const size_t staged_len = message_len + 2;
    const size_t worst_frame_len = staged_len + staged_len / 254 + 2;
    if (staged_len > MAX_INLINE_STAGED || out_cap < worst_frame_len)
        return 0;

    uint8_t staged[MAX_INLINE_STAGED];
    staged[0] = msg_type;
    memcpy(staged + 1, message, message_len);
    staged[message_len + 1] = crc8(staged, message_len + 1);

    const size_t frame_len = cobs_encode(staged, staged_len, out) + 1;
    out[frame_len - 1] = 0x00;
    return frame_len;
}

}  // namespace serial_framing
