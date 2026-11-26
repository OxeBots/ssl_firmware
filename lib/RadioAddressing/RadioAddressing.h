#ifndef _RADIO_ADDRESSING_H_
#define _RADIO_ADDRESSING_H_

#include <stddef.h>
#include <stdint.h>

#include <array>

namespace radio_addressing
{

static constexpr uint8_t MAX_CHANNEL = 125;
static constexpr uint8_t DEFAULT_CHANNEL = 76;
static constexpr uint8_t MAX_ROBOT_ID = 10;
static constexpr uint8_t BROADCAST_ROBOT_ID = 15;

// Shared 4 upper address bytes (LSByte-first hardware order: bytes 1..4).
// Chosen with balanced bit transitions and no 0x00/0xFF/0x55/0xAA preamble traps.
static constexpr std::array<uint8_t, 4> DEFAULT_ADDR_SUFFIX = {0xC6, 0x9A, 0xD3, 0xE5};

// Unique LSByte (byte 0) per endpoint on the shared upper-byte family.
static constexpr uint8_t DEFAULT_BASE_ADDR_LSB = 0xB4;
static constexpr uint8_t BROADCAST_ADDR_LSB = 0xD9;

// Unique LSByte per robot ID (0..10): balanced bit density, Hamming distance >= 2.
static constexpr std::array<uint8_t, MAX_ROBOT_ID + 1> ROBOT_ADDR_LSB = {
  0x35,  // Robot 0
  0x5A,  // Robot 1
  0x6C,  // Robot 2
  0x93,  // Robot 3
  0xA5,  // Robot 4
  0xC3,  // Robot 5
  0x3A,  // Robot 6
  0x56,  // Robot 7
  0x69,  // Robot 8
  0x96,  // Robot 9
  0xAC,  // Robot 10
};

// Rendezvous channels visited by robots after prolonged communication loss:
// preserves channel 76 for easy recovery and prioritizes clean channels above Wi-Fi (100..124).
static constexpr std::array<uint8_t, 6> RENDEZVOUS_CHANNELS = {76, 100, 104, 114, 122, 88};

inline constexpr bool is_valid_channel(uint8_t channel)
{
    return channel <= MAX_CHANNEL;
}

inline constexpr bool is_valid_robot_id(uint8_t robot_id)
{
    return robot_id <= MAX_ROBOT_ID;
}

inline std::array<uint8_t, 5> make_address(uint8_t lsb,
                                           const std::array<uint8_t, 4> & suffix = DEFAULT_ADDR_SUFFIX)
{
    return {lsb, suffix[0], suffix[1], suffix[2], suffix[3]};
}

inline std::array<uint8_t, 5> make_base_address(
  const std::array<uint8_t, 4> & suffix = DEFAULT_ADDR_SUFFIX,
  uint8_t base_lsb = DEFAULT_BASE_ADDR_LSB)
{
    return make_address(base_lsb, suffix);
}

inline std::array<uint8_t, 5> make_broadcast_address(
  const std::array<uint8_t, 4> & suffix = DEFAULT_ADDR_SUFFIX)
{
    return make_address(BROADCAST_ADDR_LSB, suffix);
}

inline std::array<uint8_t, 5> make_robot_address(
  uint8_t robot_id, const std::array<uint8_t, 4> & suffix = DEFAULT_ADDR_SUFFIX)
{
    if (robot_id <= MAX_ROBOT_ID)
        return make_address(ROBOT_ADDR_LSB[robot_id], suffix);
    return make_broadcast_address(suffix);
}

// Select the cleanest channel from a spectrum scan (busy_pct[0..count-1], 0..100).
// Accounts for 2 MHz bandwidth when data_rate == 1 (2 Mbps) by penalizing adjacent channels
// and prefers the Wi-Fi-free upper band (100..124) on ties.
inline uint8_t select_best_channel(const uint8_t * busy_pct,
                                   uint8_t start_channel,
                                   uint8_t count,
                                   uint8_t data_rate)
{
    if (busy_pct == nullptr || count == 0)
        return DEFAULT_CHANNEL;

    const bool wide_band = (data_rate == 1);  // 2 Mbps occupies ~2 MHz
    uint8_t best_ch = start_channel;
    uint32_t best_score = 0xFFFFFFFFUL;

    for (uint8_t i = 0; i < count; ++i)
    {
        const uint16_t ch = static_cast<uint16_t>(start_channel) + i;
        if (ch > MAX_CHANNEL)
            break;

        // Avoid edges at 2 Mbps when interior channels exist.
        if (wide_band && count >= 3 && (i == 0 || i + 1 == count))
            continue;

        uint32_t score = static_cast<uint32_t>(busy_pct[i]) * 100U;
        if (wide_band)
        {
            if (i > 0)
                score += static_cast<uint32_t>(busy_pct[i - 1]) * 35U;
            if (i + 1 < count)
                score += static_cast<uint32_t>(busy_pct[i + 1]) * 35U;
        }
        else
        {
            if (i > 0)
                score += static_cast<uint32_t>(busy_pct[i - 1]) * 10U;
            if (i + 1 < count)
                score += static_cast<uint32_t>(busy_pct[i + 1]) * 10U;
        }

        // Small tie-breaker preferring channels above Wi-Fi (100..124), then > 84.
        if (ch < 85)
            score += 20U;
        else if (ch < 100 || ch > 124)
            score += 5U;

        if (score < best_score)
        {
            best_score = score;
            best_ch = static_cast<uint8_t>(ch);
        }
    }
    return best_ch;
}

// Deterministic state machine for robot channel-hunt rendezvous fallback.
// Keeps the robot locked on its home channel during normal operation, and cycles
// through [home_channel, RENDEZVOUS_CHANNELS...] only after trigger_us without
// any base-station traffic.
class ChannelHunter
{
   public:
    static constexpr uint64_t DEFAULT_TRIGGER_US = 5000000ULL;  // 5 seconds
    static constexpr uint64_t DEFAULT_DWELL_US = 300000ULL;     // 300 ms per channel

    explicit ChannelHunter(uint8_t home_channel = DEFAULT_CHANNEL,
                           uint64_t trigger_us = DEFAULT_TRIGGER_US,
                           uint64_t dwell_us = DEFAULT_DWELL_US)
    : m_home_channel(is_valid_channel(home_channel) ? home_channel : DEFAULT_CHANNEL),
      m_active_channel(m_home_channel),
      m_trigger_us(trigger_us),
      m_dwell_us(dwell_us)
    {
    }

    void set_home_channel(uint8_t channel, uint64_t now_us)
    {
        if (!is_valid_channel(channel))
            return;
        m_home_channel = channel;
        m_active_channel = channel;
        m_hunting = false;
        m_step_index = 0;
        m_last_activity_us = now_us;
        m_last_hop_us = now_us;
    }

    void on_packet_received(uint64_t now_us)
    {
        m_last_activity_us = now_us;
        m_hunting = false;
        m_step_index = 0;
    }

    // Advance state machine. Returns true and writes out_channel when the radio
    // should switch to a different channel.
    bool tick(uint64_t now_us, bool inhibited, uint8_t * out_channel)
    {
        if (out_channel == nullptr)
            return false;

        if (inhibited)
        {
            m_last_activity_us = now_us;
            if (m_hunting)
            {
                m_hunting = false;
                m_step_index = 0;
                if (m_active_channel != m_home_channel)
                {
                    m_active_channel = m_home_channel;
                    *out_channel = m_active_channel;
                    return true;
                }
            }
            return false;
        }

        if (m_last_activity_us == 0)
            m_last_activity_us = now_us;

        if (!m_hunting)
        {
            if (now_us >= m_last_activity_us && (now_us - m_last_activity_us) >= m_trigger_us)
            {
                m_hunting = true;
                m_step_index = 0;
                m_last_hop_us = now_us;
                return advance_to_next_distinct(out_channel);
            }
            return false;
        }

        if (now_us >= m_last_hop_us && (now_us - m_last_hop_us) >= m_dwell_us)
        {
            m_last_hop_us = now_us;
            return advance_to_next_distinct(out_channel);
        }
        return false;
    }

    bool is_hunting() const
    {
        return m_hunting;
    }

    uint8_t active_channel() const
    {
        return m_active_channel;
    }

    uint8_t home_channel() const
    {
        return m_home_channel;
    }

   private:
    static constexpr size_t CANDIDATE_COUNT = 1 + RENDEZVOUS_CHANNELS.size();

    uint8_t candidate_at(size_t index) const
    {
        if (index == 0)
            return m_home_channel;
        return RENDEZVOUS_CHANNELS[index - 1];
    }

    bool advance_to_next_distinct(uint8_t * out_channel)
    {
        for (size_t attempt = 0; attempt < CANDIDATE_COUNT; ++attempt)
        {
            m_step_index = (m_step_index + 1) % CANDIDATE_COUNT;
            const uint8_t next_ch = candidate_at(m_step_index);
            if (next_ch != m_active_channel)
            {
                m_active_channel = next_ch;
                *out_channel = next_ch;
                return true;
            }
        }
        return false;
    }

    uint8_t m_home_channel = DEFAULT_CHANNEL;
    uint8_t m_active_channel = DEFAULT_CHANNEL;
    uint64_t m_trigger_us = DEFAULT_TRIGGER_US;
    uint64_t m_dwell_us = DEFAULT_DWELL_US;
    uint64_t m_last_activity_us = 0;
    uint64_t m_last_hop_us = 0;
    size_t m_step_index = 0;
    bool m_hunting = false;
};

}  // namespace radio_addressing

#endif  // _RADIO_ADDRESSING_H_
