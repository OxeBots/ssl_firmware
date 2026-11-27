#ifndef _RADIO_ADDRESSING_H_
#define _RADIO_ADDRESSING_H_

#include <stddef.h>
#include <stdint.h>

#include <array>

namespace radio_addressing
{

static constexpr uint8_t MAX_CHANNEL = 125;
static constexpr uint8_t DEFAULT_CHANNEL = 76;
static constexpr uint8_t DEFAULT_RX_CHANNEL = 90;  // +14 MHz above DEFAULT_CHANNEL
static constexpr uint8_t RX_CHANNEL_FOLLOW_TX = 0xFF;
static constexpr uint8_t DEFAULT_MIN_CHANNEL_SEP = 10;
static constexpr uint8_t DEFAULT_OPTIMAL_CHANNEL_SEP = 14;
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

struct ChannelPair
{
    uint8_t tx_channel = DEFAULT_CHANNEL;
    uint8_t rx_channel = DEFAULT_RX_CHANNEL;
    uint8_t separation = 14;
    uint32_t score = 0;
};

// Encode a (base_tx_ch, base_rx_ch) pair into RobotConfig.param (uint16).
// When base_tx_ch == base_rx_ch, encodes in legacy single-channel form (bit 15 = 0).
// When distinct (Full-Duplex), sets bit 15 = 1 so even rx_ch == 0 is unambiguous.
inline constexpr uint16_t encode_channel_pair_param(uint8_t base_tx_ch, uint8_t base_rx_ch)
{
    if (base_rx_ch == RX_CHANNEL_FOLLOW_TX || base_rx_ch == base_tx_ch)
        return static_cast<uint16_t>(base_tx_ch & 0x7FU);
    return static_cast<uint16_t>(0x8000U | (static_cast<uint16_t>(base_rx_ch & 0x7FU) << 8U) |
                                 static_cast<uint16_t>(base_tx_ch & 0x7FU));
}

// Decode RobotConfig.param into robot downlink (RX) and uplink (TX) channels.
// Returns false if either decoded channel exceeds MAX_CHANNEL.
inline bool decode_channel_pair_param(uint16_t param,
                                      uint8_t * out_downlink_ch,
                                      uint8_t * out_uplink_ch)
{
    if (out_downlink_ch == nullptr || out_uplink_ch == nullptr)
        return false;

    const bool split_flag = (param & 0x8000U) != 0U;
    if (!split_flag)
    {
        if (param > MAX_CHANNEL)
            return false;
        const uint8_t ch = static_cast<uint8_t>(param);
        *out_downlink_ch = ch;
        *out_uplink_ch = ch;
        return true;
    }

    const uint8_t downlink = static_cast<uint8_t>(param & 0x7FU);
    const uint8_t uplink = static_cast<uint8_t>((param >> 8U) & 0x7FU);
    if (!is_valid_channel(downlink) || !is_valid_channel(uplink))
        return false;
    *out_downlink_ch = downlink;
    *out_uplink_ch = uplink;
    return true;
}

inline uint32_t channel_noise_cost(const uint8_t * busy_pct,
                                   uint8_t start_channel,
                                   uint8_t count,
                                   uint8_t idx,
                                   uint8_t data_rate)
{
    const bool wide_band = (data_rate == 1);  // 2 Mbps occupies ~2 MHz
    const uint16_t ch = static_cast<uint16_t>(start_channel) + idx;
    uint32_t score = static_cast<uint32_t>(busy_pct[idx]) * 100U;
    if (wide_band)
    {
        if (idx > 0)
            score += static_cast<uint32_t>(busy_pct[idx - 1]) * 35U;
        if (idx + 1 < count)
            score += static_cast<uint32_t>(busy_pct[idx + 1]) * 35U;
        if (idx > 1)
            score += static_cast<uint32_t>(busy_pct[idx - 2]) * 10U;
        if (idx + 2 < count)
            score += static_cast<uint32_t>(busy_pct[idx + 2]) * 10U;
    }
    else
    {
        if (idx > 0)
            score += static_cast<uint32_t>(busy_pct[idx - 1]) * 10U;
        if (idx + 1 < count)
            score += static_cast<uint32_t>(busy_pct[idx + 1]) * 10U;
    }

    // Small tie-breaker preferring channels above Wi-Fi (100..124), then > 84.
    if (ch < 85)
        score += 20U;
    else if (ch < 100 || ch > 124)
        score += 5U;

    return score;
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

        const uint32_t score = channel_noise_cost(busy_pct, start_channel, count, i, data_rate);
        if (score < best_score)
        {
            best_score = score;
            best_ch = static_cast<uint8_t>(ch);
        }
    }
    return best_ch;
}

// Select the cleanest full-duplex (tx_channel, rx_channel) pair from a spectrum scan.
// Evaluates all clean channels with |tx - rx| >= min_sep and minimizes combined RF noise
// plus deviation from optimal_sep (default: min=10 MHz, optimal=14 MHz for 2 Mbps).
inline ChannelPair select_best_channel_pair(const uint8_t * busy_pct,
                                            uint8_t start_channel,
                                            uint8_t count,
                                            uint8_t data_rate,
                                            uint8_t min_sep = DEFAULT_MIN_CHANNEL_SEP,
                                            uint8_t optimal_sep = DEFAULT_OPTIMAL_CHANNEL_SEP,
                                            uint8_t exclude_tx = RX_CHANNEL_FOLLOW_TX,
                                            uint8_t exclude_rx = RX_CHANNEL_FOLLOW_TX)
{
    ChannelPair result = {};
    if (busy_pct == nullptr || count == 0)
        return result;

    if (min_sep == 0)
        min_sep = DEFAULT_MIN_CHANNEL_SEP;
    if (optimal_sep < min_sep)
        optimal_sep = min_sep;

    const bool wide_band = (data_rate == 1);
    const uint8_t valid_count =
      (static_cast<uint16_t>(start_channel) + count > static_cast<uint16_t>(MAX_CHANNEL) + 1U)
        ? static_cast<uint8_t>(MAX_CHANNEL - start_channel + 1U)
        : count;

    uint32_t costs[MAX_CHANNEL + 1] = {};
    bool eligible[MAX_CHANNEL + 1] = {};
    for (uint8_t i = 0; i < valid_count; ++i)
    {
        if (wide_band && valid_count >= 3 && (i == 0 || i + 1 == valid_count))
            continue;
        costs[i] = channel_noise_cost(busy_pct, start_channel, valid_count, i, data_rate);
        eligible[i] = true;
    }

    auto search = [&](uint8_t required_sep, bool apply_exclusions) -> bool {
        bool found = false;
        uint32_t best_pair_score = 0xFFFFFFFFUL;

        for (uint8_t i = 0; i < valid_count; ++i)
        {
            if (!eligible[i])
                continue;
            const uint8_t tx_ch = static_cast<uint8_t>(start_channel + i);
            if (apply_exclusions && tx_ch == exclude_tx)
                continue;

            for (uint8_t j = 0; j < valid_count; ++j)
            {
                if (!eligible[j] || i == j)
                    continue;
                const uint8_t rx_ch = static_cast<uint8_t>(start_channel + j);
                if (apply_exclusions && rx_ch == exclude_rx)
                    continue;

                const uint8_t sep =
                  (rx_ch > tx_ch) ? static_cast<uint8_t>(rx_ch - tx_ch)
                                  : static_cast<uint8_t>(tx_ch - rx_ch);
                if (sep < required_sep)
                    continue;

                // Prioritize downlink cleanliness (1.2x), then uplink cleanliness (1.0x),
                // then proximity to optimal_sep (8 pts per MHz deviation).
                const uint32_t sep_dev =
                  (sep > optimal_sep) ? static_cast<uint32_t>(sep - optimal_sep)
                                      : static_cast<uint32_t>(optimal_sep - sep);
                const uint32_t pair_score =
                  (costs[i] * 12U) / 10U + costs[j] + sep_dev * 8U + ((tx_ch > rx_ch) ? 1U : 0U);

                if (!found || pair_score < best_pair_score)
                {
                    found = true;
                    best_pair_score = pair_score;
                    result.tx_channel = tx_ch;
                    result.rx_channel = rx_ch;
                    result.separation = sep;
                    result.score = pair_score;
                }
            }
        }
        return found;
    };

    if (search(min_sep, true))
        return result;
    if ((exclude_tx != RX_CHANNEL_FOLLOW_TX || exclude_rx != RX_CHANNEL_FOLLOW_TX) &&
        search(min_sep, false))
        return result;
    // Narrow scan window fallback: accept any distinct pair with best available separation.
    if (search(1, false))
        return result;

    const uint8_t single = select_best_channel(busy_pct, start_channel, valid_count, data_rate);
    result.tx_channel = single;
    result.rx_channel = single;
    result.separation = 0;
    result.score = 0;
    return result;
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
