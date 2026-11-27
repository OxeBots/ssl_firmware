#include <string.h>
#include <unity.h>

#include "RadioAddressing.h"

static uint8_t popcount8(uint8_t v)
{
    uint8_t count = 0;
    while (v != 0)
    {
        count += static_cast<uint8_t>(v & 1U);
        v >>= 1;
    }
    return count;
}

void test_addresses_are_unique_and_high_transition(void)
{
    using namespace radio_addressing;

    // Ensure no byte in the shared suffix is a degenerate preamble/flat byte.
    for (uint8_t byte : DEFAULT_ADDR_SUFFIX)
    {
        TEST_ASSERT_NOT_EQUAL(0x00, byte);
        TEST_ASSERT_NOT_EQUAL(0xFF, byte);
        TEST_ASSERT_NOT_EQUAL(0x55, byte);
        TEST_ASSERT_NOT_EQUAL(0xAA, byte);
    }

    // Collect all endpoint LSBs (base + broadcast + 11 robots) and check uniqueness + Hamming >= 2.
    uint8_t lsbs[MAX_ROBOT_ID + 3];
    size_t n = 0;
    lsbs[n++] = DEFAULT_BASE_ADDR_LSB;
    lsbs[n++] = BROADCAST_ADDR_LSB;
    for (uint8_t id = 0; id <= MAX_ROBOT_ID; ++id)
        lsbs[n++] = make_robot_address(id)[0];

    for (size_t i = 0; i < n; ++i)
    {
        TEST_ASSERT_NOT_EQUAL(0x00, lsbs[i]);
        TEST_ASSERT_NOT_EQUAL(0xFF, lsbs[i]);
        const uint8_t ones = popcount8(lsbs[i]);
        TEST_ASSERT_GREATER_OR_EQUAL_UINT8(3, ones);
        TEST_ASSERT_LESS_OR_EQUAL_UINT8(5, ones);
        for (size_t j = i + 1; j < n; ++j)
        {
            TEST_ASSERT_NOT_EQUAL(lsbs[i], lsbs[j]);
            TEST_ASSERT_GREATER_OR_EQUAL_UINT8(2, popcount8(static_cast<uint8_t>(lsbs[i] ^ lsbs[j])));
        }
    }
}

void test_select_best_channel_prefers_quiet_upper_band_and_2mhz_guard(void)
{
    uint8_t busy[126];
    memset(busy, 0, sizeof(busy));
    // Simulate heavy Wi-Fi traffic on channels 0..84 and an isolated blocker at 105.
    for (size_t ch = 0; ch <= 84; ++ch) busy[ch] = 60;
    busy[105] = 80;

    const uint8_t best = radio_addressing::select_best_channel(busy, 0, 126, 1);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT8(100, best);
    TEST_ASSERT_LESS_OR_EQUAL_UINT8(124, best);
    TEST_ASSERT_NOT_EQUAL(104, best);
    TEST_ASSERT_NOT_EQUAL(105, best);
    TEST_ASSERT_NOT_EQUAL(106, best);
}

void test_channel_hunter_waits_for_trigger_and_locks_on_rx(void)
{
    radio_addressing::ChannelHunter hunter(76, 5000000ULL, 300000ULL);
    uint8_t next_ch = 0;

    // Seed activity at t = 1s.
    hunter.on_packet_received(1000000ULL);
    TEST_ASSERT_FALSE(hunter.tick(5900000ULL, false, &next_ch));
    TEST_ASSERT_FALSE(hunter.is_hunting());

    // After 5s of silence (t = 6.1s), starts hopping.
    TEST_ASSERT_TRUE(hunter.tick(6100000ULL, false, &next_ch));
    TEST_ASSERT_TRUE(hunter.is_hunting());
    TEST_ASSERT_NOT_EQUAL(76, next_ch);
    const uint8_t first_hop = next_ch;

    // Stays on channel during dwell time.
    TEST_ASSERT_FALSE(hunter.tick(6250000ULL, false, &next_ch));

    // Hops again after 300 ms.
    TEST_ASSERT_TRUE(hunter.tick(6450000ULL, false, &next_ch));
    TEST_ASSERT_NOT_EQUAL(first_hop, next_ch);
    const uint8_t locked_ch = next_ch;

    // Packet received locks onto current channel and stops hunting.
    hunter.on_packet_received(6500000ULL);
    TEST_ASSERT_FALSE(hunter.is_hunting());
    TEST_ASSERT_EQUAL_UINT8(locked_ch, hunter.active_channel());
    TEST_ASSERT_FALSE(hunter.tick(9000000ULL, false, &next_ch));
}

void test_select_best_channel_pair_finds_optimal_distance_in_clean_band(void)
{
    uint8_t busy[126];
    memset(busy, 0, sizeof(busy));
    // Replicate real lab scan: heavy Wi-Fi on 0..64, spike at 101, clean 104..124.
    for (size_t ch = 0; ch <= 64; ++ch) busy[ch] = 75;
    busy[101] = 38;
    busy[102] = 3;

    const radio_addressing::ChannelPair pair =
      radio_addressing::select_best_channel_pair(busy, 0, 126, 1, 10, 14);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT8(100, pair.tx_channel);
    TEST_ASSERT_LESS_OR_EQUAL_UINT8(124, pair.tx_channel);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT8(100, pair.rx_channel);
    TEST_ASSERT_LESS_OR_EQUAL_UINT8(124, pair.rx_channel);
    TEST_ASSERT_EQUAL_UINT8(14, pair.separation);
    TEST_ASSERT_EQUAL_UINT8(0, busy[pair.tx_channel]);
    TEST_ASSERT_EQUAL_UINT8(0, busy[pair.rx_channel]);
    TEST_ASSERT_NOT_EQUAL(100, pair.tx_channel);  // adjacent to busy ch 101
    TEST_ASSERT_NOT_EQUAL(101, pair.tx_channel);

    // When clean band only offers 11 MHz clean separation, cleanliness must beat exact 14 MHz.
    memset(busy, 50, sizeof(busy));
    for (size_t ch = 102; ch <= 106; ++ch) busy[ch] = 0;
    for (size_t ch = 113; ch <= 117; ++ch) busy[ch] = 0;
    const radio_addressing::ChannelPair clean_pair =
      radio_addressing::select_best_channel_pair(busy, 0, 126, 1, 10, 14);
    TEST_ASSERT_EQUAL_UINT8(0, busy[clean_pair.tx_channel]);
    TEST_ASSERT_EQUAL_UINT8(0, busy[clean_pair.rx_channel]);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT8(10, clean_pair.separation);
}

void test_channel_pair_param_encode_decode(void)
{
    uint8_t dl = 0;
    uint8_t ul = 0;

    // Legacy same-channel encoding (bit 15 == 0)
    const uint16_t same = radio_addressing::encode_channel_pair_param(104, 104);
    TEST_ASSERT_EQUAL_UINT16(104, same);
    TEST_ASSERT_TRUE(radio_addressing::decode_channel_pair_param(same, &dl, &ul));
    TEST_ASSERT_EQUAL_UINT8(104, dl);
    TEST_ASSERT_EQUAL_UINT8(104, ul);

    // Full-Duplex split-channel encoding (bit 15 == 1)
    const uint16_t split = radio_addressing::encode_channel_pair_param(104, 118);
    TEST_ASSERT_TRUE((split & 0x8000U) != 0U);
    TEST_ASSERT_TRUE(radio_addressing::decode_channel_pair_param(split, &dl, &ul));
    TEST_ASSERT_EQUAL_UINT8(104, dl);
    TEST_ASSERT_EQUAL_UINT8(118, ul);

    // Edge case: rx_ch == 0 must stay unambiguous
    const uint16_t zero_rx = radio_addressing::encode_channel_pair_param(104, 0);
    TEST_ASSERT_TRUE(radio_addressing::decode_channel_pair_param(zero_rx, &dl, &ul));
    TEST_ASSERT_EQUAL_UINT8(104, dl);
    TEST_ASSERT_EQUAL_UINT8(0, ul);

    // Reject out-of-range channels (> 125)
    TEST_ASSERT_FALSE(radio_addressing::decode_channel_pair_param(126, &dl, &ul));
    TEST_ASSERT_FALSE(radio_addressing::decode_channel_pair_param(0x8000U | (126U << 8U) | 10U, &dl, &ul));
}

extern "C" void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_addresses_are_unique_and_high_transition);
    RUN_TEST(test_select_best_channel_prefers_quiet_upper_band_and_2mhz_guard);
    RUN_TEST(test_select_best_channel_pair_finds_optimal_distance_in_clean_band);
    RUN_TEST(test_channel_pair_param_encode_decode);
    RUN_TEST(test_channel_hunter_waits_for_trigger_and_locks_on_rx);
    UNITY_END();
}
