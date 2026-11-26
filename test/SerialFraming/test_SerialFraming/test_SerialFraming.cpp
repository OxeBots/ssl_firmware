#include <string.h>
#include <unity.h>

#include "SerialFraming.h"

void test_framing_roundtrip_with_embedded_zeros(void)
{
    uint8_t payload[64];
    for (size_t i = 0; i < sizeof(payload); ++i)
        payload[i] = static_cast<uint8_t>((i % 7 == 0) ? 0x00 : (i * 5 + 3));

    uint8_t frame[serial_framing::max_frame_bytes_for(128)];
    const size_t len =
      serial_framing::encode_frame(0x84, payload, sizeof(payload), frame, sizeof(frame));
    TEST_ASSERT_GREATER_THAN(0, len);
    TEST_ASSERT_EQUAL_UINT8(0x00, frame[len - 1]);

    serial_framing::Decoder<128> decoder;
    for (size_t i = 0; i < len; ++i) decoder.feed(frame[i]);

    TEST_ASSERT_TRUE(decoder.has_frame());
    TEST_ASSERT_EQUAL_UINT8(0x84, decoder.message_type());
    TEST_ASSERT_EQUAL_size_t(sizeof(payload), decoder.message_length());
    TEST_ASSERT_EQUAL_MEMORY(payload, decoder.message(), sizeof(payload));
}

void test_framing_handles_129_byte_scan_payload(void)
{
    uint8_t payload[129];
    for (size_t i = 0; i < sizeof(payload); ++i)
        payload[i] = static_cast<uint8_t>(i % 101);

    uint8_t frame[serial_framing::max_frame_bytes_for(160)];
    const size_t len =
      serial_framing::encode_frame(0x8B, payload, sizeof(payload), frame, sizeof(frame));
    TEST_ASSERT_GREATER_THAN(0, len);

    serial_framing::Decoder<160> decoder;
    for (size_t i = 0; i < len; ++i) decoder.feed(frame[i]);

    TEST_ASSERT_TRUE(decoder.has_frame());
    TEST_ASSERT_EQUAL_UINT8(0x8B, decoder.message_type());
    TEST_ASSERT_EQUAL_size_t(sizeof(payload), decoder.message_length());
    TEST_ASSERT_EQUAL_MEMORY(payload, decoder.message(), sizeof(payload));
}

void test_framing_rejects_bad_crc_and_resyncs(void)
{
    const uint8_t payload[] = {1, 2, 3, 4};
    uint8_t frame[32];
    const size_t len =
      serial_framing::encode_frame(0x80, payload, sizeof(payload), frame, sizeof(frame));
    TEST_ASSERT_GREATER_THAN(0, len);

    uint8_t corrupted[32];
    memcpy(corrupted, frame, len);
    corrupted[2] ^= 0x20;

    serial_framing::Decoder<64> decoder;
    for (size_t i = 0; i < len; ++i) decoder.feed(corrupted[i]);
    TEST_ASSERT_FALSE(decoder.has_frame());

    for (size_t i = 0; i < len; ++i) decoder.feed(frame[i]);
    TEST_ASSERT_TRUE(decoder.has_frame());
    TEST_ASSERT_EQUAL_UINT8(0x80, decoder.message_type());
    TEST_ASSERT_EQUAL_MEMORY(payload, decoder.message(), sizeof(payload));
}

extern "C" void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_framing_roundtrip_with_embedded_zeros);
    RUN_TEST(test_framing_handles_129_byte_scan_payload);
    RUN_TEST(test_framing_rejects_bad_crc_and_resyncs);
    UNITY_END();
}
