#include <string.h>
#include <unity.h>

#include "SerialDebugCodec.h"

void test_codec_preserves_full_raw_tof_frame(void)
{
    DebugTof original = {};
    original.sequence = 0xFFFF00FF;
    const uint8_t statuses[] = {5, 6, 9, 10, 3, 12, 0, 255};
    for (size_t i = 0; i < 64; ++i)
    {
        original.distance_mm[i] = static_cast<int16_t>((i % 3 == 0) ? -12 : i * 80);
        original.target_status[i] = statuses[i % sizeof(statuses)];
    }
    uint8_t message[BYTES_LENGTH_DEBUG_TOF];
    EncodeDebugTof(&original, message);
    uint8_t frame[serial_debug::MAX_FRAME_BYTES];
    const size_t frame_len =
      serial_debug::encode_frame(DBG_MSG_TOF, message, sizeof(message), frame, sizeof(frame));
    TEST_ASSERT_GREATER_THAN(0, frame_len);
    serial_debug::Decoder decoder;
    for (size_t i = 0; i + 1 < frame_len; ++i) decoder.feed(frame[i]);
    TEST_ASSERT_FALSE(decoder.has_frame());
    decoder.feed(frame[frame_len - 1]);
    TEST_ASSERT_TRUE(decoder.has_frame());
    TEST_ASSERT_EQUAL_UINT8(DBG_MSG_TOF, decoder.message_type());
    TEST_ASSERT_EQUAL_size_t(BYTES_LENGTH_DEBUG_TOF, decoder.message_length());
    DebugTof decoded = {};
    DecodeDebugTof(&decoded, const_cast<uint8_t *>(decoder.message()));
    TEST_ASSERT_EQUAL_UINT32(original.sequence, decoded.sequence);
    TEST_ASSERT_EQUAL_INT16_ARRAY(original.distance_mm, decoded.distance_mm, 64);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(original.target_status, decoded.target_status, 64);
}

// Codec round-trip: a payload with embedded zero bytes must survive
// encode -> feed -> decode with type and content intact.
void test_codec_roundtrip_with_zero_bytes(void)
{
    uint8_t message[64];
    for (size_t i = 0; i < sizeof(message); ++i)
        message[i] = static_cast<uint8_t>((i == 0 || i == 30 || i == 63) ? 0 : i * 3 + 1);

    uint8_t frame[128];
    const size_t frame_len =
      serial_debug::encode_frame(3, message, sizeof(message), frame, sizeof(frame));
    TEST_ASSERT_GREATER_THAN(0, frame_len);

    serial_debug::Decoder decoder;
    for (size_t i = 0; i < frame_len; ++i) decoder.feed(frame[i]);

    TEST_ASSERT_TRUE(decoder.has_frame());
    TEST_ASSERT_EQUAL_UINT8(3, decoder.message_type());
    TEST_ASSERT_EQUAL_size_t(sizeof(message), decoder.message_length());
    TEST_ASSERT_EQUAL_MEMORY(message, decoder.message(), sizeof(message));
}

// Corrupted frames must be rejected: the CRC protects type and payload.
void test_codec_rejects_corrupted_frame(void)
{
    const uint8_t message[] = {1, 2, 3, 4, 5, 6, 7, 8};
    uint8_t frame[32];
    const size_t frame_len =
      serial_debug::encode_frame(2, message, sizeof(message), frame, sizeof(frame));
    TEST_ASSERT_GREATER_THAN(0, frame_len);

    frame[frame_len / 2] ^= 0x40;  // flip one bit mid-frame
    serial_debug::Decoder decoder;
    for (size_t i = 0; i < frame_len; ++i) decoder.feed(frame[i]);
    TEST_ASSERT_FALSE(decoder.has_frame());
}

// ASCII noise between frames must not desynchronize the decoder.
void test_codec_resyncs_after_garbage(void)
{
    const char * garbage = "boot noise without any zero byte in sight";
    const uint8_t message[] = {9, 9};
    uint8_t frame[32];
    const size_t frame_len =
      serial_debug::encode_frame(5, message, sizeof(message), frame, sizeof(frame));

    serial_debug::Decoder decoder;
    for (const char * p = garbage; *p != 0; ++p) decoder.feed(static_cast<uint8_t>(*p));
    decoder.feed(0);  // close the garbage "frame": dropped (COBS/CRC)
    for (size_t i = 0; i < frame_len; ++i) decoder.feed(frame[i]);

    TEST_ASSERT_TRUE(decoder.has_frame());
    TEST_ASSERT_EQUAL_UINT8(5, decoder.message_type());
    TEST_ASSERT_EQUAL_size_t(sizeof(message), decoder.message_length());
}

// A delimiter-less burst longer than the frame buffer must not stall the
// decoder: it resumes collecting at the next delimiter.
void test_codec_recovers_from_overflow(void)
{
    serial_debug::Decoder decoder;
    for (size_t i = 0; i < 1000; ++i) decoder.feed(0x41);
    decoder.feed(0);

    const uint8_t message[] = {7};
    uint8_t frame[16];
    const size_t frame_len =
      serial_debug::encode_frame(4, message, sizeof(message), frame, sizeof(frame));
    for (size_t i = 0; i < frame_len; ++i) decoder.feed(frame[i]);

    TEST_ASSERT_TRUE(decoder.has_frame());
    TEST_ASSERT_EQUAL_UINT8(4, decoder.message_type());
    TEST_ASSERT_EQUAL_size_t(1, decoder.message_length());
}

// Known CRC-8/SMBUS check value: crc8("123456789") == 0xF4.
void test_crc8_known_answer(void)
{
    const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    TEST_ASSERT_EQUAL_HEX8(0xF4, serial_debug::crc8(check, sizeof(check)));
}

// Frames larger than the output buffer must be refused, not truncated.
void test_encode_frame_rejects_small_output(void)
{
    const uint8_t message[] = {1, 2, 3};
    uint8_t frame[4];  // far too small
    TEST_ASSERT_EQUAL_size_t(
      0, serial_debug::encode_frame(1, message, sizeof(message), frame, sizeof(frame)));
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_codec_preserves_full_raw_tof_frame);
    RUN_TEST(test_codec_roundtrip_with_zero_bytes);
    RUN_TEST(test_codec_rejects_corrupted_frame);
    RUN_TEST(test_codec_resyncs_after_garbage);
    RUN_TEST(test_codec_recovers_from_overflow);
    RUN_TEST(test_crc8_known_answer);
    RUN_TEST(test_encode_frame_rejects_small_output);
    UNITY_END();
}

extern "C" void app_main(void)
{
    setup();
}
