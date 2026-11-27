#include <string.h>
#include <unity.h>

#include <vector>

#include "BaseStationRouter.h"
#include "SerialFraming.h"

namespace
{

class MockBackend : public base_station::IBackend
{
   public:
    bool radio_ready = true;
    uint8_t ready_mask = 0x01;
    uint16_t reinit_count = 2;
    base_station::RadioConfigState last_config = {};
    bool last_persist = false;
    bool apply_ok = true;

    uint8_t last_tx_addr[5] = {};
    uint8_t last_tx_payload[31] = {};
    uint8_t last_tx_len = 0;
    uint8_t last_tx_channel = 0xFF;
    bool last_no_ack = false;
    base_station::TxOutcome next_outcome = {BS_TX_STATUS_OK, 1, 420};

    std::vector<uint8_t> uart_tx_bytes;

    uint8_t radio_ready_mask() const override
    {
        return radio_ready ? ready_mask : 0x00U;
    }

    bool is_radio_ready() const override
    {
        return radio_ready && ((ready_mask & 0x01U) != 0U);
    }

    uint16_t get_radio_reinit_count() const override
    {
        return reinit_count;
    }

    bool apply_config(const base_station::RadioConfigState & config, bool persist_nvs) override
    {
        last_config = config;
        last_persist = persist_nvs;
        return apply_ok;
    }

    base_station::TxOutcome transmit_rf(const uint8_t tx_addr[5],
                                        const uint8_t * payload,
                                        uint8_t len,
                                        bool no_ack) override
    {
        memcpy(last_tx_addr, tx_addr, 5);
        memcpy(last_tx_payload, payload, len);
        last_tx_len = len;
        last_tx_channel = 0xFF;
        last_no_ack = no_ack;
        return next_outcome;
    }

    base_station::TxOutcome transmit_rf_on_channel(const uint8_t tx_addr[5],
                                                   uint8_t rf_channel,
                                                   const uint8_t * payload,
                                                   uint8_t len,
                                                   bool no_ack) override
    {
        memcpy(last_tx_addr, tx_addr, 5);
        memcpy(last_tx_payload, payload, len);
        last_tx_len = len;
        last_tx_channel = rf_channel;
        last_no_ack = no_ack;
        return next_outcome;
    }

    bool scan_spectrum(uint8_t start_ch,
                       uint8_t end_ch,
                       uint8_t /*samples_per_ch*/,
                       uint8_t * out_busy_pct) override
    {
        const uint8_t count = static_cast<uint8_t>(end_ch - start_ch + 1);
        for (uint8_t i = 0; i < count; ++i)
        {
            const uint8_t ch = static_cast<uint8_t>(start_ch + i);
            out_busy_pct[i] = (ch < 90) ? 70 : 0;
        }
        return true;
    }

    void send_uart_frame(const uint8_t * data, size_t len) override
    {
        uart_tx_bytes.insert(uart_tx_bytes.end(), data, data + len);
    }
};

void feed_encoded(base_station::Router & router,
                  uint8_t msg_type,
                  const uint8_t * raw,
                  size_t raw_len)
{
    uint8_t frame[base_station::MAX_FRAME_BYTES];
    const size_t n = serial_framing::encode_frame(msg_type, raw, raw_len, frame, sizeof(frame));
    TEST_ASSERT_GREATER_THAN(0, n);
    router.feed_uart_bytes(frame, n);
}

bool decode_first(const std::vector<uint8_t> & bytes,
                  uint8_t * out_type,
                  std::vector<uint8_t> * out_payload)
{
    serial_framing::Decoder<base_station::MAX_MESSAGE_BYTES> dec;
    for (uint8_t b : bytes)
    {
        dec.feed(b);
        if (dec.has_frame())
        {
            *out_type = dec.message_type();
            out_payload->assign(dec.message(), dec.message() + dec.message_length());
            dec.pop();
            return true;
        }
    }
    return false;
}

}  // namespace

void test_router_ping_replies_with_base_info(void)
{
    MockBackend backend;
    base_station::Router router(&backend);

    BasePing ping = {1234};
    uint8_t raw[BYTES_LENGTH_BASE_PING] = {};
    EncodeBasePing(&ping, raw);
    feed_encoded(router, BS_MSG_PING, raw, sizeof(raw));

    uint8_t type = 0;
    std::vector<uint8_t> payload;
    TEST_ASSERT_TRUE(decode_first(backend.uart_tx_bytes, &type, &payload));
    TEST_ASSERT_EQUAL_UINT8(BS_MSG_INFO, type);
    TEST_ASSERT_EQUAL_size_t(BYTES_LENGTH_BASE_INFO, payload.size());

    BaseInfo info = {};
    DecodeBaseInfo(&info, payload.data());
    TEST_ASSERT_EQUAL_UINT8(BS_PROTO_VERSION, info.protocol_version);
    TEST_ASSERT_EQUAL_UINT8(1, info.radio_ready);
    TEST_ASSERT_EQUAL_UINT8(radio_addressing::DEFAULT_CHANNEL, info.channel);
    TEST_ASSERT_EQUAL_UINT16(2, info.radio_reinit_count);
}

void test_router_unicast_and_broadcast_addressing(void)
{
    MockBackend backend;
    base_station::Router router(&backend);

    // Unicast to robot 2 -> ROBOT_ADDR_LSB[2] with Auto-ACK enabled.
    BaseRfTx tx = {};
    tx.seq = 7;
    tx.target_robot_id = 2;
    tx.tx_flags = 0;
    tx.payload_len = 5;
    tx.payload[0] = 0x23;
    tx.payload[4] = 0x99;

    uint8_t raw[BYTES_LENGTH_BASE_RF_TX] = {};
    EncodeBaseRfTx(&tx, raw);
    feed_encoded(router, BS_MSG_RF_TX, raw, sizeof(raw));

    TEST_ASSERT_FALSE(backend.last_no_ack);
    TEST_ASSERT_EQUAL_UINT8(radio_addressing::ROBOT_ADDR_LSB[2], backend.last_tx_addr[0]);
    TEST_ASSERT_EQUAL_UINT8(5, backend.last_tx_len);
    TEST_ASSERT_EQUAL_UINT32(1, router.tx_ok_count());

    // Broadcast -> BROADCAST_ADDR_LSB and forced NO_ACK.
    backend.uart_tx_bytes.clear();
    backend.next_outcome = {BS_TX_STATUS_NO_ACK_SENT, 0, 260};
    tx.seq = 8;
    tx.target_robot_id = radio_addressing::BROADCAST_ROBOT_ID;
    memset(raw, 0, sizeof(raw));
    EncodeBaseRfTx(&tx, raw);
    feed_encoded(router, BS_MSG_RF_TX, raw, sizeof(raw));

    TEST_ASSERT_TRUE(backend.last_no_ack);
    TEST_ASSERT_EQUAL_UINT8(radio_addressing::BROADCAST_ADDR_LSB, backend.last_tx_addr[0]);

    uint8_t type = 0;
    std::vector<uint8_t> payload;
    TEST_ASSERT_TRUE(decode_first(backend.uart_tx_bytes, &type, &payload));
    TEST_ASSERT_EQUAL_UINT8(BS_MSG_RF_TX_STATUS, type);

    BaseRfTxStatus st = {};
    DecodeBaseRfTxStatus(&st, payload.data());
    TEST_ASSERT_EQUAL_UINT16(8, st.seq);
    TEST_ASSERT_EQUAL_UINT8(BS_TX_STATUS_NO_ACK_SENT, st.status);
}

void test_router_scan_returns_upper_band_best_channel(void)
{
    MockBackend backend;
    base_station::Router router(&backend);

    BaseScanRequest req = {};
    req.start_channel = 0;
    req.end_channel = 125;
    req.samples_per_channel = 16;

    uint8_t raw[BYTES_LENGTH_BASE_SCAN_REQUEST] = {};
    EncodeBaseScanRequest(&req, raw);
    feed_encoded(router, BS_MSG_SCAN_REQ, raw, sizeof(raw));

    uint8_t type = 0;
    std::vector<uint8_t> payload;
    TEST_ASSERT_TRUE(decode_first(backend.uart_tx_bytes, &type, &payload));
    TEST_ASSERT_EQUAL_UINT8(BS_MSG_SCAN_RESULT, type);
    TEST_ASSERT_EQUAL_size_t(BYTES_LENGTH_BASE_SCAN_RESULT, payload.size());

    BaseScanResult res = {};
    DecodeBaseScanResult(&res, payload.data());
    TEST_ASSERT_EQUAL_UINT8(126, res.count);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT8(100, res.best_channel);
    TEST_ASSERT_LESS_OR_EQUAL_UINT8(124, res.best_channel);
}

void test_router_dual_radio_scan_and_rf_tx_on_channel(void)
{
    MockBackend backend;
    backend.ready_mask = 0x03;  // Full-Duplex Dual-Radio
    base_station::Router router(&backend);

    // 1) Set split-channel config (TX=104, RX=118)
    BaseRadioConfig cfg = {};
    cfg.channel = 104;
    cfg.rx_channel = 118;
    cfg.data_rate = BS_RATE_2MBPS;
    cfg.pa_level = 3;
    cfg.retransmit_delay = 1;
    cfg.retransmit_count = 3;
    cfg.persist_nvs = 1;
    uint8_t cfg_raw[BYTES_LENGTH_BASE_RADIO_CONFIG] = {};
    EncodeBaseRadioConfig(&cfg, cfg_raw);
    feed_encoded(router, BS_MSG_SET_CONFIG, cfg_raw, sizeof(cfg_raw));

    uint8_t type = 0;
    std::vector<uint8_t> payload;
    TEST_ASSERT_TRUE(decode_first(backend.uart_tx_bytes, &type, &payload));
    TEST_ASSERT_EQUAL_UINT8(BS_MSG_INFO, type);
    BaseInfo info = {};
    DecodeBaseInfo(&info, payload.data());
    TEST_ASSERT_EQUAL_UINT8(3, info.radio_ready);
    TEST_ASSERT_EQUAL_UINT8(104, info.channel);
    TEST_ASSERT_EQUAL_UINT8(118, info.rx_channel);

    // 2) Scan in Dual-Radio returns clean pair separated by ~14 MHz (>= 10 MHz)
    backend.uart_tx_bytes.clear();
    BaseScanRequest req = {0, 125, 16};
    uint8_t scan_raw[BYTES_LENGTH_BASE_SCAN_REQUEST] = {};
    EncodeBaseScanRequest(&req, scan_raw);
    feed_encoded(router, BS_MSG_SCAN_REQ, scan_raw, sizeof(scan_raw));

    TEST_ASSERT_TRUE(decode_first(backend.uart_tx_bytes, &type, &payload));
    TEST_ASSERT_EQUAL_UINT8(BS_MSG_SCAN_RESULT, type);
    BaseScanResult res = {};
    DecodeBaseScanResult(&res, payload.data());
    const uint8_t sep = (res.best_rx_channel > res.best_channel)
                          ? static_cast<uint8_t>(res.best_rx_channel - res.best_channel)
                          : static_cast<uint8_t>(res.best_channel - res.best_rx_channel);
    TEST_ASSERT_EQUAL_UINT8(14, sep);

    // 3) BS_MSG_RF_TX_CH routes packet to temporary old channel without changing active config
    backend.uart_tx_bytes.clear();
    BaseRfTxOnChannel tx_ch = {};
    tx_ch.seq = 42;
    tx_ch.target_robot_id = 1;
    tx_ch.tx_flags = 0;
    tx_ch.rf_channel = 76;
    tx_ch.payload_len = 8;
    tx_ch.payload[0] = 0x11;
    uint8_t tx_raw[BYTES_LENGTH_BASE_RF_TX_ON_CHANNEL] = {};
    EncodeBaseRfTxOnChannel(&tx_ch, tx_raw);
    feed_encoded(router, BS_MSG_RF_TX_CH, tx_raw, sizeof(tx_raw));

    TEST_ASSERT_EQUAL_UINT8(76, backend.last_tx_channel);
    TEST_ASSERT_EQUAL_UINT8(104, router.config().channel);
    TEST_ASSERT_EQUAL_UINT8(118, router.config().rx_channel);
}

extern "C" void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_router_ping_replies_with_base_info);
    RUN_TEST(test_router_unicast_and_broadcast_addressing);
    RUN_TEST(test_router_scan_returns_upper_band_best_channel);
    RUN_TEST(test_router_dual_radio_scan_and_rf_tx_on_channel);
    UNITY_END();
}
