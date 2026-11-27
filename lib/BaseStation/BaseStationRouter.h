#ifndef _BASE_STATION_ROUTER_H_
#define _BASE_STATION_ROUTER_H_

#include <stddef.h>
#include <stdint.h>

#include <array>
#include <cstring>

#include "RadioAddressing.h"
#include "SerialFraming.h"
#include "base_station_protocol_bp.h"

namespace base_station
{

static constexpr size_t MAX_MESSAGE_BYTES = BYTES_LENGTH_BASE_SCAN_RESULT;
static constexpr size_t MAX_FRAME_BYTES = serial_framing::max_frame_bytes_for(MAX_MESSAGE_BYTES);

struct RadioConfigState
{
    uint8_t channel = radio_addressing::DEFAULT_CHANNEL;        // TX / downlink channel
    uint8_t rx_channel = radio_addressing::DEFAULT_RX_CHANNEL;  // RX / uplink channel
    uint8_t data_rate = BS_RATE_2MBPS;
    uint8_t pa_level = 3;          // RF24_PA_MAX
    uint8_t retransmit_delay = 1;  // 500 us
    uint8_t retransmit_count = 3;  // fast fail on offline robots
    std::array<uint8_t, 4> addr_suffix = radio_addressing::DEFAULT_ADDR_SUFFIX;
    uint8_t base_addr_lsb = radio_addressing::DEFAULT_BASE_ADDR_LSB;
};

struct TxOutcome
{
    uint8_t status = BS_TX_STATUS_RADIO_ERR;
    uint8_t retries = 0;
    uint16_t tx_time_us = 0;
};

// Abstract backend decoupling protocol routing from ESP-IDF UART/NRF24 hardware.
class IBackend
{
   public:
    virtual ~IBackend() = default;

    // Bitmask: bit 0 = TX radio ready, bit 1 = RX radio ready.
    // 0x03 = Full-Duplex Dual-Radio, 0x01 = Single-Radio fallback.
    virtual uint8_t radio_ready_mask() const
    {
        return is_radio_ready() ? 0x01U : 0x00U;
    }

    virtual bool is_radio_ready() const = 0;
    virtual uint16_t get_radio_reinit_count() const = 0;
    virtual bool apply_config(const RadioConfigState & config, bool persist_nvs) = 0;
    virtual TxOutcome transmit_rf(const uint8_t tx_addr[5],
                                  const uint8_t * payload,
                                  uint8_t len,
                                  bool no_ack) = 0;
    virtual TxOutcome transmit_rf_on_channel(const uint8_t tx_addr[5],
                                             uint8_t rf_channel,
                                             const uint8_t * payload,
                                             uint8_t len,
                                             bool no_ack)
    {
        (void)rf_channel;
        return transmit_rf(tx_addr, payload, len, no_ack);
    }
    // Optional async scan hook (Dual-Radio RX task). When returning true, the backend
    // will invoke Router::emit_scan_result() upon completion without blocking UART RX.
    virtual bool start_async_scan(uint8_t start_ch, uint8_t end_ch, uint8_t samples_per_ch)
    {
        (void)start_ch;
        (void)end_ch;
        (void)samples_per_ch;
        return false;
    }
    virtual bool scan_spectrum(uint8_t start_ch,
                               uint8_t end_ch,
                               uint8_t samples_per_ch,
                               uint8_t * out_busy_pct) = 0;
    virtual void send_uart_frame(const uint8_t * data, size_t len) = 0;
};

class Router
{
   public:
    explicit Router(IBackend * backend) : m_backend(backend)
    {
    }

    void set_config_state(const RadioConfigState & config)
    {
        m_config = config;
    }

    void set_scan_separation_policy(uint8_t min_sep, uint8_t optimal_sep)
    {
        if (min_sep > 0)
            m_min_sep = min_sep;
        if (optimal_sep >= m_min_sep)
            m_optimal_sep = optimal_sep;
    }

    const RadioConfigState & config() const
    {
        return m_config;
    }

    void feed_uart_bytes(const uint8_t * data, size_t len)
    {
        if (data == nullptr)
            return;
        for (size_t i = 0; i < len; ++i)
        {
            m_decoder.feed(data[i]);
            if (m_decoder.has_frame())
            {
                handle_frame(m_decoder.message_type(),
                             m_decoder.message(),
                             m_decoder.message_length());
                m_decoder.pop();
            }
        }
    }

    void on_rf_received(uint32_t rx_timestamp_ms, uint8_t pipe, const uint8_t * payload, uint8_t len)
    {
        if (m_backend == nullptr || payload == nullptr || len == 0 || len > 31)
            return;

        ++m_rx_count;
        BaseRfRx msg = {};
        msg.rx_timestamp_ms = rx_timestamp_ms;
        msg.pipe = pipe;
        msg.payload_len = len;
        memcpy(msg.payload, payload, len);

        uint8_t raw[BYTES_LENGTH_BASE_RF_RX] = {};
        EncodeBaseRfRx(&msg, raw);
        send_message(BS_MSG_RF_RX, raw, sizeof(raw));
    }

    void emit_scan_result(uint8_t start_ch, uint8_t count, const uint8_t * busy_pct, bool ok)
    {
        if (m_backend == nullptr)
            return;

        BaseScanResult res = {};
        res.start_channel = start_ch;
        const bool dual_radio = (m_backend->radio_ready_mask() & 0x03U) == 0x03U;

        if (ok && busy_pct != nullptr && count > 0 && count <= 126)
        {
            res.count = count;
            memcpy(res.busy_pct, busy_pct, count);
            if (dual_radio)
            {
                const radio_addressing::ChannelPair pair =
                  radio_addressing::select_best_channel_pair(
                    res.busy_pct, start_ch, count, m_config.data_rate, m_min_sep, m_optimal_sep);
                res.best_channel = pair.tx_channel;
                res.best_rx_channel = pair.rx_channel;
            }
            else
            {
                res.best_channel = radio_addressing::select_best_channel(
                  res.busy_pct, start_ch, count, m_config.data_rate);
                res.best_rx_channel = res.best_channel;
            }
        }
        else
        {
            res.count = 0;
            res.best_channel = m_config.channel;
            res.best_rx_channel = dual_radio ? m_config.rx_channel : m_config.channel;
        }

        uint8_t raw[BYTES_LENGTH_BASE_SCAN_RESULT] = {};
        EncodeBaseScanResult(&res, raw);
        send_message(BS_MSG_SCAN_RESULT, raw, sizeof(raw));
    }

    void send_info()
    {
        if (m_backend == nullptr)
            return;

        const uint8_t mask = m_backend->radio_ready_mask();
        BaseInfo info = {};
        info.protocol_version = BS_PROTO_VERSION;
        info.radio_ready = mask;
        info.channel = m_config.channel;
        info.rx_channel = ((mask & 0x02U) != 0U) ? m_config.rx_channel : m_config.channel;
        info.data_rate = m_config.data_rate;
        info.pa_level = m_config.pa_level;
        info.retransmit_delay = m_config.retransmit_delay;
        info.retransmit_count = m_config.retransmit_count;
        info.tx_ok_count = m_tx_ok_count;
        info.tx_fail_count = m_tx_fail_count;
        info.rx_count = m_rx_count;
        info.radio_reinit_count = m_backend->get_radio_reinit_count();
        memcpy(info.addr_suffix, m_config.addr_suffix.data(), 4);
        info.base_addr_lsb = m_config.base_addr_lsb;

        uint8_t raw[BYTES_LENGTH_BASE_INFO] = {};
        EncodeBaseInfo(&info, raw);
        send_message(BS_MSG_INFO, raw, sizeof(raw));
    }

    uint32_t tx_ok_count() const
    {
        return m_tx_ok_count;
    }

    uint32_t tx_fail_count() const
    {
        return m_tx_fail_count;
    }

    uint32_t rx_count() const
    {
        return m_rx_count;
    }

   private:
    void send_message(uint8_t msg_type, const uint8_t * raw, size_t raw_len)
    {
        uint8_t frame[MAX_FRAME_BYTES];
        const size_t frame_len =
          serial_framing::encode_frame(msg_type, raw, raw_len, frame, sizeof(frame));
        if (frame_len > 0 && m_backend != nullptr)
            m_backend->send_uart_frame(frame, frame_len);
    }

    void handle_frame(uint8_t msg_type, const uint8_t * payload, size_t len)
    {
        if (m_backend == nullptr)
            return;

        switch (msg_type)
        {
            case BS_MSG_PING:
                if (len == BYTES_LENGTH_BASE_PING)
                    send_info();
                break;

            case BS_MSG_GET_INFO:
                if (len == BYTES_LENGTH_BASE_EMPTY)
                    send_info();
                break;

            case BS_MSG_SET_CONFIG:
                if (len == BYTES_LENGTH_BASE_RADIO_CONFIG)
                    handle_set_config(payload);
                break;

            case BS_MSG_SCAN_REQ:
                if (len == BYTES_LENGTH_BASE_SCAN_REQUEST)
                    handle_scan_request(payload);
                break;

            case BS_MSG_RF_TX:
                if (len == BYTES_LENGTH_BASE_RF_TX)
                    handle_rf_tx(payload);
                break;

            case BS_MSG_RF_TX_CH:
                if (len == BYTES_LENGTH_BASE_RF_TX_ON_CHANNEL)
                    handle_rf_tx_on_channel(payload);
                break;

            default:
                break;
        }
    }

    void handle_set_config(const uint8_t * payload)
    {
        BaseRadioConfig req = {};
        DecodeBaseRadioConfig(&req, const_cast<uint8_t *>(payload));

        const bool dual_radio = (m_backend->radio_ready_mask() & 0x02U) != 0U;
        RadioConfigState next = m_config;
        if (radio_addressing::is_valid_channel(req.channel))
            next.channel = req.channel;

        if (!dual_radio)
        {
            next.rx_channel = next.channel;
        }
        else if (req.rx_channel == radio_addressing::RX_CHANNEL_FOLLOW_TX)
        {
            next.rx_channel = next.channel;
        }
        else if (radio_addressing::is_valid_channel(req.rx_channel))
        {
            next.rx_channel = req.rx_channel;
        }

        if (req.data_rate <= BS_RATE_250KBPS)
            next.data_rate = req.data_rate;
        if (req.pa_level <= 3)
            next.pa_level = req.pa_level;
        if (req.retransmit_delay <= 15)
            next.retransmit_delay = req.retransmit_delay;
        if (req.retransmit_count <= 15)
            next.retransmit_count = req.retransmit_count;

        bool suffix_nonzero = false;
        for (size_t i = 0; i < 4; ++i)
        {
            if (req.addr_suffix[i] != 0)
                suffix_nonzero = true;
        }
        if (suffix_nonzero)
            memcpy(next.addr_suffix.data(), req.addr_suffix, 4);
        if (req.base_addr_lsb != 0 && req.base_addr_lsb != 0xFF)
            next.base_addr_lsb = req.base_addr_lsb;

        if (m_backend->apply_config(next, req.persist_nvs != 0))
            m_config = next;

        send_info();
    }

    void handle_scan_request(const uint8_t * payload)
    {
        BaseScanRequest req = {};
        DecodeBaseScanRequest(&req, const_cast<uint8_t *>(payload));

        const uint8_t start_ch =
          (req.start_channel <= radio_addressing::MAX_CHANNEL) ? req.start_channel : 0;
        const uint8_t end_ch =
          (req.end_channel <= radio_addressing::MAX_CHANNEL && req.end_channel >= start_ch)
            ? req.end_channel
            : radio_addressing::MAX_CHANNEL;
        const uint8_t samples = (req.samples_per_channel > 0) ? req.samples_per_channel : 32;
        const uint8_t count = static_cast<uint8_t>(end_ch - start_ch + 1);

        if (m_backend->start_async_scan(start_ch, end_ch, samples))
            return;

        uint8_t busy_pct[126] = {};
        const bool ok = m_backend->scan_spectrum(start_ch, end_ch, samples, busy_pct);
        emit_scan_result(start_ch, count, busy_pct, ok);
    }

    void dispatch_rf_tx(uint16_t seq,
                        uint8_t target_robot_id,
                        uint8_t tx_flags,
                        bool use_temp_channel,
                        uint8_t rf_channel,
                        const uint8_t * rf_payload,
                        uint8_t payload_len)
    {
        BaseRfTxStatus status_msg = {};
        status_msg.seq = seq;
        status_msg.target_robot_id = target_robot_id;

        const bool valid_channel =
          !use_temp_channel || radio_addressing::is_valid_channel(rf_channel);
        if (!m_backend->is_radio_ready() || payload_len == 0 || payload_len > 31 || !valid_channel)
        {
            status_msg.status = BS_TX_STATUS_RADIO_ERR;
            ++m_tx_fail_count;
        }
        else
        {
            const bool is_broadcast = (target_robot_id == radio_addressing::BROADCAST_ROBOT_ID);
            const bool no_ack = is_broadcast || ((tx_flags & BS_TX_FLAG_NO_ACK) != 0);
            const std::array<uint8_t, 5> dst_addr =
              is_broadcast ? radio_addressing::make_broadcast_address(m_config.addr_suffix)
                           : radio_addressing::make_robot_address(target_robot_id,
                                                                  m_config.addr_suffix);

            const TxOutcome outcome =
              use_temp_channel
                ? m_backend->transmit_rf_on_channel(
                    dst_addr.data(), rf_channel, rf_payload, payload_len, no_ack)
                : m_backend->transmit_rf(dst_addr.data(), rf_payload, payload_len, no_ack);
            status_msg.status = outcome.status;
            status_msg.retries = outcome.retries;
            status_msg.tx_time_us = outcome.tx_time_us;

            if (outcome.status == BS_TX_STATUS_OK || outcome.status == BS_TX_STATUS_NO_ACK_SENT)
                ++m_tx_ok_count;
            else
                ++m_tx_fail_count;
        }

        uint8_t raw[BYTES_LENGTH_BASE_RF_TX_STATUS] = {};
        EncodeBaseRfTxStatus(&status_msg, raw);
        send_message(BS_MSG_RF_TX_STATUS, raw, sizeof(raw));
    }

    void handle_rf_tx(const uint8_t * payload)
    {
        BaseRfTx req = {};
        DecodeBaseRfTx(&req, const_cast<uint8_t *>(payload));
        dispatch_rf_tx(
          req.seq, req.target_robot_id, req.tx_flags, false, 0, req.payload, req.payload_len);
    }

    void handle_rf_tx_on_channel(const uint8_t * payload)
    {
        BaseRfTxOnChannel req = {};
        DecodeBaseRfTxOnChannel(&req, const_cast<uint8_t *>(payload));
        dispatch_rf_tx(req.seq,
                       req.target_robot_id,
                       req.tx_flags,
                       true,
                       req.rf_channel,
                       req.payload,
                       req.payload_len);
    }

    IBackend * m_backend = nullptr;
    RadioConfigState m_config = {};
    uint8_t m_min_sep = radio_addressing::DEFAULT_MIN_CHANNEL_SEP;
    uint8_t m_optimal_sep = radio_addressing::DEFAULT_OPTIMAL_CHANNEL_SEP;
    serial_framing::Decoder<MAX_MESSAGE_BYTES> m_decoder;
    uint32_t m_tx_ok_count = 0;
    uint32_t m_tx_fail_count = 0;
    uint32_t m_rx_count = 0;
};

}  // namespace base_station

#endif  // _BASE_STATION_ROUTER_H_
