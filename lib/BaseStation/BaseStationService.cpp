#include "BaseStationService.h"

#include <driver/uart.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <sdkconfig.h>

#include <cstdio>
#include <cstring>

#include "NVSManager.h"

static const char * TAG = "BaseStation";

#ifdef CONFIG_BASE_STATION_UART_BAUD
static constexpr int BASE_UART_BAUD = CONFIG_BASE_STATION_UART_BAUD;
#else
static constexpr int BASE_UART_BAUD = 921600;
#endif

static uint8_t parse_log_level(const char * line, size_t len)
{
    if (len > 0)
    {
        if (line[0] == 'E')
            return 1;
        if (line[0] == 'W')
            return 2;
        if (line[0] == 'I')
            return 3;
        if (line[0] == 'D' || line[0] == 'V')
            return 4;
    }
    return 3;
}

BaseStationService & BaseStationService::get_instance()
{
    static BaseStationService instance;
    return instance;
}

BaseStationService::BaseStationService() : m_router(this)
{
}

esp_err_t BaseStationService::init(NRF24L01 * radio)
{
    if (m_uart_tx_mutex != nullptr)
        return ESP_ERR_INVALID_STATE;

    m_radio = radio;
    m_uart_tx_mutex = xSemaphoreCreateRecursiveMutex();
    if (m_uart_tx_mutex == nullptr)
        return ESP_ERR_NO_MEM;

    // Configure UART0 for high-speed COBS binary framing.
    uart_config_t uart_cfg = {};
    uart_cfg.baud_rate = BASE_UART_BAUD;
    uart_cfg.data_bits = UART_DATA_8_BITS;
    uart_cfg.parity = UART_PARITY_DISABLE;
    uart_cfg.stop_bits = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg.source_clk = UART_SCLK_DEFAULT;

    esp_err_t err = uart_driver_install(UART_NUM_0, 2048, 2048, 0, nullptr, 0);
    if (err != ESP_OK)
        return err;
    err = uart_param_config(UART_NUM_0, &uart_cfg);
    if (err != ESP_OK)
        return err;
    m_uart_ready = true;

    // Encapsulate ESP_LOG lines as BS_MSG_LOG COBS frames so UART0 stays clean binary.
    esp_log_set_vprintf(&BaseStationService::log_vprintf_hook);

    base_station::RadioConfigState cfg = {};
    load_config_from_nvs(&cfg);
    m_router.set_config_state(cfg);

    if (m_radio != nullptr)
    {
        const std::array<uint8_t, 5> base_addr =
          radio_addressing::make_base_address(cfg.addr_suffix, cfg.base_addr_lsb);
        const std::array<uint8_t, 5> default_tx =
          radio_addressing::make_robot_address(0, cfg.addr_suffix);

        if (m_radio->init_bytes(cfg.channel, 32, default_tx.data(), base_addr.data()) == ESP_OK)
        {
            m_radio->set_rf_params(static_cast<rf24_datarate_e>(cfg.data_rate),
                                   static_cast<rf24_pa_dbm_e>(cfg.pa_level),
                                   cfg.retransmit_delay,
                                   cfg.retransmit_count);
            m_radio->start(&BaseStationService::on_radio_packet, this);
        }
        else
        {
            ESP_LOGE(TAG, "Base Station radio initialization failed");
        }
    }

    ESP_LOGI(TAG, "Base Station ready (baud=%d, ch=%u)", BASE_UART_BAUD, cfg.channel);
    return ESP_OK;
}

void BaseStationService::start()
{
    if (m_uart_task_handle != nullptr)
        return;

    xTaskCreate(&BaseStationService::uart_rx_task_wrapper,
                "base_uart_rx",
                6144,
                this,
                6,
                &m_uart_task_handle);
}

bool BaseStationService::is_radio_ready() const
{
    return m_radio != nullptr && m_radio->is_ready();
}

uint16_t BaseStationService::get_radio_reinit_count() const
{
    return (m_radio != nullptr) ? m_radio->get_reinit_count() : 0U;
}

bool BaseStationService::apply_config(const base_station::RadioConfigState & config,
                                      bool persist_nvs)
{
    if (m_radio == nullptr || !m_radio->is_ready())
        return false;

    const std::array<uint8_t, 5> base_addr =
      radio_addressing::make_base_address(config.addr_suffix, config.base_addr_lsb);

    if (m_radio->set_channel(config.channel) != ESP_OK)
        return false;
    if (m_radio->set_rx_address(base_addr.data()) != ESP_OK)
        return false;
    if (m_radio->set_rf_params(static_cast<rf24_datarate_e>(config.data_rate),
                               static_cast<rf24_pa_dbm_e>(config.pa_level),
                               config.retransmit_delay,
                               config.retransmit_count) != ESP_OK)
        return false;

    if (persist_nvs)
        save_config_to_nvs(config);
    return true;
}

base_station::TxOutcome BaseStationService::transmit_rf(const uint8_t tx_addr[5],
                                                        const uint8_t * payload,
                                                        uint8_t len,
                                                        bool no_ack)
{
    base_station::TxOutcome out = {};
    if (m_radio == nullptr || !m_radio->is_ready())
        return out;

    NRF24L01::TxResult res = {};
    m_radio->send_to_sync(tx_addr, payload, len, no_ack, &res, 10);

    switch (res.status)
    {
        case NRF24L01::TxStatus::OK:
            out.status = BS_TX_STATUS_OK;
            break;
        case NRF24L01::TxStatus::MAX_RETRIES:
            out.status = BS_TX_STATUS_MAX_RT;
            break;
        case NRF24L01::TxStatus::NO_ACK_SENT:
            out.status = BS_TX_STATUS_NO_ACK_SENT;
            break;
        default:
            out.status = BS_TX_STATUS_RADIO_ERR;
            break;
    }
    out.retries = res.retries;
    out.tx_time_us = res.tx_time_us;
    return out;
}

bool BaseStationService::scan_spectrum(uint8_t start_ch,
                                       uint8_t end_ch,
                                       uint8_t samples_per_ch,
                                       uint8_t * out_busy_pct)
{
    if (m_radio == nullptr || !m_radio->is_ready())
        return false;
    return m_radio->scan_spectrum(start_ch, end_ch, samples_per_ch, out_busy_pct) == ESP_OK;
}

void BaseStationService::send_uart_frame(const uint8_t * data, size_t len)
{
    if (!m_uart_ready || m_uart_tx_mutex == nullptr || data == nullptr || len == 0)
        return;

    if (xSemaphoreTakeRecursive(m_uart_tx_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        uart_write_bytes(UART_NUM_0, reinterpret_cast<const char *>(data), len);
        xSemaphoreGiveRecursive(m_uart_tx_mutex);
    }
}

void BaseStationService::load_config_from_nvs(base_station::RadioConfigState * out_cfg)
{
    if (out_cfg == nullptr)
        return;

    *out_cfg = base_station::RadioConfigState{};
    out_cfg->channel = CONFIG_RADIO_CHANNEL;
    out_cfg->retransmit_delay = CONFIG_RETRANSMIT_DELAY;
    out_cfg->retransmit_count = CONFIG_RETRANSMIT_COUNT;

    uint8_t blob[BYTES_LENGTH_BASE_RADIO_CONFIG] = {};
    size_t len = sizeof(blob);
    if (NVSManager::load_blob(NVS_NS, NVS_KEY_CFG, blob, &len) == ESP_OK && len == sizeof(blob))
    {
        BaseRadioConfig stored = {};
        DecodeBaseRadioConfig(&stored, blob);
        if (radio_addressing::is_valid_channel(stored.channel))
            out_cfg->channel = stored.channel;
        if (stored.data_rate <= BS_RATE_250KBPS)
            out_cfg->data_rate = stored.data_rate;
        if (stored.pa_level <= 3)
            out_cfg->pa_level = stored.pa_level;
        if (stored.retransmit_delay <= 15)
            out_cfg->retransmit_delay = stored.retransmit_delay;
        if (stored.retransmit_count <= 15)
            out_cfg->retransmit_count = stored.retransmit_count;
        memcpy(out_cfg->addr_suffix.data(), stored.addr_suffix, 4);
        if (stored.base_addr_lsb != 0 && stored.base_addr_lsb != 0xFF)
            out_cfg->base_addr_lsb = stored.base_addr_lsb;
    }
}

void BaseStationService::save_config_to_nvs(const base_station::RadioConfigState & cfg)
{
    BaseRadioConfig stored = {};
    stored.channel = cfg.channel;
    stored.data_rate = cfg.data_rate;
    stored.pa_level = cfg.pa_level;
    stored.retransmit_delay = cfg.retransmit_delay;
    stored.retransmit_count = cfg.retransmit_count;
    stored.persist_nvs = 1;
    memcpy(stored.addr_suffix, cfg.addr_suffix.data(), 4);
    stored.base_addr_lsb = cfg.base_addr_lsb;

    uint8_t blob[BYTES_LENGTH_BASE_RADIO_CONFIG] = {};
    EncodeBaseRadioConfig(&stored, blob);
    NVSManager::save_blob(NVS_NS, NVS_KEY_CFG, blob, sizeof(blob));
}

void BaseStationService::on_radio_packet(void * pvParameters)
{
    auto * event = static_cast<DataReceivedEvent *>(pvParameters);
    if (event == nullptr || event->context == nullptr)
        return;

    auto * self = static_cast<BaseStationService *>(event->context);
    const uint32_t now_ms =
      static_cast<uint32_t>((esp_timer_get_time() / 1000ULL) & 0xFFFFFFFFULL);
    self->m_router.on_rf_received(now_ms, event->pipe, event->payload, event->length);
}

void BaseStationService::uart_rx_task_wrapper(void * arg)
{
    static_cast<BaseStationService *>(arg)->uart_rx_task();
}

void BaseStationService::uart_rx_task()
{
    uint8_t rx_buf[256];
    while (true)
    {
        const int n = uart_read_bytes(UART_NUM_0, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(5));
        if (n > 0)
            m_router.feed_uart_bytes(rx_buf, static_cast<size_t>(n));
    }
}

void BaseStationService::emit_log_line(uint8_t level, const char * text, size_t len)
{
    if (!m_uart_ready || text == nullptr || len == 0)
        return;

    BaseLog msg = {};
    msg.level = level;
    const size_t copy_len = (len > 96) ? 96 : len;
    msg.text_len = static_cast<uint8_t>(copy_len);
    memcpy(msg.text, text, copy_len);

    uint8_t raw[BYTES_LENGTH_BASE_LOG] = {};
    EncodeBaseLog(&msg, raw);

    uint8_t frame[base_station::MAX_FRAME_BYTES];
    const size_t frame_len =
      serial_framing::encode_frame(BS_MSG_LOG, raw, sizeof(raw), frame, sizeof(frame));
    if (frame_len > 0)
        send_uart_frame(frame, frame_len);
}

int BaseStationService::log_vprintf_hook(const char * fmt, va_list args)
{
    char buf[128];
    const int n = vsnprintf(buf, sizeof(buf), fmt, args);
    if (n <= 0)
        return n;

    size_t len = (static_cast<size_t>(n) < sizeof(buf)) ? static_cast<size_t>(n) : sizeof(buf) - 1;
    while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r'))
        --len;

    if (len > 0)
        BaseStationService::get_instance().emit_log_line(parse_log_level(buf, len), buf, len);
    return n;
}
