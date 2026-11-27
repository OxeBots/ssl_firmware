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

#ifdef CONFIG_BASE_DEFAULT_MIN_SEP
static constexpr uint8_t BASE_MIN_SEP = CONFIG_BASE_DEFAULT_MIN_SEP;
#else
static constexpr uint8_t BASE_MIN_SEP = radio_addressing::DEFAULT_MIN_CHANNEL_SEP;
#endif

#ifdef CONFIG_BASE_DEFAULT_OPTIMAL_SEP
static constexpr uint8_t BASE_OPT_SEP = CONFIG_BASE_DEFAULT_OPTIMAL_SEP;
#else
static constexpr uint8_t BASE_OPT_SEP = radio_addressing::DEFAULT_OPTIMAL_CHANNEL_SEP;
#endif

static constexpr uint64_t TX_WATCHDOG_INTERVAL_US = 250000ULL;

namespace
{

class TxEpochGuard
{
   public:
    explicit TxEpochGuard(std::atomic<uint32_t> & epoch) : m_epoch(epoch)
    {
        // Odd value signals an active TX burst on the co-located TX radio.
        m_epoch.fetch_add(1U, std::memory_order_release);
    }

    ~TxEpochGuard()
    {
        // Even value signals TX idle.
        m_epoch.fetch_add(1U, std::memory_order_release);
    }

   private:
    std::atomic<uint32_t> & m_epoch;
};

}  // namespace

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

esp_err_t BaseStationService::init(NRF24L01 * tx_radio, NRF24L01 * rx_radio)
{
    if (m_uart_tx_mutex != nullptr)
        return ESP_ERR_INVALID_STATE;

    m_tx_radio = tx_radio;
    m_rx_radio = rx_radio;
    m_dual_radio = false;

    m_uart_tx_mutex = xSemaphoreCreateRecursiveMutex();
    m_scan_req_mutex = xSemaphoreCreateMutex();
    if (m_uart_tx_mutex == nullptr || m_scan_req_mutex == nullptr)
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

    m_router.set_scan_separation_policy(BASE_MIN_SEP, BASE_OPT_SEP);

    base_station::RadioConfigState cfg = {};
    load_config_from_nvs(&cfg);

    const std::array<uint8_t, 5> base_addr =
      radio_addressing::make_base_address(cfg.addr_suffix, cfg.base_addr_lsb);
    const std::array<uint8_t, 5> default_tx =
      radio_addressing::make_robot_address(0, cfg.addr_suffix);

    // Probe dedicated RX radio first when wired (Full-Duplex Dual-Radio mode).
    if (m_rx_radio != nullptr)
    {
        if (m_rx_radio->init_bytes(cfg.rx_channel, 32, default_tx.data(), base_addr.data()) == ESP_OK)
        {
            m_rx_radio->set_rf_params(static_cast<rf24_datarate_e>(cfg.data_rate),
                                      static_cast<rf24_pa_dbm_e>(cfg.pa_level),
                                      cfg.retransmit_delay,
                                      cfg.retransmit_count);
            m_rx_radio->start(&BaseStationService::on_radio_packet, this);
            m_dual_radio = true;
            ESP_LOGI(TAG, "Dedicated RX radio ready (Full-Duplex on rx_ch=%u)", cfg.rx_channel);
        }
        else
        {
            ESP_LOGW(TAG, "RX radio not detected — falling back to single-radio mode (rx_ch == tx_ch)");
            m_dual_radio = false;
            cfg.rx_channel = cfg.channel;
        }
    }
    else
    {
        cfg.rx_channel = cfg.channel;
    }

    if (m_tx_radio != nullptr)
    {
        if (m_dual_radio)
            m_tx_radio->set_tx_only_mode(true);

        if (m_tx_radio->init_bytes(cfg.channel, 32, default_tx.data(), base_addr.data()) == ESP_OK)
        {
            m_tx_radio->set_rf_params(static_cast<rf24_datarate_e>(cfg.data_rate),
                                      static_cast<rf24_pa_dbm_e>(cfg.pa_level),
                                      cfg.retransmit_delay,
                                      cfg.retransmit_count);
            if (!m_dual_radio)
                m_tx_radio->start(&BaseStationService::on_radio_packet, this);
        }
        else
        {
            ESP_LOGE(TAG, "Base Station TX radio initialization failed");
        }
    }

    m_router.set_config_state(cfg);
    m_applied_cfg = cfg;
    m_has_applied_cfg = true;

    ESP_LOGI(TAG,
             "Base Station ready (baud=%d, mode=%s, tx_ch=%u, rx_ch=%u, mask=0x%02X)",
             BASE_UART_BAUD,
             m_dual_radio ? "FULL_DUPLEX_DUAL" : "SINGLE_FALLBACK",
             cfg.channel,
             cfg.rx_channel,
             radio_ready_mask());
    return ESP_OK;
}

void BaseStationService::start()
{
    if (m_dual_radio && m_scan_task_handle == nullptr)
    {
        xTaskCreate(&BaseStationService::scan_task_wrapper,
                    "base_rf_scan",
                    4096,
                    this,
                    3,
                    &m_scan_task_handle);
    }

    if (m_uart_task_handle == nullptr)
    {
        xTaskCreate(&BaseStationService::uart_rx_task_wrapper,
                    "base_uart_rx",
                    6144,
                    this,
                    6,
                    &m_uart_task_handle);
    }
}

uint8_t BaseStationService::radio_ready_mask() const
{
    uint8_t mask = 0;
    if (m_tx_radio != nullptr && m_tx_radio->is_ready())
        mask |= 0x01U;
    if (m_dual_radio && m_rx_radio != nullptr && m_rx_radio->is_ready())
        mask |= 0x02U;
    return mask;
}

bool BaseStationService::is_radio_ready() const
{
    return (radio_ready_mask() & 0x01U) != 0U;
}

uint16_t BaseStationService::get_radio_reinit_count() const
{
    uint16_t total = (m_tx_radio != nullptr) ? m_tx_radio->get_reinit_count() : 0U;
    if (m_dual_radio && m_rx_radio != nullptr)
        total = static_cast<uint16_t>(total + m_rx_radio->get_reinit_count());
    return total;
}

bool BaseStationService::apply_config(const base_station::RadioConfigState & config,
                                      bool persist_nvs)
{
    if (m_tx_radio == nullptr || !m_tx_radio->is_ready())
        return false;

    const bool rf_params_changed =
      !m_has_applied_cfg || config.data_rate != m_applied_cfg.data_rate ||
      config.pa_level != m_applied_cfg.pa_level ||
      config.retransmit_delay != m_applied_cfg.retransmit_delay ||
      config.retransmit_count != m_applied_cfg.retransmit_count;
    const bool addr_changed =
      !m_has_applied_cfg || config.addr_suffix != m_applied_cfg.addr_suffix ||
      config.base_addr_lsb != m_applied_cfg.base_addr_lsb;

    const std::array<uint8_t, 5> base_addr =
      radio_addressing::make_base_address(config.addr_suffix, config.base_addr_lsb);

    if (!m_has_applied_cfg || config.channel != m_applied_cfg.channel)
    {
        if (m_tx_radio->set_channel(config.channel) != ESP_OK)
            return false;
    }
    if (addr_changed && !m_dual_radio)
    {
        if (m_tx_radio->set_rx_address(base_addr.data()) != ESP_OK)
            return false;
    }
    if (rf_params_changed)
    {
        if (m_tx_radio->set_rf_params(static_cast<rf24_datarate_e>(config.data_rate),
                                      static_cast<rf24_pa_dbm_e>(config.pa_level),
                                      config.retransmit_delay,
                                      config.retransmit_count) != ESP_OK)
            return false;
    }

    if (m_dual_radio && m_rx_radio != nullptr && m_rx_radio->is_ready())
    {
        if (!m_has_applied_cfg || config.rx_channel != m_applied_cfg.rx_channel)
        {
            if (m_rx_radio->set_channel(config.rx_channel) != ESP_OK)
                return false;
        }
        if (addr_changed)
        {
            if (m_rx_radio->set_rx_address(base_addr.data()) != ESP_OK)
                return false;
        }
        if (rf_params_changed)
        {
            if (m_rx_radio->set_rf_params(static_cast<rf24_datarate_e>(config.data_rate),
                                          static_cast<rf24_pa_dbm_e>(config.pa_level),
                                          config.retransmit_delay,
                                          config.retransmit_count) != ESP_OK)
                return false;
        }
    }

    m_applied_cfg = config;
    m_has_applied_cfg = true;

    if (persist_nvs)
        save_config_to_nvs(config);
    return true;
}

base_station::TxOutcome BaseStationService::transmit_impl(const uint8_t tx_addr[5],
                                                          bool use_temp_channel,
                                                          uint8_t rf_channel,
                                                          const uint8_t * payload,
                                                          uint8_t len,
                                                          bool no_ack)
{
    base_station::TxOutcome out = {};
    if (m_tx_radio == nullptr || !m_tx_radio->is_ready())
        return out;

    TxEpochGuard epoch_guard(m_tx_epoch);

    NRF24L01::TxResult res = {};
    if (use_temp_channel)
        m_tx_radio->send_to_sync_on_channel(tx_addr, rf_channel, payload, len, no_ack, &res, 10);
    else
        m_tx_radio->send_to_sync(tx_addr, payload, len, no_ack, &res, 10);

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

base_station::TxOutcome BaseStationService::transmit_rf(const uint8_t tx_addr[5],
                                                        const uint8_t * payload,
                                                        uint8_t len,
                                                        bool no_ack)
{
    return transmit_impl(tx_addr, false, 0, payload, len, no_ack);
}

base_station::TxOutcome BaseStationService::transmit_rf_on_channel(const uint8_t tx_addr[5],
                                                                   uint8_t rf_channel,
                                                                   const uint8_t * payload,
                                                                   uint8_t len,
                                                                   bool no_ack)
{
    return transmit_impl(tx_addr, true, rf_channel, payload, len, no_ack);
}

bool BaseStationService::start_async_scan(uint8_t start_ch,
                                          uint8_t end_ch,
                                          uint8_t samples_per_ch)
{
    if (!m_dual_radio || m_rx_radio == nullptr || !m_rx_radio->is_ready() ||
        m_scan_task_handle == nullptr || m_scan_req_mutex == nullptr)
        return false;

    xSemaphoreTake(m_scan_req_mutex, portMAX_DELAY);
    m_pending_scan.start_ch = start_ch;
    m_pending_scan.end_ch = end_ch;
    m_pending_scan.samples_per_ch = samples_per_ch;
    xSemaphoreGive(m_scan_req_mutex);

    xTaskNotifyGive(m_scan_task_handle);
    return true;
}

bool BaseStationService::scan_spectrum(uint8_t start_ch,
                                       uint8_t end_ch,
                                       uint8_t samples_per_ch,
                                       uint8_t * out_busy_pct)
{
    NRF24L01 * scanner = (m_dual_radio && m_rx_radio != nullptr && m_rx_radio->is_ready())
                           ? m_rx_radio
                           : m_tx_radio;
    if (scanner == nullptr || !scanner->is_ready())
        return false;

    const std::atomic<uint32_t> * epoch = m_dual_radio ? &m_tx_epoch : nullptr;
    return scanner->scan_spectrum(start_ch, end_ch, samples_per_ch, out_busy_pct, epoch) == ESP_OK;
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
    out_cfg->rx_channel =
      (CONFIG_RADIO_CHANNEL + BASE_OPT_SEP <= radio_addressing::MAX_CHANNEL)
        ? static_cast<uint8_t>(CONFIG_RADIO_CHANNEL + BASE_OPT_SEP)
        : static_cast<uint8_t>(CONFIG_RADIO_CHANNEL - BASE_OPT_SEP);
    out_cfg->retransmit_delay = CONFIG_RETRANSMIT_DELAY;
    out_cfg->retransmit_count = CONFIG_RETRANSMIT_COUNT;

    uint8_t blob_v2[BYTES_LENGTH_BASE_RADIO_CONFIG] = {};
    size_t len = sizeof(blob_v2);
    if (NVSManager::load_blob(NVS_NS, NVS_KEY_CFG_V2, blob_v2, &len) == ESP_OK &&
        len == sizeof(blob_v2))
    {
        BaseRadioConfig stored = {};
        DecodeBaseRadioConfig(&stored, blob_v2);
        if (radio_addressing::is_valid_channel(stored.channel))
            out_cfg->channel = stored.channel;
        if (stored.rx_channel == radio_addressing::RX_CHANNEL_FOLLOW_TX)
            out_cfg->rx_channel = out_cfg->channel;
        else if (radio_addressing::is_valid_channel(stored.rx_channel))
            out_cfg->rx_channel = stored.rx_channel;
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
        return;
    }

    // Legacy v1 blob migration (11 bytes without rx_channel).
    uint8_t blob_v1[11] = {};
    len = sizeof(blob_v1);
    if (NVSManager::load_blob(NVS_NS, NVS_KEY_CFG_V1, blob_v1, &len) == ESP_OK &&
        len == sizeof(blob_v1))
    {
        if (radio_addressing::is_valid_channel(blob_v1[0]))
        {
            out_cfg->channel = blob_v1[0];
            out_cfg->rx_channel =
              (out_cfg->channel + BASE_OPT_SEP <= radio_addressing::MAX_CHANNEL)
                ? static_cast<uint8_t>(out_cfg->channel + BASE_OPT_SEP)
                : static_cast<uint8_t>(out_cfg->channel - BASE_OPT_SEP);
        }
        if (blob_v1[1] <= BS_RATE_250KBPS)
            out_cfg->data_rate = blob_v1[1];
        if (blob_v1[2] <= 3)
            out_cfg->pa_level = blob_v1[2];
        if (blob_v1[3] <= 15)
            out_cfg->retransmit_delay = blob_v1[3];
        if (blob_v1[4] <= 15)
            out_cfg->retransmit_count = blob_v1[4];
        memcpy(out_cfg->addr_suffix.data(), &blob_v1[6], 4);
        if (blob_v1[10] != 0 && blob_v1[10] != 0xFF)
            out_cfg->base_addr_lsb = blob_v1[10];
    }
}

void BaseStationService::save_config_to_nvs(const base_station::RadioConfigState & cfg)
{
    BaseRadioConfig stored = {};
    stored.channel = cfg.channel;
    stored.rx_channel = cfg.rx_channel;
    stored.data_rate = cfg.data_rate;
    stored.pa_level = cfg.pa_level;
    stored.retransmit_delay = cfg.retransmit_delay;
    stored.retransmit_count = cfg.retransmit_count;
    stored.persist_nvs = 1;
    memcpy(stored.addr_suffix, cfg.addr_suffix.data(), 4);
    stored.base_addr_lsb = cfg.base_addr_lsb;

    uint8_t blob[BYTES_LENGTH_BASE_RADIO_CONFIG] = {};
    EncodeBaseRadioConfig(&stored, blob);
    NVSManager::save_blob(NVS_NS, NVS_KEY_CFG_V2, blob, sizeof(blob));
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
    uint64_t last_tx_health_us = static_cast<uint64_t>(esp_timer_get_time());

    while (true)
    {
        const int n = uart_read_bytes(UART_NUM_0, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(5));
        if (n > 0)
        {
            m_router.feed_uart_bytes(rx_buf, static_cast<size_t>(n));
        }
        else if (m_dual_radio && m_tx_radio != nullptr && m_tx_radio->is_ready())
        {
            const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
            if (now_us >= last_tx_health_us &&
                (now_us - last_tx_health_us) >= TX_WATCHDOG_INTERVAL_US)
            {
                last_tx_health_us = now_us;
                m_tx_radio->reconfigure_if_needed();
            }
        }
    }
}

void BaseStationService::scan_task_wrapper(void * arg)
{
    static_cast<BaseStationService *>(arg)->scan_task();
}

void BaseStationService::scan_task()
{
    uint8_t busy_pct[126] = {};

    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        PendingScanRequest req = {};
        xSemaphoreTake(m_scan_req_mutex, portMAX_DELAY);
        req = m_pending_scan;
        xSemaphoreGive(m_scan_req_mutex);

        const uint8_t count = static_cast<uint8_t>(req.end_ch - req.start_ch + 1);
        memset(busy_pct, 0, sizeof(busy_pct));
        const bool ok =
          m_rx_radio != nullptr && m_rx_radio->is_ready() &&
          m_rx_radio->scan_spectrum(
            req.start_ch, req.end_ch, req.samples_per_ch, busy_pct, &m_tx_epoch) == ESP_OK;

        m_router.emit_scan_result(req.start_ch, count, busy_pct, ok);
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
