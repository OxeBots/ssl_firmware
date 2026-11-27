#ifndef _BASE_STATION_SERVICE_H_
#define _BASE_STATION_SERVICE_H_

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <atomic>
#include <cstdarg>

#include "BaseStationRouter.h"
#include "NRF24L01.h"

class BaseStationService : private base_station::IBackend
{
   public:
    static BaseStationService & get_instance();

    esp_err_t init(NRF24L01 * tx_radio, NRF24L01 * rx_radio = nullptr);
    void start();

   private:
    struct PendingScanRequest
    {
        uint8_t start_ch = 0;
        uint8_t end_ch = 125;
        uint8_t samples_per_ch = 32;
    };

    BaseStationService();
    ~BaseStationService() override = default;
    BaseStationService(const BaseStationService &) = delete;
    BaseStationService & operator=(const BaseStationService &) = delete;

    // IBackend interface
    uint8_t radio_ready_mask() const override;
    bool is_radio_ready() const override;
    uint16_t get_radio_reinit_count() const override;
    bool apply_config(const base_station::RadioConfigState & config, bool persist_nvs) override;
    base_station::TxOutcome transmit_rf(const uint8_t tx_addr[5],
                                        const uint8_t * payload,
                                        uint8_t len,
                                        bool no_ack) override;
    base_station::TxOutcome transmit_rf_on_channel(const uint8_t tx_addr[5],
                                                   uint8_t rf_channel,
                                                   const uint8_t * payload,
                                                   uint8_t len,
                                                   bool no_ack) override;
    bool start_async_scan(uint8_t start_ch, uint8_t end_ch, uint8_t samples_per_ch) override;
    bool scan_spectrum(uint8_t start_ch,
                       uint8_t end_ch,
                       uint8_t samples_per_ch,
                       uint8_t * out_busy_pct) override;
    void send_uart_frame(const uint8_t * data, size_t len) override;

    base_station::TxOutcome transmit_impl(const uint8_t tx_addr[5],
                                          bool use_temp_channel,
                                          uint8_t rf_channel,
                                          const uint8_t * payload,
                                          uint8_t len,
                                          bool no_ack);

    void load_config_from_nvs(base_station::RadioConfigState * out_cfg);
    void save_config_to_nvs(const base_station::RadioConfigState & cfg);

    static void on_radio_packet(void * pvParameters);
    static void uart_rx_task_wrapper(void * arg);
    void uart_rx_task();
    static void scan_task_wrapper(void * arg);
    void scan_task();

    static int log_vprintf_hook(const char * fmt, va_list args);
    void emit_log_line(uint8_t level, const char * text, size_t len);

    NRF24L01 * m_tx_radio = nullptr;
    NRF24L01 * m_rx_radio = nullptr;
    bool m_dual_radio = false;
    base_station::Router m_router;
    base_station::RadioConfigState m_applied_cfg = {};
    bool m_has_applied_cfg = false;

    SemaphoreHandle_t m_uart_tx_mutex = nullptr;
    SemaphoreHandle_t m_scan_req_mutex = nullptr;
    TaskHandle_t m_uart_task_handle = nullptr;
    TaskHandle_t m_scan_task_handle = nullptr;
    PendingScanRequest m_pending_scan = {};
    std::atomic<uint32_t> m_tx_epoch{0};
    bool m_uart_ready = false;

    static constexpr const char * NVS_NS = "base_rf";
    static constexpr const char * NVS_KEY_CFG_V1 = "cfg_v1";
    static constexpr const char * NVS_KEY_CFG_V2 = "cfg_v2";
};

#endif  // _BASE_STATION_SERVICE_H_
