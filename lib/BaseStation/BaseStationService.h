#ifndef _BASE_STATION_SERVICE_H_
#define _BASE_STATION_SERVICE_H_

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <cstdarg>

#include "BaseStationRouter.h"
#include "NRF24L01.h"

class BaseStationService : private base_station::IBackend
{
   public:
    static BaseStationService & get_instance();

    esp_err_t init(NRF24L01 * radio);
    void start();

   private:
    BaseStationService();
    ~BaseStationService() override = default;
    BaseStationService(const BaseStationService &) = delete;
    BaseStationService & operator=(const BaseStationService &) = delete;

    // IBackend interface
    bool is_radio_ready() const override;
    uint16_t get_radio_reinit_count() const override;
    bool apply_config(const base_station::RadioConfigState & config, bool persist_nvs) override;
    base_station::TxOutcome transmit_rf(const uint8_t tx_addr[5],
                                        const uint8_t * payload,
                                        uint8_t len,
                                        bool no_ack) override;
    bool scan_spectrum(uint8_t start_ch,
                       uint8_t end_ch,
                       uint8_t samples_per_ch,
                       uint8_t * out_busy_pct) override;
    void send_uart_frame(const uint8_t * data, size_t len) override;

    void load_config_from_nvs(base_station::RadioConfigState * out_cfg);
    void save_config_to_nvs(const base_station::RadioConfigState & cfg);

    static void on_radio_packet(void * pvParameters);
    static void uart_rx_task_wrapper(void * arg);
    void uart_rx_task();

    static int log_vprintf_hook(const char * fmt, va_list args);
    void emit_log_line(uint8_t level, const char * text, size_t len);

    NRF24L01 * m_radio = nullptr;
    base_station::Router m_router;
    SemaphoreHandle_t m_uart_tx_mutex = nullptr;
    TaskHandle_t m_uart_task_handle = nullptr;
    bool m_uart_ready = false;

    static constexpr const char * NVS_NS = "base_rf";
    static constexpr const char * NVS_KEY_CFG = "cfg_v1";
};

#endif  // _BASE_STATION_SERVICE_H_
