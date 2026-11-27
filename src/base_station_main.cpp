/**
 * @file base_station_main.cpp
 * @brief Application entry point for the ESP32 DevKit nRF24L01+ Base Station.
 *
 * Built by PlatformIO env:base_station (selected via build_src_filter and
 * src/CMakeLists.txt when CONFIG_FIRMWARE_ROLE_BASE_STATION=y).
 * Connects directly to an nRF24L01+ module via dedicated SPI + CE/CSN/IRQ GPIOs
 * and bridges COBS+CRC8 Bitproto frames over UART0 at CONFIG_BASE_STATION_UART_BAUD.
 */

#include <driver/gpio.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdkconfig.h>

#include "BaseStationService.h"
#include "NRF24L01.h"
#include "NVSManager.h"
#include "SPIdev.h"

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(NVSManager::init());
    NVSManager::start();

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    SPIdev::init();

    NRF24_CePin tx_ce = {};
    tx_ce.gpio = static_cast<gpio_num_t>(CONFIG_CE_GPIO);
    tx_ce.write = nullptr;
    tx_ce.context = nullptr;

    static NRF24L01 tx_radio(static_cast<gpio_num_t>(CONFIG_IRQ_GPIO),
                             tx_ce,
                             static_cast<gpio_num_t>(CONFIG_CSN_GPIO),
                             static_cast<uint8_t>(CONFIG_NRF24_SPI_DEVICE_KEY));

#if CONFIG_BASE_DUAL_RADIO
    NRF24_CePin rx_ce = {};
    rx_ce.gpio = static_cast<gpio_num_t>(CONFIG_BASE_RX_CE_GPIO);
    rx_ce.write = nullptr;
    rx_ce.context = nullptr;

    static NRF24L01 rx_radio(static_cast<gpio_num_t>(CONFIG_BASE_RX_IRQ_GPIO),
                             rx_ce,
                             static_cast<gpio_num_t>(CONFIG_BASE_RX_CSN_GPIO),
                             static_cast<uint8_t>(CONFIG_BASE_RX_SPI_DEVICE_KEY));

    // Deselect BOTH CSN pins (HIGH) and hold BOTH CE pins (LOW) before any SPI
    // traffic runs on the shared SPI2_HOST bus.
    tx_radio.init_hw();
    rx_radio.init_hw();

    auto & service = BaseStationService::get_instance();
    ESP_ERROR_CHECK(service.init(&tx_radio, &rx_radio));
#else
    tx_radio.init_hw();

    auto & service = BaseStationService::get_instance();
    ESP_ERROR_CHECK(service.init(&tx_radio, nullptr));
#endif
    service.start();

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
