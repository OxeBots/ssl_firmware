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

    NRF24_CePin radio_ce = {};
    radio_ce.gpio = static_cast<gpio_num_t>(CONFIG_CE_GPIO);
    radio_ce.write = nullptr;
    radio_ce.context = nullptr;

    static NRF24L01 radio(static_cast<gpio_num_t>(CONFIG_IRQ_GPIO), radio_ce);
    radio.init_hw();

    auto & service = BaseStationService::get_instance();
    ESP_ERROR_CHECK(service.init(&radio));
    service.start();

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
