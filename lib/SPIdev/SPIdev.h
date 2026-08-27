#ifndef SPIDEV_H
#define SPIDEV_H

#include <driver/spi_master.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stddef.h>
#include <stdint.h>

#define SPIDEV_MAX_CACHED_DEVICES 8
#define SPIDEV_DEFAULT_CLOCK_SPEED 1000000

typedef struct
{
    uint8_t device_key;
    spi_device_handle_t handle;
    bool has_config;
    spi_device_interface_config_t config;
} spi_dev_cache_entry_t;

class SPIdev
{
   public:
    SPIdev() = default;

    // The caller owns and initializes the SPI bus before this call.
    static void init(spi_host_device_t host_id);

    // Register a device before its first transaction. The key is a software
    // identifier and does not have to be a chip-select GPIO.
    static bool register_device(uint8_t device_key,
                                const spi_device_interface_config_t * dev_config);

    static bool is_initialized();

    static uint8_t shift(uint8_t device_key, uint8_t data);
    static bool read(uint8_t device_key, const uint8_t * tx_data, uint8_t * rx_data, size_t length);
    static bool write(uint8_t device_key, const uint8_t * tx_data, size_t length);

   private:
    static spi_host_device_t global_host_id;
    static bool initialized;
    static spi_dev_cache_entry_t device_cache[SPIDEV_MAX_CACHED_DEVICES];
    static uint8_t cached_count;
    static SemaphoreHandle_t transaction_mutex;

    static spi_device_handle_t get_device_handle(uint8_t device_key);
    static esp_err_t perform_transaction(uint8_t device_key,
                                         const uint8_t * tx_data,
                                         uint8_t * rx_data,
                                         size_t length);
};

#endif  // SPIDEV_H
