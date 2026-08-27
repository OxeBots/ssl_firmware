/**
 * @file SPIdev.cpp
 * @brief Shared raw SPI transactions for devices on one ESP-IDF SPI bus.
 */

#include "SPIdev.h"

#include <esp_log.h>
#include <stdlib.h>
#include <string.h>

static const char * TAG = "SPIdev";

spi_host_device_t SPIdev::global_host_id = SPI2_HOST;
bool SPIdev::initialized = false;
spi_dev_cache_entry_t SPIdev::device_cache[SPIDEV_MAX_CACHED_DEVICES] = {};
uint8_t SPIdev::cached_count = 0;
SemaphoreHandle_t SPIdev::transaction_mutex = nullptr;

void SPIdev::init(spi_host_device_t host_id)
{
    global_host_id = host_id;
    initialized = true;
    cached_count = 0;
    memset(device_cache, 0, sizeof(device_cache));
    if (transaction_mutex == nullptr)
        transaction_mutex = xSemaphoreCreateMutex();
}

bool SPIdev::is_initialized()
{
    return initialized;
}

bool SPIdev::register_device(uint8_t device_key, const spi_device_interface_config_t * dev_config)
{
    if (!initialized || dev_config == nullptr)
        return false;

    for (uint8_t i = 0; i < cached_count; ++i)
    {
        if (device_cache[i].device_key != device_key)
            continue;

        if (device_cache[i].handle != nullptr)
            return false;

        device_cache[i].config = *dev_config;
        device_cache[i].has_config = true;
        return true;
    }

    if (cached_count >= SPIDEV_MAX_CACHED_DEVICES)
    {
        ESP_LOGE(TAG, "SPI device cache is full");
        return false;
    }

    device_cache[cached_count].device_key = device_key;
    device_cache[cached_count].config = *dev_config;
    device_cache[cached_count].has_config = true;
    ++cached_count;
    return true;
}

spi_device_handle_t SPIdev::get_device_handle(uint8_t device_key)
{
    if (!initialized)
    {
        ESP_LOGE(TAG, "SPIdev is not initialized");
        return nullptr;
    }

    for (uint8_t i = 0; i < cached_count; ++i)
    {
        auto & entry = device_cache[i];
        if (entry.device_key != device_key)
            continue;

        if (entry.handle != nullptr)
            return entry.handle;

        spi_device_interface_config_t config = entry.config;
        if (config.queue_size == 0)
            config.queue_size = 1;

        esp_err_t err = spi_bus_add_device(global_host_id, &config, &entry.handle);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to add device key %u: %s", device_key, esp_err_to_name(err));
            return nullptr;
        }
        return entry.handle;
    }

    if (cached_count >= SPIDEV_MAX_CACHED_DEVICES)
    {
        ESP_LOGE(TAG, "SPI device cache is full");
        return nullptr;
    }

    spi_device_interface_config_t config = {};
    config.clock_speed_hz = SPIDEV_DEFAULT_CLOCK_SPEED;
    config.mode = 0;
    config.spics_io_num = device_key;
    config.queue_size = 1;

    spi_device_handle_t handle = nullptr;
    esp_err_t err = spi_bus_add_device(global_host_id, &config, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add device key %u: %s", device_key, esp_err_to_name(err));
        return nullptr;
    }

    device_cache[cached_count].device_key = device_key;
    device_cache[cached_count].handle = handle;
    ++cached_count;
    return handle;
}

esp_err_t SPIdev::perform_transaction(uint8_t device_key,
                                      const uint8_t * tx_data,
                                      uint8_t * rx_data,
                                      size_t length)
{
    if (length == 0 || (tx_data == nullptr && rx_data == nullptr))
        return ESP_ERR_INVALID_ARG;

    if (transaction_mutex == nullptr)
        return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(transaction_mutex, portMAX_DELAY) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    spi_device_handle_t handle = get_device_handle(device_key);
    if (handle == nullptr)
    {
        xSemaphoreGive(transaction_mutex);
        return ESP_FAIL;
    }

    uint8_t * zero_tx = nullptr;
    if (tx_data == nullptr)
    {
        zero_tx = static_cast<uint8_t *>(calloc(length, sizeof(uint8_t)));
        if (zero_tx == nullptr)
        {
            xSemaphoreGive(transaction_mutex);
            return ESP_ERR_NO_MEM;
        }
        tx_data = zero_tx;
    }

    spi_transaction_t transaction = {};
    transaction.length = length * 8;
    transaction.tx_buffer = tx_data;
    transaction.rx_buffer = rx_data;

    esp_err_t err = spi_device_transmit(handle, &transaction);
    free(zero_tx);
    xSemaphoreGive(transaction_mutex);
    return err;
}

uint8_t SPIdev::shift(uint8_t device_key, uint8_t data)
{
    uint8_t received = 0;
    uint8_t transmitted = data;
    if (perform_transaction(device_key, &transmitted, &received, 1) != ESP_OK)
        return 0;
    return received;
}

bool SPIdev::read(uint8_t device_key, const uint8_t * tx_data, uint8_t * rx_data, size_t length)
{
    if (rx_data == nullptr || length == 0)
        return false;
    return perform_transaction(device_key, tx_data, rx_data, length) == ESP_OK;
}

bool SPIdev::write(uint8_t device_key, const uint8_t * tx_data, size_t length)
{
    if (tx_data == nullptr || length == 0)
        return false;
    return perform_transaction(device_key, tx_data, nullptr, length) == ESP_OK;
}
