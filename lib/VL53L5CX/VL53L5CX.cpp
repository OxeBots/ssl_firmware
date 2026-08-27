#include "VL53L5CX.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

static const char * TAG = "VL53L5CX";

/**
 * @brief Constructs a VL53L5CX driver instance.
 * @param address I2C slave address (default 0x52).
 */
VL53L5CX::VL53L5CX(uint8_t address) : m_dev_addr(address)
{
    memset(&m_dev, 0, sizeof(m_dev));
}

VL53L5CX::~VL53L5CX()
{
    if (m_ranging)
        vl53l5cx_stop_ranging(&m_dev);
    if (m_dev_handle != nullptr)
        i2c_master_bus_rm_device(m_dev_handle);
}

/**
 * @brief Creates an I2C device on the bus and loads sensor firmware.
 *
 * Wakes the sensor from sleep, then loads firmware via the ST API.
 * Must be called once before ranging.
 *
 * @param bus_handle Handle to an existing I2C master bus.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t VL53L5CX::init(i2c_master_bus_handle_t bus_handle)
{
    if (bus_handle == nullptr)
        return ESP_ERR_INVALID_ARG;
    if (m_initialized)
        return ESP_ERR_INVALID_STATE;

    i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = static_cast<uint16_t>(m_dev_addr >> 1),
      .scl_speed_hz = 1000000,
      .scl_wait_us = 0,
      .flags = {.disable_ack_check = 0},
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &m_dev_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add I2C device (0x%02X)", m_dev_addr);
        return ret;
    }

    m_dev.platform.handle = m_dev_handle;
    m_dev.platform.address = m_dev_addr;
    m_dev.platform.reset_gpio = GPIO_NUM_NC;

    uint8_t is_alive = 0;
    uint8_t status = VL53L5CX_STATUS_ERROR;
    for (uint8_t attempt = 0; attempt < 3; ++attempt)
    {
        status = vl53l5cx_is_alive(&m_dev, &is_alive);
        if (status == VL53L5CX_STATUS_OK && is_alive != 0)
            break;
        if (attempt < 2)
            vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (status != VL53L5CX_STATUS_OK || is_alive == 0)
    {
        ESP_LOGE(TAG, "VL53L5CX was not detected at 0x%02X", m_dev_addr >> 1);
        i2c_master_bus_rm_device(m_dev_handle);
        m_dev_handle = nullptr;
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t power_mode = 0;
    status = vl53l5cx_get_power_mode(&m_dev, &power_mode);
    if (status != VL53L5CX_STATUS_OK)
    {
        // The sensor can remain streaming when the ESP32 resets because its
        // power rail is independent. Stop that session before reinitializing.
        ESP_LOGW(TAG, "Recovering VL53L5CX from an active ranging session.");
        const uint8_t stop_status = vl53l5cx_stop_ranging(&m_dev);
        if (stop_status != VL53L5CX_STATUS_OK)
            ESP_LOGW(TAG, "Ranging recovery returned status %u.", stop_status);
        status = vl53l5cx_get_power_mode(&m_dev, &power_mode);
    }
    if (status != VL53L5CX_STATUS_OK)
    {
        ESP_LOGE(TAG, "Power state query failed (status %u)", status);
        i2c_master_bus_rm_device(m_dev_handle);
        m_dev_handle = nullptr;
        return ESP_ERR_INVALID_STATE;
    }

    status = vl53l5cx_set_power_mode(&m_dev, VL53L5CX_POWER_MODE_WAKEUP);
    if (status != VL53L5CX_STATUS_OK)
    {
        ESP_LOGE(TAG, "Power mode failed (status %u)", status);
        return ESP_ERR_INVALID_STATE;
    }

    status = vl53l5cx_init(&m_dev);
    if (status != VL53L5CX_STATUS_OK)
    {
        ESP_LOGE(TAG, "Init failed (status %u)", status);
        i2c_master_bus_rm_device(m_dev_handle);
        m_dev_handle = nullptr;
        return ESP_ERR_INVALID_STATE;
    }

    m_initialized = true;
    return ESP_OK;
}

/**
 * @brief Checks sensor communication by verifying device and revision IDs.
 * @return true if the sensor responds with expected IDs.
 */
bool VL53L5CX::test_connection()
{
    if (!m_initialized)
        return false;

    uint8_t is_alive = 0;
    uint8_t status = vl53l5cx_is_alive(&m_dev, &is_alive);
    return (status == VL53L5CX_STATUS_OK) && (is_alive != 0);
}

uint8_t VL53L5CX::set_resolution(uint8_t resolution)
{
    if (!m_initialized || m_ranging)
        return VL53L5CX_STATUS_ERROR;
    return vl53l5cx_set_resolution(&m_dev, resolution);
}

uint8_t VL53L5CX::set_ranging_frequency(uint8_t frequency_hz)
{
    if (!m_initialized || m_ranging)
        return VL53L5CX_STATUS_ERROR;
    return vl53l5cx_set_ranging_frequency_hz(&m_dev, frequency_hz);
}

/**
 * @brief Starts continuous ranging.
 * @return VL53L5CX_STATUS_OK on success.
 */
uint8_t VL53L5CX::start_ranging()
{
    if (!m_initialized)
        return VL53L5CX_STATUS_ERROR;
    uint8_t status = vl53l5cx_start_ranging(&m_dev);
    if (status == VL53L5CX_STATUS_OK)
        m_ranging = true;
    return status;
}

/**
 * @brief Polls for new ranging data.
 * @param is_ready Set to 1 if data ready, 0 otherwise.
 * @return VL53L5CX_STATUS_OK on success.
 */
uint8_t VL53L5CX::check_data_ready(uint8_t * is_ready)
{
    if (!m_initialized || is_ready == nullptr)
        return VL53L5CX_STATUS_ERROR;
    return vl53l5cx_check_data_ready(&m_dev, is_ready);
}

/**
 * @brief Copies current ranging data into the provided struct.
 * @param results Pointer to output results struct.
 * @return VL53L5CX_STATUS_OK on success.
 */
uint8_t VL53L5CX::get_ranging_data(VL53L5CX_ResultsData * results)
{
    if (!m_initialized || results == nullptr)
        return VL53L5CX_STATUS_ERROR;
    return vl53l5cx_get_ranging_data(&m_dev, results);
}

/**
 * @brief Stops the ranging session.
 * @return VL53L5CX_STATUS_OK on success.
 */
uint8_t VL53L5CX::stop_ranging()
{
    if (!m_initialized)
        return VL53L5CX_STATUS_ERROR;
    uint8_t status = vl53l5cx_stop_ranging(&m_dev);
    if (status == VL53L5CX_STATUS_OK)
        m_ranging = false;
    return status;
}
