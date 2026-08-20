#include "VL53L5CX.h"

#include <esp_log.h>
#include <string.h>

static const char * TAG = "VL53L5CX";

/**
 * @brief Constructs a VL53L5CX driver instance.
 * @param address I2C slave address (default 0x52).
 */
VL53L5CX::VL53L5CX(uint8_t address) : m_dev_addr(address), m_dev_handle(NULL)
{
    memset(&m_dev, 0, sizeof(m_dev));
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

    uint8_t status = vl53l5cx_set_power_mode(&m_dev, VL53L5CX_POWER_MODE_WAKEUP);
    if (status != VL53L5CX_STATUS_OK)
    {
        ESP_LOGE(TAG, "Power mode failed (status %u)", status);
        return ESP_ERR_INVALID_STATE;
    }

    status = vl53l5cx_init(&m_dev);
    if (status != VL53L5CX_STATUS_OK)
    {
        ESP_LOGE(TAG, "Init failed (status %u)", status);
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

/**
 * @brief Checks sensor communication by verifying device and revision IDs.
 * @return true if the sensor responds with expected IDs.
 */
bool VL53L5CX::test_connection()
{
    uint8_t is_alive = 0;
    uint8_t status = vl53l5cx_is_alive(&m_dev, &is_alive);
    return (status == VL53L5CX_STATUS_OK) && (is_alive != 0);
}

/**
 * @brief Starts continuous ranging.
 * @return VL53L5CX_STATUS_OK on success.
 */
uint8_t VL53L5CX::start_ranging()
{
    return vl53l5cx_start_ranging(&m_dev);
}

/**
 * @brief Polls for new ranging data.
 * @param is_ready Set to 1 if data ready, 0 otherwise.
 * @return VL53L5CX_STATUS_OK on success.
 */
uint8_t VL53L5CX::check_data_ready(uint8_t *is_ready)
{
    return vl53l5cx_check_data_ready(&m_dev, is_ready);
}

/**
 * @brief Copies current ranging data into the provided struct.
 * @param results Pointer to output results struct.
 * @return VL53L5CX_STATUS_OK on success.
 */
uint8_t VL53L5CX::get_ranging_data(VL53L5CX_ResultsData *results)
{
    return vl53l5cx_get_ranging_data(&m_dev, results);
}

/**
 * @brief Stops the ranging session.
 * @return VL53L5CX_STATUS_OK on success.
 */
uint8_t VL53L5CX::stop_ranging()
{
    return vl53l5cx_stop_ranging(&m_dev);
}
