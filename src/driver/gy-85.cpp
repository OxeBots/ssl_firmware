#include "driver/gy-85.hpp"

// --- Inicialização dos dispositivos ---
esp_err_t GY85_I2C::init(I2C_Wrapper & i2c_wrapper)
{
    if (m_initialized)
        return ESP_OK;

    if (!i2c_wrapper.is_initialized())
    {
        ESP_LOGE(TAG, "I2C_Wrapper not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    m_i2c_wrapper = &i2c_wrapper;
    esp_err_t ret;

    // Configure ADXL345
    uint8_t power_on = 0x08;
    ret = m_i2c_wrapper->write_register(ADXL345_ADDR, 0x2D, &power_on, 1);  // Power on (Measure)
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure ADXL345");
        return ret;
    }

    // Configure ITG3205
    uint8_t power_mgmt = 0x00;
    ret = m_i2c_wrapper->write_register(ITG3205_ADDR, 0x3E, &power_mgmt, 1);  // Power management
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure ITG3205");
        return ret;
    }

    // Configure HMC5883L
    uint8_t config_a = 0x70;
    uint8_t config_b = 0xA0;
    uint8_t mode = 0x00;

    ret = m_i2c_wrapper->write_register(HMC5883L_ADDR, 0x00, &config_a, 1);  // Config A
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure HMC5883L Config A");
        return ret;
    }

    ret = m_i2c_wrapper->write_register(HMC5883L_ADDR, 0x01, &config_b, 1);  // Config B
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure HMC5883L Config B");
        return ret;
    }

    ret = m_i2c_wrapper->write_register(HMC5883L_ADDR, 0x02, &mode, 1);  // Continuous mode
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure HMC5883L Mode");
        return ret;
    }

    m_initialized = true;
    ESP_LOGI(TAG, "GY-85 inicializado com sucesso!");
    return ESP_OK;
}

// --- Leitura de sensores ---
esp_err_t GY85_I2C::read_accel(int16_t & x, int16_t & y, int16_t & z)
{
    uint8_t buf[6];
    esp_err_t ret = m_i2c_wrapper->read_register(ADXL345_ADDR, 0x32, buf, 6);
    if (ret == ESP_OK)
    {
        x = (int16_t)((buf[1] << 8) | buf[0]);
        y = (int16_t)((buf[3] << 8) | buf[2]);
        z = (int16_t)((buf[5] << 8) | buf[4]);
    }
    return ret;
}

esp_err_t GY85_I2C::read_gyro(int16_t & x, int16_t & y, int16_t & z)
{
    uint8_t buf[6];
    esp_err_t ret = m_i2c_wrapper->read_register(ITG3205_ADDR, 0x1D, buf, 6);
    if (ret == ESP_OK)
    {
        x = (int16_t)((buf[0] << 8) | buf[1]);
        y = (int16_t)((buf[2] << 8) | buf[3]);
        z = (int16_t)((buf[4] << 8) | buf[5]);
    }
    return ret;
}

esp_err_t GY85_I2C::read_mag(int16_t & x, int16_t & y, int16_t & z)
{
    uint8_t buf[6];
    esp_err_t ret = m_i2c_wrapper->read_register(HMC5883L_ADDR, 0x03, buf, 6);
    if (ret == ESP_OK)
    {
        x = (int16_t)((buf[0] << 8) | buf[1]);
        z = (int16_t)((buf[2] << 8) | buf[3]);
        y = (int16_t)((buf[4] << 8) | buf[5]);
    }
    return ret;
}

void GY85_I2C::deinit()
{
    if (!m_initialized)
        return;

    m_initialized = false;
    ESP_LOGI(TAG, "GY-85 desinicializado.");
}
