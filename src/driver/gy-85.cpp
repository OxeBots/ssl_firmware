#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

class GY85_I2C {
public:
    GY85_I2C() : m_initialized(false), m_bus_handle(nullptr),
                 adxl_handle(nullptr), itg_handle(nullptr), hmc_handle(nullptr) {}
    ~GY85_I2C() { deinit(); }

    // Inicializa os três dispositivos GY-85 no barramento já criado
    esp_err_t init(i2c_master_bus_handle_t bus_handle);

    // Leitura dos sensores
    esp_err_t read_accel(int16_t &xA, int16_t &yA, int16_t &zA);
    esp_err_t read_gyro(int16_t &xG, int16_t &yG, int16_t &zG);
    esp_err_t read_mag(int16_t &xM, int16_t &yM, int16_t &zM);

    // Libera os devices do barramento
    void deinit();

private:
    static constexpr uint8_t ADXL345_ADDR = 0x53;
    static constexpr uint8_t ITG3205_ADDR = 0x68;
    static constexpr uint8_t HMC5883L_ADDR = 0x1E;
    static constexpr int I2C_TIMEOUT_MS = 100;
    static constexpr const char *TAG = "GY85_I2C";

    bool m_initialized;
    i2c_master_bus_handle_t m_bus_handle;
    i2c_master_dev_handle_t adxl_handle;
    i2c_master_dev_handle_t itg_handle;
    i2c_master_dev_handle_t hmc_handle;

    // Funções utilitárias
    esp_err_t i2c_write_byte(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t data);
    esp_err_t i2c_read_bytes(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data, size_t len);
};

// --- Implementações auxiliares ---
esp_err_t GY85_I2C::i2c_write_byte(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(dev, buf, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

esp_err_t GY85_I2C::i2c_read_bytes(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

// --- Inicialização dos dispositivos ---
esp_err_t GY85_I2C::init(i2c_master_bus_handle_t bus_handle) {
    if (m_initialized) return ESP_OK;
    m_bus_handle = bus_handle;

    esp_err_t ret;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz = 100000,
        .scl_wait_us = 0
    };

    // --- ADXL345 ---
    dev_cfg.device_address = ADXL345_ADDR;
    ret = i2c_master_bus_add_device(m_bus_handle, &dev_cfg, &adxl_handle);
    if (ret != ESP_OK) goto fail;
    i2c_write_byte(adxl_handle, 0x2D, 0x08); // Power on (Measure)

    // --- ITG3205 ---
    dev_cfg.device_address = ITG3205_ADDR;
    ret = i2c_master_bus_add_device(m_bus_handle, &dev_cfg, &itg_handle);
    if (ret != ESP_OK) goto fail;
    i2c_write_byte(itg_handle, 0x3E, 0x00); // Power management

    // --- HMC5883L ---
    dev_cfg.device_address = HMC5883L_ADDR;
    ret = i2c_master_bus_add_device(m_bus_handle, &dev_cfg, &hmc_handle);
    if (ret != ESP_OK) goto fail;
    i2c_write_byte(hmc_handle, 0x00, 0x70); // Config A
    i2c_write_byte(hmc_handle, 0x01, 0xA0); // Config B
    i2c_write_byte(hmc_handle, 0x02, 0x00); // Continuous mode

    m_initialized = true;
    ESP_LOGI(TAG, "GY-85 inicializado com sucesso!");
    return ESP_OK;

fail:
    ESP_LOGE(TAG, "Falha ao inicializar GY-85");
    deinit();
    return ret;
}

// --- Leitura de sensores ---
esp_err_t GY85_I2C::read_accel(int16_t &x, int16_t &y, int16_t &z) {
    uint8_t buf[6];
    esp_err_t ret = i2c_read_bytes(adxl_handle, 0x32, buf, 6);
    if (ret == ESP_OK) {
        x = (int16_t)((buf[1] << 8) | buf[0]);
        y = (int16_t)((buf[3] << 8) | buf[2]);
        z = (int16_t)((buf[5] << 8) | buf[4]);
    }
    return ret;
}

esp_err_t GY85_I2C::read_gyro(int16_t &x, int16_t &y, int16_t &z) {
    uint8_t buf[6];
    esp_err_t ret = i2c_read_bytes(itg_handle, 0x1D, buf, 6);
    if (ret == ESP_OK) {
        x = (int16_t)((buf[0] << 8) | buf[1]);
        y = (int16_t)((buf[2] << 8) | buf[3]);
        z = (int16_t)((buf[4] << 8) | buf[5]);
    }
    return ret;
}

esp_err_t GY85_I2C::read_mag(int16_t &x, int16_t &y, int16_t &z) {
    uint8_t buf[6];
    esp_err_t ret = i2c_read_bytes(hmc_handle, 0x03, buf, 6);
    if (ret == ESP_OK) {
        x = (int16_t)((buf[0] << 8) | buf[1]);
        z = (int16_t)((buf[2] << 8) | buf[3]);
        y = (int16_t)((buf[4] << 8) | buf[5]);
    }
    return ret;
}


void GY85_I2C::deinit() {
    if (!m_initialized) return;
    if (adxl_handle) i2c_master_bus_rm_device(adxl_handle);
    if (itg_handle) i2c_master_bus_rm_device(itg_handle);
    if (hmc_handle) i2c_master_bus_rm_device(hmc_handle);
    m_initialized = false;
    ESP_LOGI(TAG, "GY-85 desinicializado.");
}
