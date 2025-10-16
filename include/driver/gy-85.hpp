#ifndef DRIVER_GY85_H
#define DRIVER_GY85_H

#include <cstring>

#include "driver/i2c_wrapper.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class GY85_I2C
{
   public:
    GY85_I2C() : m_initialized(false) {}
    ~GY85_I2C() { deinit(); }

    /**
     * @brief Initializes the GY-85 IMU using an existing I2C_Wrapper
     * @param i2c_wrapper Reference to a pre-initialized I2C_Wrapper
     * @return ESP_OK on success
     */
    esp_err_t init(I2C_Wrapper & i2c_wrapper);

    // Leitura dos sensores
    esp_err_t read_accel(int16_t & xA, int16_t & yA, int16_t & zA);
    esp_err_t read_gyro(int16_t & xG, int16_t & yG, int16_t & zG);
    esp_err_t read_mag(int16_t & xM, int16_t & yM, int16_t & zM);

    // Libera os devices
    void deinit();

   private:
    static constexpr uint8_t ADXL345_ADDR = 0x53;
    static constexpr uint8_t ITG3205_ADDR = 0x68;
    static constexpr uint8_t HMC5883L_ADDR = 0x1E;
    static constexpr const char * TAG = "GY85_I2C";

    bool m_initialized;
    I2C_Wrapper * m_i2c_wrapper;  // Pointer to shared I2C wrapper
};

#endif
