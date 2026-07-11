#ifndef IMUGY85_H
#define IMUGY85_H

#include <driver/i2c_master.h>
#include <esp_log.h>
#include <stdint.h>

#include "ADXL345.h"
#include "ITG3200.h"
#include "QMC5883L.h"

// Simple 3D vector structure for calibration data
struct Vector3f
{
    float x;
    float y;
    float z;
};

class IMUGY85
{
   public:
    static IMUGY85 & get_instance();

    esp_err_t init();

    void read_acceleration(int16_t * x, int16_t * y, int16_t * z);

    void read_gyro(int16_t * x, int16_t * y, int16_t * z);

    void read_magnetometer(int16_t * x, int16_t * y, int16_t * z);

    Vector3f read_acceleration_calibrated();

    Vector3f read_gyro_calibrated();

    Vector3f read_magnetometer_calibrated();

    void set_accel_calibration(const Vector3f & scale, const Vector3f & offset);

    void set_gyro_calibration(const Vector3f & scale, const Vector3f & offset);

    void set_mag_calibration(const Vector3f & scale, const Vector3f & offset);

    float get_accel_resolution() const;

    float get_gyro_resolution() const;

    float get_mag_resolution() const;

    ADXL345 & get_accel();
    ITG3200 & get_gyro();
    QMC5883L & get_mag();

   private:
    IMUGY85();

    i2c_master_bus_handle_t m_i2c_bus;
    ADXL345 m_accel;
    ITG3200 m_gyro;
    QMC5883L m_mag;

    Vector3f m_accel_scale = {1.0f, 1.0f, 1.0f};
    Vector3f m_accel_offset = {0.0f, 0.0f, 0.0f};
    Vector3f m_gyro_scale = {1.0f, 1.0f, 1.0f};
    Vector3f m_gyro_offset = {0.0f, 0.0f, 0.0f};
    Vector3f m_mag_scale = {1.0f, 1.0f, 1.0f};
    Vector3f m_mag_offset = {0.0f, 0.0f, 0.0f};

    float m_a_res = 0.0f;
    float m_g_res = 0.0f;
    float m_m_res = 0.0f;

    void update_a_res();
    void update_g_res();
    void update_m_res();
};

#endif  // _IMUGY85_H_
