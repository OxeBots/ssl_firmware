/**
 * @file IMUGY85.h
 * @brief Driver for the GY-85 9-DOF IMU sensor board using Fusion AHRS.
 */
#ifndef _IMUGY85_H_
#define _IMUGY85_H_

#include <esp_log.h>
#include <esp_timer.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "ADXL345.h"
#include "Fusion.h"
#include "I2Cdev.h"
#include "ITG3200.h"
#include "QMC5883L.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define degrees(x) ((x) * 180.0f / M_PI)
#define radians(x) ((x) * M_PI / 180.0f)

#define IMU_SAMPLE_RATE 100

class IMUGY85
{
    uint8_t m_g_scale = static_cast<uint8_t>(ITG3200::FullScaleRange::FS_2000);
    uint8_t m_a_scale = static_cast<uint8_t>(ADXL345::Range::RNG_16G);
    bool m_a_full_res = true;

    float m_a_res = 0.0f, m_g_res = 0.0f, m_m_res = 0.0f;

    int16_t m_accel_count[3] = {0, 0, 0};
    int16_t m_gyro_count[3] = {0, 0, 0};
    int16_t m_mag_count[3] = {0, 0, 0};

    bool m_use_magnetometer_fusion = false;
    FusionOffset m_offset = {};
    FusionAhrs m_ahrs = {};
    FusionAhrsSettings m_settings = {};

    FusionMatrix m_gyro_misalignment = {};
    FusionVector m_gyro_sensitivity = {};
    FusionVector m_gyro_offset = {};
    FusionMatrix m_accel_misalignment = {};
    FusionVector m_accel_sensitivity = {};
    FusionVector m_accel_offset = {};
    FusionMatrix m_soft_iron_matrix = {};
    FusionVector m_hard_iron_offset = {};

    double m_dt = 0.0f;
    int64_t m_last_update = 0;
    int64_t m_now = 0;

    float m_ax = 0.0f, m_ay = 0.0f, m_az = 0.0f;
    float m_gx = 0.0f, m_gy = 0.0f, m_gz = 0.0f;
    float m_mx = 0.0f, m_my = 0.0f, m_mz = 0.0f;

    double m_pitch = 0.0;
    double m_roll = 0.0;
    double m_yaw = 0.0;

   public:
    ADXL345 accel;
    ITG3200 gyro;
    QMC5883L mag;

    IMUGY85();

    void init();
    void update();

    void set_fusion_mode(bool use_mag);

    double get_roll() const;
    double get_pitch() const;
    double get_yaw() const;

    void get_acceleration(double * a1, double * a2, double * a3) const;
    void get_gyro(double * m1, double * m2, double * m3) const;
    void get_magnetometer(double * m1, double * m2, double * m3) const;

    void set_gyroscope_calibration(FusionMatrix misalignment, FusionVector sensitivity, FusionVector offset_vec);
    void set_accelerometer_calibration(FusionMatrix misalignment, FusionVector sensitivity, FusionVector offset_vec);
    void set_magnetometer_calibration(FusionMatrix softIron, FusionVector hardIron);

    esp_err_t calibrate_accelerometer(uint32_t seconds = 10);
    esp_err_t calibrate_gyroscope(uint32_t seconds = 10);
    esp_err_t calibrate_magnetometer(uint32_t seconds = 10);

    esp_err_t load_or_calibrate_accel(uint32_t seconds = 10);
    esp_err_t load_or_calibrate_gyro(uint32_t seconds = 10);
    esp_err_t load_or_calibrate_mag(uint32_t seconds = 10);

   private:
    void update_a_res();
    void update_g_res();
    void update_m_res();
};

#endif  // _IMUGY85_H_
