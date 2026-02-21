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

// Arduino compatibility macros
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define degrees(x) ((x) * 180.0f / M_PI)
#define radians(x) ((x) * M_PI / 180.0f)

#define IMU_SAMPLE_RATE 100

/**
 * @brief Class for GY-85 9DOF IMU.
 *
 * This class interfaces with the ADXL345 accelerometer, ITG3200 gyroscope, and QMC5883L magnetometer
 * on the GY-85 board. It uses the Fusion AHRS filter to fuse sensor data and calculate
 * orientation (Pitch, Roll, Yaw).
 */
class IMUGY85
{
    /**
     * @brief Gyroscope Full Scale Range options.
     * @note The ITG-3200 supports only +/- 2000 dps.
     */
    enum Gscale
    {
        GFS_2000DPS = ITG3200_FULLSCALE_2000
    };

    /**
     * @brief Accelerometer Full Scale Range options.
     */
    enum Ascale
    {
        AFS_2G = 0,
        AFS_4G,
        AFS_8G,
        AFS_16G
    };

    uint8_t m_g_scale = GFS_2000DPS;
    uint8_t m_a_scale = AFS_16G;
    bool m_a_full_res = true;

    float m_a_res, m_g_res, m_m_res;

    int16_t m_accel_count[3];
    int16_t m_gyro_count[3];
    int16_t m_mag_count[3];

    bool m_use_magnetometer_fusion = false;
    FusionOffset m_offset;
    FusionAhrs m_ahrs;
    FusionAhrsSettings m_settings;

    // Fusion Calibration Data
    FusionMatrix m_gyro_misalignment;
    FusionVector m_gyro_sensitivity;
    FusionVector m_gyro_offset;
    FusionMatrix m_accel_misalignment;
    FusionVector m_accel_sensitivity;
    FusionVector m_accel_offset;
    FusionMatrix m_soft_iron_matrix;
    FusionVector m_hard_iron_offset;

    double m_dt = 0.0f;         ///< Integration interval for filter (seconds)
    int64_t m_last_update = 0;  ///< Last update time in microseconds
    int64_t m_now = 0;          ///< Current time in microseconds

    float m_ax, m_ay, m_az;  ///< Latest accelerometer values (g)
    float m_gx, m_gy, m_gz;  ///< Latest gyroscope values (deg/s)
    float m_mx, m_my, m_mz;  ///< Latest magnetometer values (mG)

    double m_pitch = 0;  ///< Calculated Pitch angle (degrees)
    double m_roll = 0;   ///< Calculated Roll angle (degrees)
    double m_yaw = 0;    ///< Calculated Yaw angle (degrees)

   public:
    ADXL345 accel;
    ITG3200 gyro;
    QMC5883L mag;

    IMUGY85();

    /**
     * @brief Initialize all internal sensors (Accel, Gyro, Mag) and Fusion algorithm.
     * @note I2C bus must be initialized before calling this.
     */
    void init();

    /**
     * @brief Update the IMU state.
     *
     * Reads new data from all sensors, applies calibrations, updates the Fusion AHRS,
     * and computes Euler angles.
     */
    void update();

    // Getters
    double get_roll() const;
    double get_pitch() const;
    double get_yaw() const;

    void get_acceleration(double * a1, double * a2, double * a3) const;
    void get_gyro(double * m1, double * m2, double * m3) const;
    void get_magnetometer(double * m1, double * m2, double * m3) const;

    // Calibration setters for external calibration
    void set_gyroscope_calibration(FusionMatrix misalignment, FusionVector sensitivity, FusionVector offset_vec);
    void set_accelerometer_calibration(FusionMatrix misalignment, FusionVector sensitivity, FusionVector offset_vec);
    void set_magnetometer_calibration(FusionMatrix soft_iron, FusionVector hard_iron);

    /**
     * @brief Loads magnetometer calibration from NVS, or starts calibration if missing.
     * @param seconds Duration in seconds for the calibration routine.
     * @return ESP_OK on success, ESP_FAIL on failure.
     */
    esp_err_t load_or_calibrate_mag(uint32_t seconds = 10);

    /**
     * @brief Force a new magnetometer calibration sequence.
     * @param seconds Duration in seconds.
     * @return ESP_OK on success.
     */
    esp_err_t calibrate_magnetometer(uint32_t seconds = 10);

   private:
    void update_a_res();
    void update_g_res();
    void update_m_res();
};

#endif  // _IMUGY85_H_
