/**
 * @file IMUGY85.h
 * @brief Driver for the GY-85 9-DOF IMU sensor board using Fusion AHRS.
 * @details This library fuses data from the ADXL345 accelerometer, ITG3200 gyroscope, and QMC5883L
 * magnetometer using the Fusion AHRS library to provide stable orientation estimates (Roll, Pitch,
 * Yaw).
 */

#ifndef _IMUGY85_H_
#define _IMUGY85_H_

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

// Estimated sample rate for initialization (Hz)
#define IMU_SAMPLE_RATE 100

/**
 * @brief Class for GY-85 9DOF IMU.
 *
 * This class interfaces with the ADXL345 accelerometer, ITG3200 gyroscope, and QMC5883L magnetometer
 * on the GY-85 board. It implements the Fusion AHRS filter to fuse sensor data and calculate
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
        GFS_2000DPS = ITG3200_FULLSCALE_2000  ///< +/- 2000 degrees per second
    };

    /**
     * @brief Accelerometer Full Scale Range options.
     */
    enum Ascale
    {
        AFS_2G = 0,  ///< +/- 2G
        AFS_4G,      ///< +/- 4G
        AFS_8G,      ///< +/- 8G
        AFS_16G      ///< +/- 16G
    };

    /**
     * @brief Magnetometer Scale/Resolution options.
     */
    enum mScale
    {
        M_RNG_2G = QMC5883L_RNG_2G,  // +/- 2 Gauss
        M_RNG_8G = QMC5883L_RNG_8G   // +/- 8 Gauss
    };

    uint8_t Gscale = GFS_2000DPS;  ///< Current Gyro scale setting
    uint8_t Ascale = AFS_16G;      ///< Current Accel scale setting
    uint8_t Mscale = M_RNG_8G;   ///< Current Mag scale setting (16-bit default)
    bool AfullRes = true;          ///< Full Resolution mode for Accel
    float aRes, gRes, mRes;        ///< Calculated resolutions per LSB for the sensors
    uint8_t currentMagMode = QMC5883L_MODE_CONTINUOUS;
    uint8_t currentMagODR  = QMC5883L_ODR_200HZ;
    uint8_t currentMagRNG  = QMC5883L_RNG_8G;
    uint8_t currentMagOSR  = QMC5883L_OSR_512;


    int16_t accelCount[3];  ///< Raw accelerometer sensor output (X, Y, Z)
    int16_t gyroCount[3];   ///< Raw gyroscope sensor output (X, Y, Z)
    int16_t magCount[3];    ///< Raw magnetometer sensor output (X, Y, Z)

    // Fusion Objects
    bool useMagnetometerFusion = false;  // Default to FALSE (6-DOF) like MPU6050
    FusionOffset offset;
    FusionAhrs ahrs;
    FusionAhrsSettings settings;

    // Fusion Calibration Data
    FusionMatrix gyroscopeMisalignment;
    FusionVector gyroscopeSensitivity;
    FusionVector gyroscopeOffset;
    FusionMatrix accelerometerMisalignment;
    FusionVector accelerometerSensitivity;
    FusionVector accelerometerOffset;
    FusionMatrix softIronMatrix;
    FusionVector hardIronOffset;

    double dt = 0.0f;        ///< Integration interval for filter (seconds)
    int64_t lastUpdate = 0;  ///< Last update time in microseconds
    int64_t Now = 0;         ///< Current time in microseconds

    float ax, ay, az;  ///< Latest accelerometer values (g)
    float gx, gy, gz;  ///< Latest gyroscope values (deg/s)
    float mx, my, mz;  ///< Latest magnetometer values (mG)

    double pitch = 0;  ///< Calculated Pitch angle (degrees)
    double roll = 0;   ///< Calculated Roll angle (degrees)
    double yaw = 0;    ///< Calculated Yaw angle (degrees)

   public:
    ADXL345 accel;  ///< Accelerometer driver instance
    ITG3200 gyro;   ///< Gyroscope driver instance
    QMC5883L mag;   ///< Magnetometer driver instance

    /**
     * @brief Constructor for IMUGY85.
     */
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

    /**
     * @brief Enable or Disable magnetometer fusion.
     * Set to false to behave like an MPU6050 (6-DOF).
     * Set to true only if Magnetometer is fully calibrated.
     */
    void setFusionMode(bool useMag);

    /**
     * @brief Get the calculated Roll angle.
     * @return Roll angle in degrees.
     */
    double getRoll();

    /**
     * @brief Get the calculated Pitch angle.
     * @return Pitch angle in degrees.
     */
    double getPitch();

    /**
     * @brief Get the calculated Yaw angle.
     * @return Yaw angle in degrees (0-360).
     */
    double getYaw();

    /**
     * @brief Get current acceleration values (in g's).
     * @param a1 Pointer to store X acceleration.
     * @param a2 Pointer to store Y acceleration.
     * @param a3 Pointer to store Z acceleration.
     */
    void getAcceleration(double * a1, double * a2, double * a3);

    /**
     * @brief Get current gyroscope values (in deg/s).
     * @param m1 Pointer to store X gyro rate.
     * @param m2 Pointer to store Y gyro rate.
     * @param m3 Pointer to store Z gyro rate.
     */
    void getGyro(double * m1, double * m2, double * m3);

    /**
     * @brief Get current magnetometer values (in mG).
     * @param m1 Pointer to store X mag.
     * @param m2 Pointer to store Y mag.
     * @param m3 Pointer to store Z mag.
     */
    void getMagnetometer(double * m1, double * m2, double * m3);

    void setGyroscopeCalibration(FusionMatrix misalignment, FusionVector sensitivity, FusionVector offset_vec)
    {
        gyroscopeMisalignment = misalignment;
        gyroscopeSensitivity = sensitivity;
        gyroscopeOffset = offset_vec;
    }

    void setAccelerometerCalibration(FusionMatrix misalignment, FusionVector sensitivity, FusionVector offset_vec)
    {
        accelerometerMisalignment = misalignment;
        accelerometerSensitivity = sensitivity;
        accelerometerOffset = offset_vec;
    }

    void setMagnetometerCalibration(FusionMatrix softIron, FusionVector hardIron)
    {
        softIronMatrix = softIron;
        hardIronOffset = hardIron;
    }

   private:
    /**
     * @brief Calculate accelerometer resolution based on current Ascale.
     */
    void getAres();

    /**
     * @brief Calculate gyroscope resolution based on current Gscale.
     */
    void getGres();

    /**
     * @brief Calculate magnetometer resolution based on current Mscale.
     */
    void getMres();
};

#endif  // _IMUGY85_H_
