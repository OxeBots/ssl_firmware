/**
 * @file IMUGY85.cpp
 * @brief Driver implementation for the GY-85 9-DOF IMU sensor board using Fusion AHRS.
 */

#include "IMUGY85.h"

static const char * TAG = "IMUGY85";

/**
 * @brief Default Constructor.
 */
IMUGY85::IMUGY85()
{
}

/**
 * @brief Initialize all sensors and Fusion Algorithm.
 * Configures default ranges, loads existing calibrations from NVS safely,
 * and initializes the Fusion AHRS matrices to identity.
 */
void IMUGY85::init()
{
    accel.init();
    gyro.init();
    mag.init();
    mag.set_smoothing(10, true);

    // Setup ranges
    accel.set_range(static_cast<ADXL345::Range>(m_a_scale));
    accel.set_full_resolution(true);
    m_a_scale = static_cast<uint8_t>(accel.get_range());

    // Identity matrices by default
    m_accel_misalignment = {1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f};
    m_gyro_misalignment = {1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f};
    m_soft_iron_matrix = {1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f};

    m_hard_iron_offset = {0.f, 0.f, 0.f};
    m_gyro_offset = {0.f, 0.f, 0.f};  // internal driver handles zero-rate offset
    m_gyro_sensitivity = {1.f, 1.f, 1.f};

    // Attempt to load calibrations safely from NVS.
    // They fall back to zeroes/identities inside the sensors if not found.
    if (accel.load_calibration_from_nvs() == ESP_OK)
    {
        m_accel_sensitivity = {accel.get_calibration_scale(0), accel.get_calibration_scale(1),
                               accel.get_calibration_scale(2)};
        m_accel_offset = {accel.get_calibration_offset(0), accel.get_calibration_offset(1),
                          accel.get_calibration_offset(2)};
    }
    else
    {
        m_accel_sensitivity = {1.f, 1.f, 1.f};
        m_accel_offset = {0.f, 0.f, 0.f};
    }

    if (gyro.load_calibration_from_nvs() != ESP_OK)
    {
        ESP_LOGW(TAG, "No Gyro calibration found in NVS. Defaulting to 0 offsets.");
    }

    if (mag.load_calibration_from_nvs() != ESP_OK)
    {
        ESP_LOGW(TAG, "No Mag calibration found in NVS. Use load_or_calibrate_mag().");
    }

    FusionOffsetInitialise(&m_offset, IMU_SAMPLE_RATE);
    FusionAhrsInitialise(&m_ahrs);

    m_settings.convention = FusionConventionEnu;
    m_settings.gain = 0.5f;
    m_settings.gyroscopeRange = 2000.0f;
    m_settings.accelerationRejection = 10.0f;
    m_settings.magneticRejection = 10.0f;
    m_settings.recoveryTriggerPeriod = 5 * IMU_SAMPLE_RATE;

    FusionAhrsSetSettings(&m_ahrs, &m_settings);

    m_last_update = esp_timer_get_time();
}

/**
 * @brief Performs interactive Accelerometer Calibration and saves to NVS.
 * @param seconds Duration in seconds (unused parameter kept for uniform API signatures).
 * @return ESP_OK on success.
 */
esp_err_t IMUGY85::calibrate_accelerometer(uint32_t seconds)
{
    ESP_LOGI(TAG, "Starting Accelerometer Calibration. Place robot completely FLAT and STATIONARY!");
    for (int i = 3; i > 0; i--)
    {
        ESP_LOGI(TAG, "%d...", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // Note: The accel calibration loops automatically until full ranges are found.
    // The user must rotate it to all faces slowly.
    ESP_LOGI(TAG, "Slowly rotate the robot 360 degrees on all 3 axes.");
    accel.calibrate();

    m_accel_sensitivity = {accel.get_calibration_scale(0), accel.get_calibration_scale(1),
                           accel.get_calibration_scale(2)};
    m_accel_offset = {accel.get_calibration_offset(0), accel.get_calibration_offset(1),
                      accel.get_calibration_offset(2)};

    return accel.save_calibration_to_nvs();
}

/**
 * @brief Performs interactive Gyroscope Calibration and saves to NVS.
 * @param seconds Duration in seconds (unused parameter kept for uniform API signatures).
 * @return ESP_OK on success.
 */
esp_err_t IMUGY85::calibrate_gyroscope(uint32_t seconds)
{
    ESP_LOGI(TAG, "Starting Gyroscope Calibration. Keep robot COMPLETELY STILL!");
    for (int i = 3; i > 0; i--)
    {
        ESP_LOGI(TAG, "%d...", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    gyro.calibrate(1000);

    return gyro.save_calibration_to_nvs();
}

/**
 * @brief Performs interactive Magnetometer Calibration and saves to NVS.
 * @param seconds Duration in seconds.
 * @return ESP_OK on success.
 */
esp_err_t IMUGY85::calibrate_magnetometer(uint32_t seconds)
{
    ESP_LOGI(TAG, "Starting Magnetometer Calibration. Rotate the sensor in all directions!");
    for (int i = 3; i > 0; i--)
    {
        ESP_LOGI(TAG, "Starting in %d...", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "GO!");

    mag.start_calibration_mode(seconds);
    bool finished = false;

    while (!finished)
    {
        finished = mag.calibration_update();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    mag.stop_calibration_mode();
    return ESP_OK;
}

/**
 * @brief Loads Accelerometer calibration from NVS, or forces a new one if missing.
 * @param seconds Duration in seconds.
 * @return ESP_OK on success.
 */
esp_err_t IMUGY85::load_or_calibrate_accel(uint32_t seconds)
{
    if (accel.is_calibrated())
        return ESP_OK;
    ESP_LOGW(TAG, "No accelerometer calibration found. Starting interactive calibration...");
    return calibrate_accelerometer(seconds);
}

/**
 * @brief Loads Gyroscope calibration from NVS, or forces a new one if missing.
 * @param seconds Duration in seconds.
 * @return ESP_OK on success.
 */
esp_err_t IMUGY85::load_or_calibrate_gyro(uint32_t seconds)
{
    if (gyro.is_calibrated())
        return ESP_OK;
    ESP_LOGW(TAG, "No gyroscope calibration found. Starting interactive calibration...");
    return calibrate_gyroscope(seconds);
}

/**
 * @brief Loads Magnetometer calibration from NVS, or starts interactive mode if missing.
 * @param seconds Duration in seconds.
 * @return ESP_OK on success.
 */
esp_err_t IMUGY85::load_or_calibrate_mag(uint32_t seconds)
{
    if (mag.is_calibrated())
        return ESP_OK;
    ESP_LOGW(TAG, "No magnetometer calibration found. Starting interactive calibration...");
    return calibrate_magnetometer(seconds);
}

/**
 * @brief Main periodic update function. Reads raw data, applies resolutions, and updates the Fusion AHRS algorithm.
 */
void IMUGY85::update()
{
    accel.get_acceleration(&m_accel_count[0], &m_accel_count[1], &m_accel_count[2]);
    gyro.get_rotation(&m_gyro_count[0], &m_gyro_count[1], &m_gyro_count[2]);
    mag.get_orientation(&m_mag_count[0], &m_mag_count[1], &m_mag_count[2]);

    update_a_res();
    update_g_res();
    update_m_res();

    FusionVector gyroscopeUncal = {(float)m_gyro_count[0] * m_g_res, (float)m_gyro_count[1] * m_g_res,
                                   (float)m_gyro_count[2] * m_g_res};

    FusionVector accelerometerUncal = {(float)m_accel_count[1] * m_a_res, -(float)m_accel_count[0] * m_a_res,
                                       (float)m_accel_count[2] * m_a_res};

    FusionVector magnetometerUncal = {(float)m_mag_count[0] * m_m_res, (float)m_mag_count[1] * m_m_res,
                                      (float)m_mag_count[2] * m_m_res};

    FusionVector gyroscope =
      FusionCalibrationInertial(gyroscopeUncal, m_gyro_misalignment, m_gyro_sensitivity, m_gyro_offset);
    FusionVector accelerometer =
      FusionCalibrationInertial(accelerometerUncal, m_accel_misalignment, m_accel_sensitivity, m_accel_offset);
    FusionVector magnetometer = FusionCalibrationMagnetic(magnetometerUncal, m_soft_iron_matrix, m_hard_iron_offset);

    magnetometer.axis.x = magnetometer.axis.y;
    magnetometer.axis.y = magnetometer.axis.x;
    magnetometer.axis.z = magnetometer.axis.z;

    gyroscope = FusionOffsetUpdate(&m_offset, gyroscope);

    m_gx = gyroscope.axis.x;
    m_gy = gyroscope.axis.y;
    m_gz = gyroscope.axis.z;
    m_ax = accelerometer.axis.x;
    m_ay = accelerometer.axis.y;
    m_az = accelerometer.axis.z;
    m_mx = magnetometer.axis.x;
    m_my = magnetometer.axis.y;
    m_mz = magnetometer.axis.z;

    m_now = esp_timer_get_time();
    m_dt = ((m_now - m_last_update) / 1000000.0f);
    m_last_update = m_now;

    if (m_dt > 1.0f)
        m_dt = 0.01f;

    FusionAhrsUpdateNoMagnetometer(&m_ahrs, gyroscope, accelerometer, m_dt);
    FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&m_ahrs));

    m_roll = euler.angle.roll;
    m_pitch = euler.angle.pitch;
    m_yaw = euler.angle.yaw;
}

/**
 * @brief Get the calculated Roll angle.
 * @return Roll angle in degrees.
 */
double IMUGY85::get_roll() const
{
    return m_roll;
}

/**
 * @brief Get the calculated Pitch angle.
 * @return Pitch angle in degrees.
 */
double IMUGY85::get_pitch() const
{
    return m_pitch;
}

/**
 * @brief Get the calculated Yaw angle.
 * @return Yaw angle in degrees.
 */
double IMUGY85::get_yaw() const
{
    return m_yaw;
}

/**
 * @brief Get current acceleration values (in g's).
 * @param a1 Pointer to store X acceleration.
 * @param a2 Pointer to store Y acceleration.
 * @param a3 Pointer to store Z acceleration.
 */
void IMUGY85::get_acceleration(double * a1, double * a2, double * a3) const
{
    *a1 = m_ax;
    *a2 = m_ay;
    *a3 = m_az;
}

/**
 * @brief Get current gyroscope values (in deg/s).
 * @param m1 Pointer to store X gyro rate.
 * @param m2 Pointer to store Y gyro rate.
 * @param m3 Pointer to store Z gyro rate.
 */
void IMUGY85::get_gyro(double * m1, double * m2, double * m3) const
{
    *m1 = m_gx;
    *m2 = m_gy;
    *m3 = m_gz;
}

/**
 * @brief Get current magnetometer values (in mG).
 * @param m1 Pointer to store X mag.
 * @param m2 Pointer to store Y mag.
 * @param m3 Pointer to store Z mag.
 */
void IMUGY85::get_magnetometer(double * m1, double * m2, double * m3) const
{
    *m1 = m_mx;
    *m2 = m_my;
    *m3 = m_mz;
}

/**
 * @brief Updates the Fusion AHRS gyroscope calibration matrices.
 * @param misalignment Gyroscope misalignment matrix.
 * @param sensitivity Gyroscope sensitivity vector.
 * @param offset_vec Gyroscope offset vector.
 */
void IMUGY85::set_gyroscope_calibration(FusionMatrix misalignment, FusionVector sensitivity, FusionVector offset_vec)
{
    m_gyro_misalignment = misalignment;
    m_gyro_sensitivity = sensitivity;
    m_gyro_offset = offset_vec;
}

/**
 * @brief Updates the Fusion AHRS accelerometer calibration matrices.
 * @param misalignment Accelerometer misalignment matrix.
 * @param sensitivity Accelerometer sensitivity vector.
 * @param offset_vec Accelerometer offset vector.
 */
void IMUGY85::set_accelerometer_calibration(FusionMatrix misalignment, FusionVector sensitivity,
                                            FusionVector offset_vec)
{
    m_accel_misalignment = misalignment;
    m_accel_sensitivity = sensitivity;
    m_accel_offset = offset_vec;
}

/**
 * @brief Updates the Fusion AHRS magnetometer calibration matrices.
 * @param softIron Soft iron distortion matrix.
 * @param hardIron Hard iron offset vector.
 */
void IMUGY85::set_magnetometer_calibration(FusionMatrix softIron, FusionVector hardIron)
{
    m_soft_iron_matrix = softIron;
    m_hard_iron_offset = hardIron;
}

/**
 * @brief Enable or disable magnetometer usage inside the Fusion AHRS filter.
 * @param use_mag If false, falls back to 6-DOF tracking.
 */
void IMUGY85::set_fusion_mode(bool use_mag)
{
    m_use_magnetometer_fusion = use_mag;
}

/**
 * @brief Calculates magnetometer resolution based on current hardware settings.
 */
void IMUGY85::update_m_res()
{
    m_m_res = (mag.get_range() == QMC5883L::Range::RNG_2G) ? (1000.0f / 12000.0f) : (1000.0f / 3000.0f);
}

/**
 * @brief Calculates gyroscope resolution based on current hardware settings.
 */
void IMUGY85::update_g_res()
{
    m_g_res = 1.0f / 14.375f;
}

/**
 * @brief Calculates accelerometer resolution based on current hardware settings.
 */
void IMUGY85::update_a_res()
{
    if (m_a_full_res)
        m_a_res = 1.0f / 256.0f;
    else
    {
        switch (static_cast<ADXL345::Range>(m_a_scale))
        {
            case ADXL345::Range::RNG_2G:
                m_a_res = 4.0f / 1024.0f;
                break;
            case ADXL345::Range::RNG_4G:
                m_a_res = 8.0f / 1024.0f;
                break;
            case ADXL345::Range::RNG_8G:
                m_a_res = 16.0f / 1024.0f;
                break;
            case ADXL345::Range::RNG_16G:
                m_a_res = 32.0f / 1024.0f;
                break;
            default:
                m_a_res = 1.0f / 256.0f;
                break;
        }
    }
}
