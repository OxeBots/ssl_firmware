#include "OrientationHandler.h"

#include <esp_timer.h>

static const char * TAG = "OrientationHandler";

OrientationHandler & OrientationHandler::get_instance()
{
    static OrientationHandler instance;
    return instance;
}

OrientationHandler::OrientationHandler() : m_driver(IMUGY85::get_instance()), m_mutex(nullptr)
{
}

/**
 * @brief Initialize the IMU hardware driver and load calibration from NVS.
 * @return ESP_OK on success.
 */
esp_err_t OrientationHandler::init()
{
    // Create mutex
    m_mutex = xSemaphoreCreateMutex();
    configASSERT(m_mutex != nullptr);

    // Initialize hardware driver
    esp_err_t err = m_driver.init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize IMU hardware driver.");
        return err;
    }

    // Initialize AHRS
    FusionOffsetInitialise(&m_offset, TASK_RATE_HZ);
    FusionAhrsInitialise(&m_ahrs);

    m_settings.convention = FusionConventionEnu;
    m_settings.gain = 0.5f;
    m_settings.gyroscopeRange = 2000.0f;
    m_settings.accelerationRejection = 10.0f;
    m_settings.magneticRejection = 10.0f;
    m_settings.recoveryTriggerPeriod = 5 * TASK_RATE_HZ;

    FusionAhrsSetSettings(&m_ahrs, &m_settings);

    ESP_LOGI(TAG, "OrientationHandler initialized.");
    return ESP_OK;
}

/**
 * @brief Load all calibrations (accel, gyro, mag) from NVS.
 * Does NOT run calibration routines if data is missing.
 * @return ESP_OK if all calibrations loaded, ESP_ERR_NOT_FOUND if any missing.
 */
esp_err_t OrientationHandler::load_calibration()
{
    esp_err_t result = ESP_OK;

    // Load accelerometer calibration
    esp_err_t err = load_accel_calibration_from_nvs();
    if (err == ESP_OK)
    {
        m_accel_calibrated = true;
        ESP_LOGI(TAG, "Accelerometer calibration loaded.");
    }
    else
    {
        m_accel_calibrated = false;
        ESP_LOGW(TAG, "Accelerometer calibration not found.");
        result = ESP_ERR_NOT_FOUND;
    }

    // Load gyroscope calibration
    err = load_gyro_calibration_from_nvs();
    if (err == ESP_OK)
    {
        m_gyro_calibrated = true;
        ESP_LOGI(TAG, "Gyroscope calibration loaded.");
    }
    else
    {
        m_gyro_calibrated = false;
        ESP_LOGW(TAG, "Gyroscope calibration not found.");
        result = ESP_ERR_NOT_FOUND;
    }

    // Load magnetometer calibration
    err = load_mag_calibration_from_nvs();
    if (err == ESP_OK)
    {
        m_mag_calibrated = true;
        ESP_LOGI(TAG, "Magnetometer calibration loaded.");
    }
    else
    {
        m_mag_calibrated = false;
        ESP_LOGW(TAG, "Magnetometer calibration not found.");
        result = ESP_ERR_NOT_FOUND;
    }

    return result;
}

/**
 * @brief Run interactive magnetometer calibration and save to NVS.
 * @param timeout_s Maximum seconds to wait for interactive calibration.
 * @return ESP_OK on success.
 */
esp_err_t OrientationHandler::calibrate_mag(uint32_t timeout_s)
{
    ESP_LOGI(TAG, "Starting Magnetometer Calibration. Rotate the sensor in all directions!");
    for (int i = 3; i > 0; i--)
    {
        ESP_LOGI(TAG, "Starting in %d...", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "GO!");

    QMC5883L & mag = m_driver.get_mag();
    mag.start_calibration_mode(timeout_s);
    bool finished = false;

    while (!finished)
    {
        finished = mag.calibration_update();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    mag.stop_calibration_mode();

    // Save to NVS
    esp_err_t err = save_mag_calibration_to_nvs();
    if (err == ESP_OK)
    {
        m_mag_calibrated = true;
        ESP_LOGI(TAG, "Magnetometer calibration saved to NVS.");
    }
    return err;
}

/**
 * @brief Run interactive accelerometer calibration and save to NVS.
 * @param timeout_s Maximum seconds to wait for interactive calibration.
 * @return ESP_OK on success.
 */
esp_err_t OrientationHandler::calibrate_accel(uint32_t timeout_s)
{
    ESP_LOGI(TAG,
             "Starting Accelerometer Calibration. Place robot completely FLAT and STATIONARY!");
    for (int i = 3; i > 0; i--)
    {
        ESP_LOGI(TAG, "%d...", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    ADXL345 & accel = m_driver.get_accel();
    accel.calibrate();

    // Apply calibration to driver
    Vector3f scale = {accel.get_calibration_scale(0),
                      accel.get_calibration_scale(1),
                      accel.get_calibration_scale(2)};
    Vector3f offset = {accel.get_calibration_offset(0),
                       accel.get_calibration_offset(1),
                       accel.get_calibration_offset(2)};
    m_driver.set_accel_calibration(scale, offset);

    // Save to NVS
    esp_err_t err = save_accel_calibration_to_nvs();
    if (err == ESP_OK)
    {
        m_accel_calibrated = true;
        ESP_LOGI(TAG, "Accelerometer calibration saved to NVS.");
    }
    return err;
}

/**
 * @brief Run interactive gyroscope calibration and save to NVS.
 * @param timeout_s Maximum seconds to wait for interactive calibration.
 * @return ESP_OK on success.
 */
esp_err_t OrientationHandler::calibrate_gyro(uint32_t timeout_s)
{
    ESP_LOGI(TAG, "Starting Gyroscope Calibration. Keep robot COMPLETELY STILL!");
    for (int i = 3; i > 0; i--)
    {
        ESP_LOGI(TAG, "%d...", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    ITG3200 & gyro = m_driver.get_gyro();
    gyro.calibrate(1000);

    // Apply calibration to driver (ITG3200 only has offset, no scale)
    int16_t x_off, y_off, z_off;
    gyro.get_offsets(&x_off, &y_off, &z_off);
    Vector3f scale = {1.0f, 1.0f, 1.0f};  // ITG3200 doesn't use scale
    Vector3f offset = {
      static_cast<float>(x_off), static_cast<float>(y_off), static_cast<float>(z_off)};
    m_driver.set_gyro_calibration(scale, offset);

    // Save to NVS
    esp_err_t err = save_gyro_calibration_to_nvs();
    if (err == ESP_OK)
    {
        m_gyro_calibrated = true;
        ESP_LOGI(TAG, "Gyroscope calibration saved to NVS.");
    }
    return err;
}

/**
 * @brief Launch the IMU update task at 100Hz.
 * Must be called after init().
 */
void OrientationHandler::start_task()
{
    configASSERT(m_task_handle == nullptr);

    xTaskCreate(task_wrapper, "orient_task", TASK_STACK_BYTES, this, TASK_PRIORITY, &m_task_handle);

    ESP_LOGI(TAG, "Orientation task started at %lu Hz.", (unsigned long)TASK_RATE_HZ);
}

/**
 * @brief FreeRTOS task entry point — casts argument to OrientationHandler and calls task_loop().
 *
 * @param arg Pointer to OrientationHandler instance
 */
void OrientationHandler::task_wrapper(void * arg)
{
    static_cast<OrientationHandler *>(arg)->task_loop();
}

/**
 * @brief Main task loop — runs update_ahrs() at 100Hz using vTaskDelayUntil for precise timing.
 */
void OrientationHandler::task_loop()
{
    TickType_t xLastWake = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(1000 / TASK_RATE_HZ);

    while (true)
    {
        vTaskDelayUntil(&xLastWake, xPeriod);
        update_ahrs();
    }
}

/**
 * @brief Read calibrated IMU sensors, update AHRS fusion, and cache orientation data.
 *
 * Reads accelerometer, gyroscope, and magnetometer from the driver. Applies gyroscope
 * offset correction, updates the AHRS state using Madgwick/Mahony fusion (no magnetometer),
 * converts quaternion to Euler angles (roll, pitch, yaw), and caches all raw and fused
 * data with mutex protection for thread-safe access.
 */
void OrientationHandler::update_ahrs()
{
    // Read calibrated sensor data from driver
    Vector3f accel = m_driver.read_acceleration_calibrated();
    Vector3f gyro = m_driver.read_gyro_calibrated();
    Vector3f mag = m_driver.read_magnetometer_calibrated();

    // Convert to Fusion types
    FusionVector accelerometer;
    accelerometer.axis.x = accel.x;
    accelerometer.axis.y = accel.y;
    accelerometer.axis.z = accel.z;

    FusionVector gyroscope;
    gyroscope.axis.x = gyro.x;
    gyroscope.axis.y = gyro.y;
    gyroscope.axis.z = gyro.z;

    // Apply gyroscope offset
    gyroscope = FusionOffsetUpdate(&m_offset, gyroscope);

    // Update AHRS (no magnetometer for now)
    FusionAhrsUpdateNoMagnetometer(&m_ahrs, gyroscope, accelerometer, 1.0f / TASK_RATE_HZ);

    // Convert quaternion to Euler angles
    FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&m_ahrs));

    // Cache data with mutex protection
    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE)
    {
        m_roll = euler.angle.roll;
        m_pitch = euler.angle.pitch;
        m_yaw = euler.angle.yaw;

        m_ax = accel.x;
        m_ay = accel.y;
        m_az = accel.z;

        m_gx = gyro.x;
        m_gy = gyro.y;
        m_gz = gyro.z;

        m_mx = mag.x;
        m_my = mag.y;
        m_mz = mag.z;

        xSemaphoreGive(m_mutex);
    }
    else
    {
        ESP_LOGW(TAG, "update_ahrs: mutex timeout, sensor data skipped");
    }
}

bool OrientationHandler::is_accel_calibrated() const
{
    return m_accel_calibrated;
}

bool OrientationHandler::is_gyro_calibrated() const
{
    return m_gyro_calibrated;
}

bool OrientationHandler::is_mag_calibrated() const
{
    return m_mag_calibrated;
}

/**
 * @brief Save accelerometer calibration to NVS.
 * @return ESP_OK on success, esp_err_t error code on failure.
 */
esp_err_t OrientationHandler::save_accel_calibration_to_nvs()
{
    ADXL345 & accel = m_driver.get_accel();
    return accel.save_calibration_to_nvs();
}

/**
 * @brief Save gyroscope calibration to NVS.
 * @return ESP_OK on success, esp_err_t error code on failure.
 */
esp_err_t OrientationHandler::save_gyro_calibration_to_nvs()
{
    ITG3200 & gyro = m_driver.get_gyro();
    return gyro.save_calibration_to_nvs();
}

/**
 * @brief Save magnetometer calibration to NVS.
 * @return ESP_OK on success, esp_err_t error code on failure.
 */
esp_err_t OrientationHandler::save_mag_calibration_to_nvs()
{
    QMC5883L & mag = m_driver.get_mag();
    return mag.save_calibration_to_nvs();
}

/**
 * @brief Load accelerometer calibration from NVS and apply to driver.
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if no calibration data exists.
 */
esp_err_t OrientationHandler::load_accel_calibration_from_nvs()
{
    ADXL345 & accel = m_driver.get_accel();
    esp_err_t err = accel.load_calibration_from_nvs();

    if (err == ESP_OK)
    {
        Vector3f scale = {accel.get_calibration_scale(0),
                          accel.get_calibration_scale(1),
                          accel.get_calibration_scale(2)};
        Vector3f offset = {accel.get_calibration_offset(0),
                           accel.get_calibration_offset(1),
                           accel.get_calibration_offset(2)};
        m_driver.set_accel_calibration(scale, offset);
    }

    return err;
}

/**
 * @brief Load gyroscope calibration from NVS and apply to driver.
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if no calibration data exists.
 */
esp_err_t OrientationHandler::load_gyro_calibration_from_nvs()
{
    ITG3200 & gyro = m_driver.get_gyro();
    esp_err_t err = gyro.load_calibration_from_nvs();

    if (err == ESP_OK)
    {
        // ITG3200 only has offset, no scale
        int16_t x_off, y_off, z_off;
        gyro.get_offsets(&x_off, &y_off, &z_off);
        Vector3f scale = {1.0f, 1.0f, 1.0f};
        Vector3f offset = {
          static_cast<float>(x_off), static_cast<float>(y_off), static_cast<float>(z_off)};
        m_driver.set_gyro_calibration(scale, offset);
    }

    return err;
}

/**
 * @brief Load magnetometer calibration from NVS and apply to driver.
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if no calibration data exists.
 */
esp_err_t OrientationHandler::load_mag_calibration_from_nvs()
{
    QMC5883L & mag = m_driver.get_mag();
    esp_err_t err = mag.load_calibration_from_nvs();

    if (err == ESP_OK)
    {
        Vector3f scale = {
          mag.get_calibration_scale(0), mag.get_calibration_scale(1), mag.get_calibration_scale(2)};
        Vector3f offset = {mag.get_calibration_offset(0),
                           mag.get_calibration_offset(1),
                           mag.get_calibration_offset(2)};
        m_driver.set_mag_calibration(scale, offset);
    }

    return err;
}

/**
 * @brief Get current yaw angle from AHRS (degrees, ENU convention).
 * @return Yaw angle in degrees, or 0.0 if mutex timeout.
 */
double OrientationHandler::get_yaw() const
{
    double v = 0.0;
    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE)
    {
        v = m_yaw;
        xSemaphoreGive(m_mutex);
    }
    return v;
}

/**
 * @brief Get current pitch angle from AHRS (degrees, ENU convention).
 * @return Pitch angle in degrees, or 0.0 if mutex timeout.
 */
double OrientationHandler::get_pitch() const
{
    double v = 0.0;
    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE)
    {
        v = m_pitch;
        xSemaphoreGive(m_mutex);
    }
    return v;
}

/**
 * @brief Get current roll angle from AHRS (degrees, ENU convention).
 * @return Roll angle in degrees, or 0.0 if mutex timeout.
 */
double OrientationHandler::get_roll() const
{
    double v = 0.0;
    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE)
    {
        v = m_roll;
        xSemaphoreGive(m_mutex);
    }
    return v;
}

/**
 * @brief Get calibrated accelerometer readings (m/s²).
 * @param ax Pointer to store X-axis acceleration
 * @param ay Pointer to store Y-axis acceleration
 * @param az Pointer to store Z-axis acceleration
 */
void OrientationHandler::get_acceleration(double * ax, double * ay, double * az) const
{
    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE)
    {
        *ax = m_ax;
        *ay = m_ay;
        *az = m_az;
        xSemaphoreGive(m_mutex);
    }
}

/**
 * @brief Get calibrated gyroscope readings (rad/s).
 * @param gx Pointer to store X-axis angular velocity
 * @param gy Pointer to store Y-axis angular velocity
 * @param gz Pointer to store Z-axis angular velocity
 */
void OrientationHandler::get_gyro(double * gx, double * gy, double * gz) const
{
    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE)
    {
        *gx = m_gx;
        *gy = m_gy;
        *gz = m_gz;
        xSemaphoreGive(m_mutex);
    }
}

/**
 * @brief Get calibrated magnetometer readings (µT).
 * @param mx Pointer to store X-axis magnetic field
 * @param my Pointer to store Y-axis magnetic field
 * @param mz Pointer to store Z-axis magnetic field
 */
void OrientationHandler::get_magnetometer(double * mx, double * my, double * mz) const
{
    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE)
    {
        *mx = m_mx;
        *my = m_my;
        *mz = m_mz;
        xSemaphoreGive(m_mutex);
    }
}

/**
 * @brief Set the AHRS fusion gain (convergence speed vs noise rejection).
 * @param gain Fusion gain value (typical: 0.5)
 */
void OrientationHandler::set_fusion_gain(float gain)
{
    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE)
    {
        m_settings.gain = gain;
        FusionAhrsSetSettings(&m_ahrs, &m_settings);
        xSemaphoreGive(m_mutex);
    }
    else
    {
        ESP_LOGW(TAG, "set_fusion_gain: mutex timeout, setting skipped");
    }
}

/**
 * @brief Set acceleration rejection threshold for AHRS (degrees).
 * @param degrees Maximum allowable acceleration deviation before fusion is rejected
 */
void OrientationHandler::set_acceleration_rejection(float degrees)
{
    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE)
    {
        m_settings.accelerationRejection = degrees;
        FusionAhrsSetSettings(&m_ahrs, &m_settings);
        xSemaphoreGive(m_mutex);
    }
    else
    {
        ESP_LOGW(TAG, "set_acceleration_rejection: mutex timeout, setting skipped");
    }
}

/**
 * @brief Set magnetic rejection threshold for AHRS (degrees).
 * @param degrees Maximum allowable magnetic deviation before fusion is rejected
 */
void OrientationHandler::set_magnetic_rejection(float degrees)
{
    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE)
    {
        m_settings.magneticRejection = degrees;
        FusionAhrsSetSettings(&m_ahrs, &m_settings);
        xSemaphoreGive(m_mutex);
    }
    else
    {
        ESP_LOGW(TAG, "set_magnetic_rejection: mutex timeout, setting skipped");
    }
}
