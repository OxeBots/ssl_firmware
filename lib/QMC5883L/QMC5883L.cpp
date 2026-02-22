/**
 * @file QMC5883L.cpp
 * @brief Driver implementation for the QMC5883L I2C magnetometer sensor.
 */
#include "QMC5883L.h"

static const char * TAG = "QMC5883L";
static const char * NVS_NS = "mag_calib";
static const char * NVS_KEY_BLOB = "calib_blob";

/**
 * @brief Constructor with specified I2C address.
 * @param address I2C address of the QMC5883L device.
 */
QMC5883L::QMC5883L(uint8_t address) : m_dev_addr(address)
{
}

/**
 * @brief Initialize the QMC5883L sensor with default continuous settings.
 */
void QMC5883L::init()
{
    // Define Set/Reset Period (Recommended by datasheet)
    I2Cdev::writeByte(m_dev_addr, static_cast<uint8_t>(Register::PERIOD), 0x01);

    // Set Continuous Mode, 200Hz, 8G Range, OSR 512
    set_mode(Mode::CONTINUOUS, OutputDataRate::ODR_200HZ, Range::RNG_8G, Oversampling::OSR_512);
}

/**
 * @brief Overrides the I2C address.
 * @param hex New I2C address.
 */
void QMC5883L::set_addr(uint8_t hex)
{
    m_dev_addr = hex;
}

/**
 * @brief Set the configuration for the sensor.
 * @param mode Operational mode (Standby/Continuous)
 * @param odr Output Data Rate
 * @param rng Magnetic Range
 * @param osr Oversampling Ratio
 */
void QMC5883L::set_mode(Mode mode, OutputDataRate odr, Range rng, Oversampling osr)
{
    m_mode = mode;
    m_odr = odr;
    m_rng = rng;
    m_osr = osr;

    uint8_t config_val = static_cast<uint8_t>(m_mode) | static_cast<uint8_t>(m_odr) | static_cast<uint8_t>(m_rng) |
                         static_cast<uint8_t>(m_osr);

    I2Cdev::writeByte(m_dev_addr, static_cast<uint8_t>(Register::CONTROL_1), config_val);
}

/**
 * @brief Perform a software reset.
 */
void QMC5883L::set_reset()
{
    I2Cdev::writeByte(m_dev_addr, static_cast<uint8_t>(Register::CONTROL_2), 0x80);
}

/**
 * @brief Set local magnetic declination to accurately convert magnetic North to true North.
 * @param degrees Declination degrees.
 * @param minutes Declination minutes.
 */
void QMC5883L::set_magnetic_declination(int degrees, uint8_t minutes)
{
    m_magnetic_declination_degrees = degrees + (minutes / 60.0f);
}

/**
 * @brief Configure internal software data smoothing (moving average).
 * @param steps Number of past samples to retain (up to 10).
 * @param adv Whether to use advanced smoothing (discard highest/lowest samples).
 */
void QMC5883L::set_smoothing(uint8_t steps, bool adv)
{
    m_smooth_use = true;
    m_smooth_steps = (steps > 10) ? 10 : steps;
    m_smooth_advanced = adv;

    memset(m_v_history, 0, sizeof(m_v_history));
    m_v_totals[0] = m_v_totals[1] = m_v_totals[2] = 0;
    m_v_scan = 0;
}

/**
 * @brief Clears all internal calibration offsets and scales.
 */
void QMC5883L::clear_calibration()
{
    set_calibration_offsets(0.0f, 0.0f, 0.0f);
    set_calibration_scales(1.0f, 1.0f, 1.0f);
}

/**
 * @brief Manually sets calibration offsets.
 * @param x_offset Offset for the X-axis.
 * @param y_offset Offset for the Y-axis.
 * @param z_offset Offset for the Z-axis.
 */
void QMC5883L::set_calibration_offsets(float x_offset, float y_offset, float z_offset)
{
    m_offset[0] = x_offset;
    m_offset[1] = y_offset;
    m_offset[2] = z_offset;
}

/**
 * @brief Manually sets calibration scales.
 * @param x_scale Scale for the X-axis.
 * @param y_scale Scale for the Y-axis.
 * @param z_scale Scale for the Z-axis.
 */
void QMC5883L::set_calibration_scales(float x_scale, float y_scale, float z_scale)
{
    m_scale[0] = x_scale;
    m_scale[1] = y_scale;
    m_scale[2] = z_scale;
}

/**
 * @brief Get the calibration offset for a given axis.
 * @param index 0=X, 1=Y, 2=Z
 * @return Axis offset.
 */
float QMC5883L::get_calibration_offset(uint8_t index) const
{
    return (index < 3) ? m_offset[index] : 0.0f;
}

/**
 * @brief Get the calibration scale for a given axis.
 * @param index 0=X, 1=Y, 2=Z
 * @return Axis scale.
 */
float QMC5883L::get_calibration_scale(uint8_t index) const
{
    return (index < 3) ? m_scale[index] : 1.0f;
}

/**
 * @brief Start a non-blocking calibration session.
 * @param seconds Duration in seconds.
 */
void QMC5883L::start_calibration_mode(uint32_t seconds)
{
    for (int i = 0; i < 3; i++)
    {
        m_calib_min[i] = 32767;
        m_calib_max[i] = -32768;
    }
    m_calib_duration_us = seconds * 1000000ULL;
    m_calib_start_time = esp_timer_get_time();
    m_calib_active = true;
}

/**
 * @brief Update calibration state (must be called periodically during calibration).
 * @return true if calibration duration has finished, false if still running.
 */
bool QMC5883L::calibration_update()
{
    if (!m_calib_active)
        return true;

    read();

    for (int i = 0; i < 3; i++)
    {
        if (m_v_raw[i] < m_calib_min[i])
            m_calib_min[i] = m_v_raw[i];
        if (m_v_raw[i] > m_calib_max[i])
            m_calib_max[i] = m_v_raw[i];
    }

    if ((esp_timer_get_time() - m_calib_start_time) >= m_calib_duration_us)
        return true;

    return false;
}

/**
 * @brief Finalizes calibration, computes the offsets/scales, and applies them.
 */
void QMC5883L::stop_calibration_mode()
{
    if (!m_calib_active)
        return;

    // Calculate offsets as the average of min and max, and scales to normalize the range to be equal across axes
    float x_avg = (m_calib_max[0] + m_calib_min[0]) / 2.0f;
    float y_avg = (m_calib_max[1] + m_calib_min[1]) / 2.0f;
    float z_avg = (m_calib_max[2] + m_calib_min[2]) / 2.0f;

    set_calibration_offsets(x_avg, y_avg, z_avg);

    // Calculate scales to normalize the half-range of each axis to be the same (assuming the true magnetic field
    // strength is similar across axes)
    float x_half_range = (m_calib_max[0] - m_calib_min[0]) / 2.0f;
    float y_half_range = (m_calib_max[1] - m_calib_min[1]) / 2.0f;
    float z_half_range = (m_calib_max[2] - m_calib_min[2]) / 2.0f;

    float avg_half_range = (x_half_range + y_half_range + z_half_range) / 3.0f;

    if (x_half_range != 0)
        m_scale[0] = avg_half_range / x_half_range;
    if (y_half_range != 0)
        m_scale[1] = avg_half_range / y_half_range;
    if (z_half_range != 0)
        m_scale[2] = avg_half_range / z_half_range;

    set_calibration_scales(m_scale[0], m_scale[1], m_scale[2]);

    ESP_LOGI(TAG, "Calibration Results: Offsets[%.2f, %.2f, %.2f], Scales[%.2f, %.2f, %.2f]", m_offset[0], m_offset[1],
             m_offset[2], m_scale[0], m_scale[1], m_scale[2]);

    m_calib_active = false;
}

/**
 * @brief Saves current calibration data (offsets and scales) to NVS via NVSManager.
 * @return ESP_OK on success.
 */
esp_err_t QMC5883L::save_calibration_to_nvs()
{
    float data[6] = {m_offset[0], m_offset[1], m_offset[2], m_scale[0], m_scale[1], m_scale[2]};
    esp_err_t err = NVSManager::save_blob(NVS_NS, NVS_KEY_BLOB, data, sizeof(data));

    if (err == ESP_OK)
        ESP_LOGI(TAG, "Magnetometer calibration saved to NVS.");

    return err;
}

/**
 * @brief Loads calibration data (offsets and scales) from NVS via NVSManager.
 * @return ESP_OK on success.
 */
esp_err_t QMC5883L::load_calibration_from_nvs()
{
    float data[6];
    size_t req_size = sizeof(data);

    esp_err_t err = NVSManager::load_blob(NVS_NS, NVS_KEY_BLOB, data, &req_size);
    if (err == ESP_OK)
    {
        if (req_size == sizeof(data))
        {
            m_offset[0] = data[0];
            m_offset[1] = data[1];
            m_offset[2] = data[2];
            m_scale[0] = data[3];
            m_scale[1] = data[4];
            m_scale[2] = data[5];
            ESP_LOGI(TAG, "Loaded magnetometer calibration: Off[%.2f, %.2f, %.2f] Scl[%.2f, %.2f, %.2f]", m_offset[0],
                     m_offset[1], m_offset[2], m_scale[0], m_scale[1], m_scale[2]);
            return ESP_OK;
        }
        else
        {
            ESP_LOGE(TAG, "NVS Blob size mismatch! Expected %zu, got %zu", sizeof(data), req_size);
            return ESP_ERR_NVS_INVALID_LENGTH;
        }
    }
    else if (err == ESP_ERR_NVS_NOT_FOUND)
        ESP_LOGW(TAG, "Magnetometer calibration not found in NVS.");
    else
        ESP_LOGE(TAG, "Error reading calibration from NVS: %s", esp_err_to_name(err));

    return err;
}

/**
 * @brief Checks if valid (non-default) calibration data is currently loaded.
 * @return true if calibrated.
 */
bool QMC5883L::is_calibrated() const
{
    return !((m_scale[0] == 1.0f) &&   //
             (m_scale[1] == 1.0f) &&   //
             (m_scale[2] == 1.0f) &&   //
             (m_offset[0] == 0.0f) &&  //
             (m_offset[1] == 0.0f) &&  //
             (m_offset[2] == 0.0f));
}

/**
 * @brief Reads new data from the sensor. Must be called periodically to update values.
 */
void QMC5883L::read()
{
    // sizeof(m_buffer) limits FlawFinder boundaries warning.
    if (I2Cdev::readBytes(m_dev_addr, static_cast<uint8_t>(Register::DATAX_L), sizeof(m_buffer), m_buffer) ==
        sizeof(m_buffer))
    {
        m_v_raw[0] = (int16_t)(((uint16_t)m_buffer[1] << 8) | m_buffer[0]);
        m_v_raw[1] = (int16_t)(((uint16_t)m_buffer[3] << 8) | m_buffer[2]);
        m_v_raw[2] = (int16_t)(((uint16_t)m_buffer[5] << 8) | m_buffer[4]);

        apply_calibration();

        if (m_smooth_use)
            apply_smoothing();
    }
}

/**
 * @brief Computes m_v_calibrated from m_v_raw using offsets and scales.
 */
void QMC5883L::apply_calibration()
{
    m_v_calibrated[0] = (m_v_raw[0] - m_offset[0]) * m_scale[0];
    m_v_calibrated[1] = (m_v_raw[1] - m_offset[1]) * m_scale[1];
    m_v_calibrated[2] = (m_v_raw[2] - m_offset[2]) * m_scale[2];
}

/**
 * @brief Triggers a read and populates pointers with the calibrated 3-axis values.
 * @param x Pointer to X container.
 * @param y Pointer to Y container.
 * @param z Pointer to Z container.
 */
void QMC5883L::get_orientation(int16_t * x, int16_t * y, int16_t * z)
{
    read();
    *x = get_x();
    *y = get_y();
    *z = get_z();
}

/**
 * @brief Applies historical smoothing to the raw values.
 */
void QMC5883L::apply_smoothing()
{
    int max_idx = 0;
    int min_idx = 0;

    if (m_v_scan >= m_smooth_steps)
        m_v_scan = 0;

    for (int i = 0; i < 3; i++)
    {
        if (m_v_totals[i] != 0)
            m_v_totals[i] -= m_v_history[m_v_scan][i];

        m_v_history[m_v_scan][i] = m_v_calibrated[i];
        m_v_totals[i] += m_v_history[m_v_scan][i];

        if (m_smooth_advanced)
        {
            max_idx = min_idx = 0;
            for (int j = 0; j < m_smooth_steps; j++)
            {
                if (m_v_history[j][i] > m_v_history[max_idx][i])
                    max_idx = j;
                if (m_v_history[j][i] < m_v_history[min_idx][i])
                    min_idx = j;
            }

            int32_t sum = m_v_totals[i] - (m_v_history[max_idx][i] + m_v_history[min_idx][i]);
            if (m_smooth_steps > 2)
                m_v_smooth[i] = sum / (m_smooth_steps - 2);
            else
                m_v_smooth[i] = m_v_totals[i] / m_smooth_steps;
        }
        else
        {
            m_v_smooth[i] = m_v_totals[i] / m_smooth_steps;
        }
    }

    m_v_scan++;
}

/**
 * @brief Safely retrieve the final computed axis.
 * @param index Axis index (0=X, 1=Y, 2=Z)
 * @return Output value for the axis.
 */
int16_t QMC5883L::get_axis(int index) const
{
    if (index < 0 || index > 2)
        return 0;

    if (m_smooth_use)
        return m_v_smooth[index];

    return m_v_calibrated[index];
}

/**
 * @brief Computes the azimuth (heading) taking into account calibration and magnetic declination.
 * @return Azimuth in degrees (0 to 359).
 */
int QMC5883L::get_azimuth() const
{
    float heading = atan2((float)get_y(), (float)get_x()) * 180.0 / M_PI;
    heading += m_magnetic_declination_degrees;

    while (heading < 0) heading += 360;

    while (heading >= 360) heading -= 360;

    return static_cast<int>(heading);
}

/**
 * @brief Converts an azimuth degree into a 16-point compass bearing index.
 * @param azimuth Calculated azimuth.
 * @return Bearing index (0-15).
 */
uint8_t QMC5883L::get_bearing(int azimuth) const
{
    float sector = static_cast<float>(azimuth) / 22.5f;
    int bearing = static_cast<int>(sector + 0.5f);
    return static_cast<uint8_t>(bearing % 16);
}

/**
 * @brief Retrieves a 3-character string representing the compass direction (e.g. "N  ", "NNE").
 * @param myArray Buffer to place the resulting string (must be at least 4 bytes).
 * @param azimuth Calculated azimuth.
 */
void QMC5883L::get_direction(char * myArray, int azimuth) const
{
    int d = get_bearing(azimuth);
    myArray[0] = m_bearings[d][0];
    myArray[1] = m_bearings[d][1];
    myArray[2] = m_bearings[d][2];
    myArray[3] = '\0';
}

/**
 * @brief Reads the device chip ID from the hardware.
 * @return Chip ID value.
 */
uint8_t QMC5883L::get_chip_id()
{
    I2Cdev::readByte(m_dev_addr, static_cast<uint8_t>(Register::CHIP_ID), m_buffer);
    return m_buffer[0];
}

/**
 * @brief Test connection to the sensor.
 * @return true if communication is successful and the chip ID matches, false otherwise.
 */
bool QMC5883L::test_connection()
{
    return get_chip_id() == 0xFF;
}

/**
 * @brief Set the magnetic field measurement range.
 * @param rng The new range (RNG_2G or RNG_8G)
 */
void QMC5883L::set_range(Range rng)
{
    m_rng = rng;
    set_mode(m_mode, m_odr, m_rng, m_osr);
}

/**
 * @brief Get the currently configured magnetic field measurement range.
 * @return Current Range
 */
QMC5883L::Range QMC5883L::get_range() const
{
    return m_rng;
}
