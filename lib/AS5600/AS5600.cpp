/**
 * @file AS5600.cpp
 * @brief Implementation of the AS5600 magnetic rotary encoder driver.
 */

#include "AS5600.h"

#include <algorithm>
#include <numeric>

#include "I2Cdev.h"
#include "NVSManager.h"

static const char * TAG = "AS5600";

/**
 * @brief Constructor for AS5600.
 * @param channel ADC channel for analog reading.
 * @param cali_handle ADC calibration handle.
 * @param voltage_calibrated Whether the ADC is voltage-calibrated.
 * @param unit ADC unit (default ADC_UNIT_1).
 * @param bitwidth ADC bitwidth (default ADC_BITWIDTH_12).
 */
AS5600::AS5600(adc_channel_t channel, adc_cali_handle_t cali_handle, bool voltage_calibrated, adc_unit_t unit,
               adc_bitwidth_t bitwidth)
: ADC_BITWIDTH(bitwidth),
  m_channel(channel),
  m_filter(std::make_unique<WheelKalmanFilter>()),
  m_cali_handle(cali_handle),
  m_is_voltage_calibrated(voltage_calibrated)
{
}

/**
 * @brief Initializes the AS5600 device via I2C. Validates connection.
 * @return ESP_OK on success, ESP_FAIL on failure, ESP_ERR_INVALID_STATE if already initialized.
 */
esp_err_t AS5600::init_i2c()
{
    if (m_i2c_initialized)
        return ESP_ERR_INVALID_STATE;

    uint16_t test_conf;
    if (read_config_register(&test_conf) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to communicate with AS5600 sensor via I2C.");
        return ESP_FAIL;
    }

    m_i2c_initialized = true;
    ESP_LOGI(TAG, "AS5600 I2C interface initialized successfully.");
    return ESP_OK;
}

/**
 * @brief Sets the analog output mode (full, reduced, PWM).
 * @param stage Selected OutputStage enum.
 * @return ESP_OK on success.
 */
esp_err_t AS5600::set_output_stage(OutputStage stage)
{
    if (!m_i2c_initialized)
        return ESP_ERR_INVALID_STATE;

    uint16_t config;
    esp_err_t ret = read_config_register(&config);
    if (ret != ESP_OK)
        return ret;

    config &= ~0x0030;  // Clear bits 4 and 5
    config |= (static_cast<uint8_t>(stage) << 4);

    return write_config_register(config);
}

/**
 * @brief Sets the internal slow filter step.
 * @param filter Selected SlowFilter enum.
 * @return ESP_OK on success.
 */
esp_err_t AS5600::set_slow_filter(SlowFilter filter)
{
    if (!m_i2c_initialized)
        return ESP_ERR_INVALID_STATE;

    uint16_t config;
    esp_err_t ret = read_config_register(&config);
    if (ret != ESP_OK)
        return ret;

    config &= ~0x0300;  // Clear bits 8 and 9
    config |= (static_cast<uint8_t>(filter) << 8);

    return write_config_register(config);
}

/**
 * @brief Sets the internal fast filter threshold.
 * @param threshold Selected FastFilter enum.
 * @return ESP_OK on success.
 */
esp_err_t AS5600::set_fast_filter(FastFilter threshold)
{
    if (!m_i2c_initialized)
        return ESP_ERR_INVALID_STATE;

    uint16_t config;
    esp_err_t ret = read_config_register(&config);
    if (ret != ESP_OK)
        return ret;

    config &= ~0x1C00;  // Clear bits 10, 11, and 12
    config |= (static_cast<uint8_t>(threshold) << 10);

    return write_config_register(config);
}

/**
 * @brief Permanently burns settings into the AS5600 OTP memory. Can only be performed 3 times per chip.
 * @return ESP_OK on success.
 */
esp_err_t AS5600::burn_settings()
{
    if (!m_i2c_initialized)
        return ESP_ERR_INVALID_STATE;

    ESP_LOGW(TAG, "Burning settings to AS5600 OTP memory. This is permanent!");
    uint8_t cmd = static_cast<uint8_t>(Cmd::BURN);
    if (!I2Cdev::writeByte(AS5600_ADDR, static_cast<uint8_t>(Register::BURN), cmd))
        return ESP_FAIL;

    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

/**
 * @brief Retrieves the raw magnetic angle using I2C (instead of ADC).
 * @param angle Pointer to output variable.
 * @return ESP_OK on success.
 */
esp_err_t AS5600::get_i2c_raw_angle(uint16_t * angle)
{
    if (!m_i2c_initialized)
        return ESP_ERR_INVALID_STATE;

    if (I2Cdev::readWord(AS5600_ADDR, static_cast<uint8_t>(Register::RAW_ANGLE_H), angle) != 0)
        return ESP_FAIL;

    *angle &= 0x0FFF;
    return ESP_OK;
}

/**
 * @brief Processes an incoming raw ADC reading, applying oversampling and feeding the Kalman filter.
 * @param raw_adc_value Raw uint16_t coming directly from the ADC buffer.
 */
void AS5600::process_new_reading(uint16_t raw_adc_value)
{
    // Oversampling
    if (m_sampling_state.count < OVERSAMPLE_COUNT)
        m_sampling_state.samples[m_sampling_state.count++] = raw_adc_value;

    // When the sampling buffer is full, process the data
    if (m_sampling_state.count >= OVERSAMPLE_COUNT)
    {
        uint32_t sum = std::accumulate(m_sampling_state.samples.begin(), m_sampling_state.samples.end(), 0u);
        m_last_avg_value = sum / OVERSAMPLE_COUNT;

        // Convert to Angle
        int current_voltage_mv = 0;
        if (m_is_voltage_calibrated)
            adc_cali_raw_to_voltage(m_cali_handle, m_last_avg_value, &current_voltage_mv);

        if (m_is_calibrating.load())
        {
            // If we are calibrating, we need to update the min/max voltage range
            int current_min = m_min_voltage_mv.load();
            int current_max = m_max_voltage_mv.load();
            if (current_voltage_mv < current_min)
                m_min_voltage_mv.store(current_voltage_mv);
            if (current_voltage_mv > current_max)
                m_max_voltage_mv.store(current_voltage_mv);
        }

        float measured_angle_rad = 0.0f;
        int cal_min = m_min_voltage_mv.load();
        int cal_max = m_max_voltage_mv.load();
        if (cal_max > cal_min)
        {
            int clamped_mv = std::clamp(current_voltage_mv, cal_min, cal_max);
            float ratio = static_cast<float>(clamped_mv - cal_min) / static_cast<float>(cal_max - cal_min);
            measured_angle_rad = ratio * 2.0f * static_cast<float>(PI);
        }
        else
        {
            // Fallback if not calibrated: map raw value to angle
            const float max_raw = static_cast<float>((1 << ADC_BITWIDTH) - 1);
            measured_angle_rad = static_cast<float>(m_last_avg_value) * (2.0f * static_cast<float>(PI) / max_raw);
        }

        // Update Kalman Filter
        m_filter->update(measured_angle_rad);
        m_sampling_state.count = 0;
    }
}

/**
 * @brief Starts an interactive calibration routine determining the true min/max analog voltages.
 * @param duration_ms Time to sample min/max readings.
 * @return ESP_OK on success.
 */
esp_err_t AS5600::calibrate_range(uint32_t duration_ms)
{
    if (!m_is_voltage_calibrated)
    {
        ESP_LOGE(TAG, "Channel %d must be voltage-calibrated first.", m_channel);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting range calibration for channel %d. Rotate sensor now...", m_channel);
    m_is_calibrating.store(true);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    m_is_calibrating.store(false);

    ESP_LOGI(TAG, "Channel %d calibrated. Min: %d mV, Max: %d mV", m_channel, m_min_voltage_mv.load(),
             m_max_voltage_mv.load());

    if (m_max_voltage_mv.load() - m_min_voltage_mv.load() < MIN_VALID_VOLTAGE_RANGE_MV)
    {
        ESP_LOGE(TAG, "Error: Voltage range for channel %d is invalid (%d mV).", m_channel,
                 m_max_voltage_mv.load() - m_min_voltage_mv.load());
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

/**
 * @brief Safely saves the encoder's min/max calibration range to NVS via NVSManager.
 * @return ESP_OK on success.
 */
esp_err_t AS5600::save_calibration_to_nvs()
{
    char key_min[15], key_max[15];
    snprintf(key_min, sizeof(key_min), "ch%d_min", static_cast<int>(m_channel));
    snprintf(key_max, sizeof(key_max), "ch%d_max", static_cast<int>(m_channel));

    esp_err_t err = NVSManager::save_i32(NVS_NS, key_min, m_min_voltage_mv.load());

    if (err == ESP_OK)
        err = NVSManager::save_i32(NVS_NS, key_max, m_max_voltage_mv.load());

    if (err == ESP_OK)
        ESP_LOGI(TAG, "Saved calibration for Ch %d: [%d, %d] mV", static_cast<int>(m_channel), m_min_voltage_mv.load(),
                 m_max_voltage_mv.load());

    return err;
}

/**
 * @brief Safely loads the encoder's min/max calibration range from NVS via NVSManager.
 * @return ESP_OK on success.
 */
esp_err_t AS5600::load_calibration_from_nvs()
{
    char key_min[15], key_max[15];
    snprintf(key_min, sizeof(key_min), "ch%d_min", static_cast<int>(m_channel));
    snprintf(key_max, sizeof(key_max), "ch%d_max", static_cast<int>(m_channel));

    int32_t min_v = 0, max_v = 0;
    esp_err_t err_min = NVSManager::load_i32(NVS_NS, key_min, &min_v);
    esp_err_t err_max = NVSManager::load_i32(NVS_NS, key_max, &max_v);

    if (err_min == ESP_OK && err_max == ESP_OK)
    {
        set_calibration_range(static_cast<int>(min_v), static_cast<int>(max_v));
        ESP_LOGI(TAG, "Loaded calibration for Ch %d: [%d, %d] mV", static_cast<int>(m_channel), m_min_voltage_mv.load(),
                 m_max_voltage_mv.load());
        return ESP_OK;
    }

    return ESP_ERR_NOT_FOUND;
}

/**
 * @brief Get the final filtered absolute angle in radians.
 * @return Angle (0.0 to 2PI)
 */
float AS5600::get_angle_rad() const
{
    return m_filter->get_angle_rad();
}

/**
 * @brief Get the final filtered absolute angle in degrees.
 * @return Angle (0.0 to 360)
 */
float AS5600::get_angle_deg() const
{
    return m_filter->get_angle_rad() * RAD_TO_DEG;
}

/**
 * @brief Get the filtered rotational velocity in RPM.
 * @return Rotational velocity (Revolutions Per Minute).
 */
float AS5600::get_rpm() const
{
    return m_filter->get_velocity_rad_s() * RAD_S_TO_RPM;
}

/**
 * @brief Get the filtered rotational acceleration.
 * @return Acceleration (Radians Per Second Squared).
 */
float AS5600::get_acceleration_rps2() const
{
    return m_filter->get_acceleration_rad_s2() * RAD_S2_TO_RPS2;
}

/**
 * @brief Get the last sampled raw voltage.
 * @return Voltage in millivolts.
 */
int AS5600::get_last_voltage_mv() const
{
    if (m_is_voltage_calibrated)
    {
        int voltage = 0;
        adc_cali_raw_to_voltage(m_cali_handle, m_last_avg_value, &voltage);
        return voltage;
    }
    return -1;
}

/**
 * @brief Get the last 12-bit ADC raw read value.
 * @return Raw value (0-4095).
 */
uint16_t AS5600::get_last_raw_value() const
{
    return m_last_avg_value;
}

/**
 * @brief Internal helper to read the CONF_H I2C register.
 * @param config Output word pointer.
 * @return ESP_OK on success.
 */
esp_err_t AS5600::read_config_register(uint16_t * config)
{
    if (I2Cdev::readWord(AS5600_ADDR, static_cast<uint8_t>(Register::CONF_H), config) != 0)
        return ESP_FAIL;

    return ESP_OK;
}

/**
 * @brief Internal helper to write the CONF_H I2C register.
 * @param config Input word.
 * @return ESP_OK on success.
 */
esp_err_t AS5600::write_config_register(uint16_t config)
{
    if (!I2Cdev::writeWord(AS5600_ADDR, static_cast<uint8_t>(Register::CONF_H), config))
        return ESP_FAIL;

    return ESP_OK;
}

/**
 * @brief Reads the currently active filter and stage configurations via I2C.
 * @param stage Pointer to receive output stage setting.
 * @param slow Pointer to receive slow filter setting.
 * @param fast Pointer to receive fast filter setting.
 * @return ESP_OK on success.
 */
esp_err_t AS5600::read_configuration(OutputStage * stage, SlowFilter * slow, FastFilter * fast)
{
    if (!m_i2c_initialized)
        return ESP_ERR_INVALID_STATE;

    uint16_t config_val;
    esp_err_t ret = read_config_register(&config_val);
    if (ret != ESP_OK)
        return ret;

    // Bit manipulation based on datasheet (Register 0x07)
    // Bits 5:4 = Output Stage
    if (stage)
        *stage = static_cast<OutputStage>((config_val >> 4) & 0x03);

    // Bits 9:8 = Slow Filter
    if (slow)
        *slow = static_cast<SlowFilter>((config_val >> 8) & 0x03);

    // Bits 12:10 = Fast Filter
    if (fast)
        *fast = static_cast<FastFilter>((config_val >> 10) & 0x07);

    return ESP_OK;
}

/**
 * @brief Manually sets the upper and lower voltage bounds for rotation mapping.
 * @param min_mv Minimum recorded voltage at 0 degrees.
 * @param max_mv Maximum recorded voltage at 360 degrees.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if invalid range.
 */
esp_err_t AS5600::set_calibration_range(int min_mv, int max_mv)
{
    if (min_mv >= max_mv)
    {
        ESP_LOGE(TAG, "Invalid calibration range: min_mv (%d) must be less than max_mv (%d)", min_mv, max_mv);
        return ESP_ERR_INVALID_ARG;
    }

    if ((min_mv < 0 || min_mv > 3300) || (max_mv < 0 || max_mv > 3300))
    {
        ESP_LOGE(TAG,
                 "Invalid calibration range: min_mv and max_mv must be non-negative and <= 3300 mV. Received "
                 "min_mv=%d, max_mv=%d",
                 min_mv, max_mv);
        return ESP_ERR_INVALID_ARG;
    }

    m_min_voltage_mv.store(min_mv);
    m_max_voltage_mv.store(max_mv);

    return ESP_OK;
}

/**
 * @brief Resets the calibration boundaries, preparing for a new tracking run.
 */
void AS5600::reset_calibration_min_max()
{
    m_min_voltage_mv.store(5000);
    m_max_voltage_mv.store(0);
}

/**
 * @brief Enables live tracking of minimum and maximum voltages during readings.
 */
void AS5600::start_calibration_mode()
{
    m_is_calibrating.store(true);
}

/**
 * @brief Disables live tracking of min/max voltages.
 */
void AS5600::stop_calibration_mode()
{
    m_is_calibrating.store(false);
}

/**
 * @brief Get currently calibrated minimum voltage.
 * @return Millivolts.
 */
int AS5600::get_calib_min() const
{
    return m_min_voltage_mv.load();
}

/**
 * @brief Get currently calibrated maximum voltage.
 * @return Millivolts.
 */
int AS5600::get_calib_max() const
{
    return m_max_voltage_mv.load();
}
