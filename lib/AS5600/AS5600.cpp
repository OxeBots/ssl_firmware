#include "AS5600.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <algorithm>
#include <numeric>

static const char * TAG = "AS5600";

AS5600::AS5600(adc_channel_t channel, adc_cali_handle_t cali_handle, bool voltage_calibrated, adc_unit_t unit,
               adc_bitwidth_t bitwidth)
: ADC_BITWIDTH(bitwidth), m_channel(channel), m_cali_handle(cali_handle), m_is_voltage_calibrated(voltage_calibrated)
{
    m_filter = std::make_unique<WheelKalmanFilter>();
}

esp_err_t AS5600::init_i2c()
{
    if (m_i2c_initialized)
        return ESP_ERR_INVALID_STATE;

    // Test communication by reading a register
    uint16_t test_conf;
    if (read_config_register(&test_conf) != 0)
    {
        ESP_LOGE(TAG, "Failed to communicate with AS5600 sensor via I2C.");
        return ESP_FAIL;
    }

    m_i2c_initialized = true;
    ESP_LOGI(TAG, "AS5600 I2C interface initialized successfully.");
    return ESP_OK;
}

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

esp_err_t AS5600::burn_settings()
{
    if (!m_i2c_initialized)
        return ESP_ERR_INVALID_STATE;

    ESP_LOGW(TAG, "Burning settings to AS5600 OTP memory. This is permanent!");
    uint8_t cmd = static_cast<uint8_t>(Cmd::BURN);
    if (!I2Cdev::writeByte(AS5600_ADDR, static_cast<uint8_t>(Register::BURN), cmd))
        return ESP_FAIL;

    vTaskDelay(pdMS_TO_TICKS(10));  // Wait for burn to complete
    return ESP_OK;
}

esp_err_t AS5600::get_i2c_raw_angle(uint16_t * angle)
{
    if (!m_i2c_initialized)
        return ESP_ERR_INVALID_STATE;

    if (I2Cdev::readWord(AS5600_ADDR, static_cast<uint8_t>(Register::RAW_ANGLE_H), angle) != 0)
        return ESP_FAIL;

    // AS5600 returns 12 bits
    *angle &= 0x0FFF;
    return ESP_OK;
}

void AS5600::process_new_reading(uint16_t raw_adc_value)
{
    // --- Step 1: Oversampling ---
    if (m_sampling_state.count < OVERSAMPLE_COUNT)
        m_sampling_state.samples[m_sampling_state.count++] = raw_adc_value;

    // When the sampling buffer is full, process the data
    if (m_sampling_state.count >= OVERSAMPLE_COUNT)
    {
        uint32_t sum = std::accumulate(m_sampling_state.samples.begin(), m_sampling_state.samples.end(), 0u);
        m_last_avg_value = sum / OVERSAMPLE_COUNT;

        // --- Step 2: Convert to Angle ---
        int current_voltage_mv = 0;
        if (m_is_voltage_calibrated)
            adc_cali_raw_to_voltage(m_cali_handle, m_last_avg_value, &current_voltage_mv);

        if (m_is_calibrating)
        {
            // If we are calibrating, we need to update the min/max voltage range
            if (current_voltage_mv < m_min_voltage_mv)
                m_min_voltage_mv = current_voltage_mv;
            if (current_voltage_mv > m_max_voltage_mv)
                m_max_voltage_mv = current_voltage_mv;
        }

        float measured_angle_rad = 0.0f;
        if (m_max_voltage_mv > m_min_voltage_mv)
        {
            int clamped_mv = std::clamp(current_voltage_mv, (int)m_min_voltage_mv, (int)m_max_voltage_mv);
            float ratio = static_cast<float>(clamped_mv - m_min_voltage_mv) / (m_max_voltage_mv - m_min_voltage_mv);
            measured_angle_rad = ratio * 2.0f * PI;
        }
        else
        {
            // Fallback if not calibrated: map raw value to angle
            const float max_raw = static_cast<float>((1 << ADC_BITWIDTH) - 1);
            measured_angle_rad = static_cast<float>(m_last_avg_value) * (2.0f * PI / max_raw);
        }

        // --- Step 3: Update Kalman Filter ---
        m_filter->update(measured_angle_rad);

        // Reset sampling
        m_sampling_state.count = 0;
    }
}

esp_err_t AS5600::calibrate_range(uint32_t duration_ms)
{
    if (!m_is_voltage_calibrated)
    {
        ESP_LOGE(TAG, "Channel %d must be voltage-calibrated first.", m_channel);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting range calibration for channel %d. Rotate sensor now...", m_channel);

    m_is_calibrating = true;

    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    m_is_calibrating = false;

    ESP_LOGI(TAG, "Channel %d calibrated. Min: %d mV, Max: %d mV", m_channel, m_min_voltage_mv, m_max_voltage_mv);

    // Sanity check
    if (m_max_voltage_mv - m_min_voltage_mv < MIN_VALID_VOLTAGE_RANGE_MV)
    {
        ESP_LOGE(TAG, "Error: Voltage range for channel %d is invalid (%d mV).", m_channel,
                 m_max_voltage_mv - m_min_voltage_mv);
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

float AS5600::get_angle_rad() const
{
    return m_filter->get_angle_rad();
}

float AS5600::get_angle_deg() const
{
    return m_filter->get_angle_rad() * RAD_TO_DEG;
}

float AS5600::get_rpm() const
{
    return m_filter->get_velocity_rad_s() * RAD_S_TO_RPM;
}

float AS5600::get_acceleration_rps2() const
{
    return m_filter->get_acceleration_rad_s2() * RAD_S2_TO_RPS2;
}

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

uint16_t AS5600::get_last_raw_value() const
{
    return m_last_avg_value;
}

esp_err_t AS5600::read_config_register(uint16_t * config)
{
    if (I2Cdev::readWord(AS5600_ADDR, static_cast<uint8_t>(Register::CONF_H), config) != 0)
        return ESP_FAIL;

    return ESP_OK;
}

esp_err_t AS5600::write_config_register(uint16_t config)
{
    if (!I2Cdev::writeWord(AS5600_ADDR, static_cast<uint8_t>(Register::CONF_H), config))
        return ESP_FAIL;

    return ESP_OK;
}

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

    m_min_voltage_mv = min_mv;
    m_max_voltage_mv = max_mv;

    return ESP_OK;
}

void AS5600::reset_calibration_min_max()
{
    // Set min to a value higher than any possible reading (3.3V = 3300mV)
    m_min_voltage_mv = 5000;
    // Set max to a value lower than any possible reading
    m_max_voltage_mv = 0;
}

void AS5600::start_calibration_mode()
{
    m_is_calibrating = true;
}

void AS5600::stop_calibration_mode()
{
    m_is_calibrating = false;
}

int AS5600::get_calib_min() const
{
    return m_min_voltage_mv;
}

int AS5600::get_calib_max() const
{
    return m_max_voltage_mv;
}
