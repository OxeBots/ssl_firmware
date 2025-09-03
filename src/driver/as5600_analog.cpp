#include "driver/as5600_analog.hpp"

#include <algorithm>
#include <numeric>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char * TAG = "AS5600_analog";

AS5600_analog::AS5600_analog(adc_channel_t channel, adc_cali_handle_t cali_handle, bool voltage_calibrated,
                             adc_unit_t unit, adc_bitwidth_t bitwidth)
: ADC_BITWIDTH(bitwidth), m_channel(channel), m_cali_handle(cali_handle), m_is_voltage_calibrated(voltage_calibrated)
{
    m_filter = std::make_unique<WheelKalmanFilter>();
}

AS5600_analog::~AS5600_analog()
{
}

void AS5600_analog::process_new_reading(uint16_t raw_adc_value)
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
            int clamped_mv = std::clamp(current_voltage_mv, m_min_voltage_mv, m_max_voltage_mv);
            float ratio = static_cast<float>(clamped_mv - m_min_voltage_mv) / (m_max_voltage_mv - m_min_voltage_mv);
            measured_angle_rad = ratio * 2.0f * M_PI;
        }
        else
        {
            // Fallback if not calibrated: map raw value to angle
            const float max_raw = static_cast<float>((1 << ADC_BITWIDTH) - 1);
            measured_angle_rad = static_cast<float>(m_last_avg_value) * (2.0f * M_PI / max_raw);
        }

        // --- Step 3: Update Kalman Filter ---
        m_filter->update(measured_angle_rad);

        // Reset sampling
        m_sampling_state.count = 0;
    }
}

esp_err_t AS5600_analog::calibrate_range(uint32_t duration_ms)
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

// --- Getters ---

float AS5600_analog::get_angle_rad() const
{
    return m_filter->get_angle_rad();
}

float AS5600_analog::get_angle_deg() const
{
    return m_filter->get_angle_rad() * RAD_TO_DEG;
}

float AS5600_analog::get_rpm() const
{
    return m_filter->get_velocity_rad_s() * RAD_S_TO_RPM;
}

float AS5600_analog::get_acceleration_rps2() const
{
    return m_filter->get_acceleration_rad_s2() * RAD_S2_TO_RPS2;
}

int AS5600_analog::get_last_voltage_mv() const
{
    if (m_is_voltage_calibrated)
    {
        int voltage = 0;
        adc_cali_raw_to_voltage(m_cali_handle, m_last_avg_value, &voltage);
        return voltage;
    }
    return -1;
}

uint16_t AS5600_analog::get_last_raw_value() const
{
    return m_last_avg_value;
}
