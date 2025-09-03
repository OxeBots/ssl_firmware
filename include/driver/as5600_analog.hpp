/**
 * @file as5600_analog.hpp
 * @brief Manages the state and filtering for a single as5600 connected by an analog interface.
 */
#ifndef DRIVER_AS5600_ANALOG_HPP
#define DRIVER_AS5600_ANALOG_HPP

#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>

#include <array>
#include <cstdint>
#include <memory>

#include "helper_func.h"
#include "kinematics/wheel_kalman_filter.hpp"

class AS5600_analog
{
   public:
    explicit AS5600_analog(adc_channel_t channel, adc_cali_handle_t cali_handle, bool voltage_calibrated,
                           adc_unit_t unit = ADC_UNIT_1, adc_bitwidth_t bitwidth = ADC_BITWIDTH_12);
    ~AS5600_analog();

    /**
     * @brief Processes a new raw ADC value from the hardware.
     * This method handles oversampling and feeding the result to the Kalman filter.
     * @param raw_adc_value The new 12-bit ADC reading.
     */
    void process_new_reading(uint16_t raw_adc_value);

    /**
     * @brief Calibrates the voltage for the specified ADC unit and attenuation linked to the
     * channel that this sensor is connected to.
     * @param unit The ADC unit.
     * @param attenuation The ADC attenuation level.
     * @return ESP_OK on success.
     */
    // esp_err_t calibrate_voltage(adc_unit_t unit, adc_atten_t attenuation);

    /**
     * @brief Calibrates the min/max voltage range for angle conversion.
     * @param duration_ms The duration for the calibration process.
     * @return ESP_OK on success.
     */
    esp_err_t calibrate_range(uint32_t duration_ms);

    // --- Getters ---
    float get_angle_rad() const;
    float get_angle_deg() const;
    float get_rpm() const;
    float get_acceleration_rps2() const;
    int get_last_voltage_mv() const;
    uint16_t get_last_raw_value() const;

   private:
    const adc_bitwidth_t ADC_BITWIDTH = ADC_BITWIDTH_12;
    static constexpr int MIN_VALID_VOLTAGE_RANGE_MV = 500;
    static constexpr float RAD_TO_DEG = 180.0f / M_PI;
    static constexpr float RAD_S_TO_RPM = 60.0f / (2.0f * M_PI);
    static constexpr float RAD_S2_TO_RPS2 = 1.0f / (2.0f * M_PI);
    static constexpr size_t OVERSAMPLE_COUNT = 16;

    struct SamplingState
    {
        std::array<uint16_t, OVERSAMPLE_COUNT> samples;
        size_t count = 0;
    };

    adc_channel_t m_channel;
    std::unique_ptr<WheelKalmanFilter> m_filter;

    // ADC Calibration
    adc_cali_handle_t m_cali_handle = nullptr;
    bool m_is_voltage_calibrated = false;
    bool m_is_calibrating = false;
    int m_min_voltage_mv = 5000;
    int m_max_voltage_mv = 0;

    SamplingState m_sampling_state;
    uint16_t m_last_avg_value = 0;
};

#endif  // DRIVER_AS5600_ANALOG_HPP
