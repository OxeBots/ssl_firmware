/**
 * @file adc_reader.hpp
 * @brief This file contains the definition of the ADC_Reader class with EKF.
 */
#ifndef HAL_ADC_READER_HPP
#define HAL_ADC_READER_HPP

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>

#include <array>
#include <cmath>
#include <memory>
#include <numeric>
#include <vector>

#include "driver/adc_types_legacy.h"
#include "driver/ledc.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/adc_types.h"
#include "soc/soc_caps.h"
// NOTE: vt_kalman and vt_linalg are now included in the .cpp file

/**
 * @class ADC_Reader
 * @brief Manages ADC readings and provides Kalman-filtered estimates for
 * angle, velocity (RPM), and acceleration.
 *
 * This class uses an Extended Kalman Filter (EKF) for each channel to track
 * the rotational state. It takes noisy, oversampled angle measurements and
 * produces a smoothed, physically consistent estimate of the angle, angular
 * velocity, and angular acceleration.
 */
class ADC_Reader
{
   public:
    static constexpr size_t OVERSAMPLE_COUNT = 16;
    static constexpr float RAW_TO_RAD = (2.0f * M_PI) / 4096.0f;
    static constexpr float RAD_S_TO_RPM = 60.0f / (2.0f * M_PI);
    static constexpr float RPS2_TO_DEGS2 = 360.0f;

    struct Measurement
    {
        uint16_t value = 0;
        int64_t duration_us = 0;
    };

    ADC_Reader(const ADC_Reader &) = delete;
    ADC_Reader & operator=(const ADC_Reader &) = delete;

    static ADC_Reader & get_instance()
    {
        static ADC_Reader instance;
        return instance;
    }

    esp_err_t init(const std::vector<adc_channel_t> & channels);

    // --- Filtered State Getters ---
    float get_filtered_angle_deg(adc_channel_t channel);
    float get_filtered_rpm(adc_channel_t channel);
    float get_filtered_acceleration_rps2(adc_channel_t channel);

    // --- Raw Data Getters (for debugging) ---
    uint32_t get_value(adc_channel_t channel);
    int64_t get_measurement_duration_us(adc_channel_t channel);

   private:
    ADC_Reader();
    ~ADC_Reader();

    // Forward-declare the private implementation struct
    struct KalmanState;

    adc_continuous_handle_t m_adc_handle;
    TaskHandle_t m_task_handle;
    bool m_initialized;
    static constexpr size_t ADC_BUFFER_SIZE = 512;

    struct SamplingState
    {
        std::array<uint16_t, OVERSAMPLE_COUNT> samples;
        size_t count = 0;
    };

    std::array<SamplingState, ADC1_CHANNEL_MAX> m_sampling_states;
    std::array<Measurement, ADC1_CHANNEL_MAX> m_current_measurements;
    // Use a unique_ptr to hide the implementation details of KalmanState
    std::array<std::unique_ptr<KalmanState>, ADC1_CHANNEL_MAX> m_kalman_states;
    std::array<bool, ADC1_CHANNEL_MAX> m_active_channels;

    static bool IRAM_ATTR
    s_adc_callback(adc_continuous_handle_t handle,
                   const adc_continuous_evt_data_t * edata, void * user_data);
    static void s_adc_task_wrapper(void * param);
    void adc_task();
};

#endif  // HAL_ADC_READER_HPP
