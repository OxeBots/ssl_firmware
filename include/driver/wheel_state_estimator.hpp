/**
 * @file state_estimator.hpp
 * @brief This file contains the definition of the WheelStateEstimator class.
 * @author Erick Suzart
 * @date 2023-10-27
 */
#ifndef DRIVER_WHEEL_STATE_ESTIMATOR_HPP
#define DRIVER_WHEEL_STATE_ESTIMATOR_HPP

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
#include "driver/i2c_master.h"
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

/** @brief Output stage settings for the AS5600 sensor. */
typedef enum
{
    AS5600_OUTPUT_STAGE_ANALOG_FULL = 0x00,
    AS5600_OUTPUT_STAGE_ANALOG_REDUCED = 0x01,
    AS5600_OUTPUT_STAGE_DIGITAL_PWM = 0x02,
} as5600_output_stage_t;

/** @brief Slow filter settings for the AS5600 sensor. */
typedef enum
{
    AS5600_SLOW_FILTER_16X = 0x00,
    AS5600_SLOW_FILTER_8X = 0x01,
    AS5600_SLOW_FILTER_4X = 0x02,
    AS5600_SLOW_FILTER_2X = 0x03
} as5600_slow_filter_t;

/** @brief Fast filter threshold settings for the AS5600 sensor. */
typedef enum
{
    AS5600_FAST_FILTER_THRESH_SLOW_ONLY = 0x00,
    AS5600_FAST_FILTER_THRESH_6LSB = 0x01,
    AS5600_FAST_FILTER_THRESH_7LSB = 0x02,
    AS5600_FAST_FILTER_THRESH_9LSB = 0x03,
    AS5600_FAST_FILTER_THRESH_18LSB = 0x04,
    AS5600_FAST_FILTER_THRESH_21LSB = 0x05,
    AS5600_FAST_FILTER_THRESH_24LSB = 0x06,
    AS5600_FAST_FILTER_THRESH_10LSB = 0x07
} as5600_fast_filter_thresh_t;

/**
 * @class WheelStateEstimator
 * @brief Manages ADC readings from analog encoders and provides
 * Kalman-filtered estimates for angle, velocity (RPM), and acceleration.
 *
 * This class implements a singleton pattern to provide a single point of
 * access for ADC-based state estimation. It uses the ESP-IDF's continuous ADC
 * driver to sample analog encoders, performs oversampling to reduce noise, and
 * then feeds the averaged measurements into an Extended Kalman Filter (EKF)
 * for each channel. The EKF tracks the rotational state (angle, angular
 * velocity, angular acceleration), providing a smoothed and physically
 * consistent estimate that is more robust than raw sensor readings.
 */
class WheelStateEstimator
{
   public:
    /** @brief The number of raw ADC samples to average for each measurement.
     */
    static constexpr size_t OVERSAMPLE_COUNT = 16;
    /** @brief Conversion factor from a 12-bit raw ADC value to radians. */
    static constexpr float RAW_TO_RAD = (2.0f * M_PI) / 4096.0f;
    /** @brief Conversion factor from radians per second to revolutions per
     * minute. */
    static constexpr float RAD_S_TO_RPM = 60.0f / (2.0f * M_PI);

    /**
     * @struct Measurement
     * @brief Holds the result of a completed oversampling window.
     */
    struct Measurement
    {
        uint16_t value = 0;       ///< The averaged raw ADC value.
        int64_t duration_us = 0;  ///< Time taken to collect the samples.
    };

    // --- Singleton Pattern Implementation ---
    WheelStateEstimator(const WheelStateEstimator &) = delete;
    WheelStateEstimator & operator=(const WheelStateEstimator &) = delete;

    /**
     * @brief Get the singleton instance of the WheelStateEstimator.
     * @return A reference to the singleton instance.
     */
    static WheelStateEstimator & get_instance()
    {
        static WheelStateEstimator instance;
        return instance;
    }

    /**
     * @brief Initializes the ADC continuous driver and the Kalman filters for
     * the specified channels.
     * @param channels A vector of ADC channels to read from.
     * @return ESP_OK on success, otherwise an error code.
     */
    esp_err_t init(const std::vector<adc_channel_t> & channels);

    /**
     * @brief Initializes the I2C master driver for sensor communication.
     * @param i2c_port The I2C port number.
     * @param sda_pin The GPIO pin for SDA.
     * @param scl_pin The GPIO pin for SCL.
     * @return ESP_OK on success, otherwise an error code.
     */
    esp_err_t init_i2c(i2c_port_t i2c_port, gpio_num_t sda_pin,
                       gpio_num_t scl_pin);

    // --- Filtered State Getters ---

    /**
     * @brief Get the filtered angle estimate from the Kalman filter.
     * @param channel The ADC channel corresponding to the desired encoder.
     * @return The estimated angle in degrees, normalized from -180 to 180.
     */
    float get_filtered_angle_deg(adc_channel_t channel);

    /**
     * @brief Get the filtered rotational speed estimate from the Kalman
     * filter.
     * @param channel The ADC channel corresponding to the desired encoder.
     * @return The estimated speed in Revolutions Per Minute (RPM).
     */
    float get_filtered_rpm(adc_channel_t channel);

    /**
     * @brief Get the filtered rotational acceleration estimate from the Kalman
     * filter.
     * @param channel The ADC channel corresponding to the desired encoder.
     * @return The estimated acceleration in revolutions per second squared
     * (rev/s^2).
     */
    float get_filtered_acceleration_rps2(adc_channel_t channel);

    // --- Raw Data Getters (for debugging) ---

    /**
     * @brief Get the most recent raw (but averaged) ADC value.
     * @param channel The ADC channel to read from.
     * @return The 12-bit ADC value after oversampling and averaging.
     */
    uint32_t get_value(adc_channel_t channel);

    /**
     * @brief Get the duration of the last oversampling window.
     * @param channel The ADC channel to query.
     * @return The time in microseconds it took to collect the samples.
     */
    int64_t get_measurement_duration_us(adc_channel_t channel);

    esp_err_t setOutputStage(as5600_output_stage_t stage);
    esp_err_t setSlowFilter(as5600_slow_filter_t filter);
    esp_err_t setFastFilter(as5600_fast_filter_thresh_t threshold);

   private:
    /** @brief Private constructor to enforce singleton pattern. */
    WheelStateEstimator();
    /** @brief Private destructor to clean up resources. */
    ~WheelStateEstimator();

    /**
     * @struct KalmanState
     * @brief A private struct to hold the EKF and its state for one channel.
     * @note The full definition is in the .cpp file to hide implementation
     * details.
     */
    struct KalmanState;

    // --- Member Variables ---
    adc_continuous_handle_t m_adc_handle;
    TaskHandle_t m_task_handle;
    bool m_initialized;
    bool m_i2c_initialized;
    static constexpr size_t ADC_BUFFER_SIZE = 512;
    i2c_master_bus_handle_t m_i2c_bus_handle;
    i2c_master_dev_handle_t m_i2c_dev_handle;

    /**
     * @struct SamplingState
     * @brief Internal state for collecting a window of samples for averaging.
     */
    struct SamplingState
    {
        std::array<uint16_t, OVERSAMPLE_COUNT> samples;
        size_t count = 0;
    };

    std::array<SamplingState, ADC1_CHANNEL_MAX> m_sampling_states;
    std::array<Measurement, ADC1_CHANNEL_MAX> m_current_measurements;
    std::array<std::unique_ptr<KalmanState>, ADC1_CHANNEL_MAX> m_kalman_states;
    std::array<bool, ADC1_CHANNEL_MAX> m_active_channels;

    // --- Private Methods ---

    // --- AS5600 Configuration ---
    esp_err_t read_register(uint8_t reg_addr, uint8_t * data, size_t len);
    esp_err_t write_register(uint8_t reg_addr, uint8_t * data, size_t len);
    esp_err_t read_config_register(uint16_t * config);
    esp_err_t write_config_register(uint16_t config);

    /**
     * @brief ISR callback for the ADC continuous driver.
     * @note This function is called from an interrupt context. It only
     * notifies the processing task and returns immediately.
     */
    static bool IRAM_ATTR
    s_adc_callback(adc_continuous_handle_t handle,
                   const adc_continuous_evt_data_t * edata, void * user_data);

    /**
     * @brief A C-style wrapper to launch the C++ member function as a FreeRTOS
     * task.
     */
    static void s_adc_task_wrapper(void * param);

    /**
     * @brief The main FreeRTOS task for processing ADC data.
     * @note This task waits for notifications from the ISR, reads the ADC
     * data, performs oversampling, and runs the Kalman filter update step.
     */
    void adc_task();
};

#endif  // DRIVER_WHEEL_STATE_ESTIMATOR_HPP
