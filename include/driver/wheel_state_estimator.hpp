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

// AS5600 Register Addresses
#define AS5600_REG_CONF_H 0x07
#define AS5600_REG_RAWANGLE_H 0x0C
#define AS5600_REG_BURN 0xFF
#define AS5600_ADDR 0x36

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

// ADC config
#define ADC_ATTENUATION ADC_ATTEN_DB_0  // 0 - 1100 mV
#define ADC_BITWIDTH ADC_BITWIDTH_12    // 0 - ADC_MAX_VALUE
#define ADC_MAX_VALUE ((2 << ADC_BITWIDTH) - 1)
#define ADC_UNIT ADC_UNIT_1  // ESP32 has two available units
#define ADC_GET_CHANNEL(p_data) ((p_data)->type1.channel)
#define ADC_GET_DATA(p_data) ((p_data)->type1.data)

// --- EKF Implementation Details ---

// EKF types and state definition are fully contained in the .cpp file State vector [angle,
// velocity, acceleration]
static constexpr int STATE_DIM = 3;
static constexpr int MEAS_DIM = 1;     // Measurement vector [angle]
static constexpr int CONTROL_DIM = 1;  // Control vector [dt]

/**
 * @class WheelStateEstimator
 * @brief Manages ADC readings from analog encoders and provides
 * Kalman-filtered estimates for angle, velocity (RPM), and acceleration.
 */
class WheelStateEstimator
{
   public:
    /** @brief The number of raw ADC samples to average for each measurement.
     */
    static constexpr size_t OVERSAMPLE_COUNT = 16;
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
     * @brief Initializes the ADC continuous driver, sets up ESP-IDF ADC
     * calibration for each channel, and starts the Kalman filters.
     * @param channels A vector of ADC channels connected to AS5600 encoders.
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
    esp_err_t init_i2c(i2c_port_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin);

    /**
     * @brief Calibrates the analog operational range for a specific channel.
     * @note During this period, you MUST slowly rotate the sensor through its
     * full 360-degree range to capture the min and max calibrated voltage
     * values.
     * @param channel The ADC channel to calibrate.
     * @param duration_ms The time in milliseconds to perform the calibration.
     * @return ESP_OK on success.
     */
    esp_err_t calibrate_range(adc_channel_t channel, uint32_t duration_ms = 5000);

    /**
     * @brief Calibrates the analog operational range for all active channels.
     * @note Rotates through each active channel, performing the range
     * calibration.
     * @param duration_ms_per_channel The time in milliseconds to calibrate
     * each channel.
     * @return ESP_OK on success, ESP_FAIL if any channel fails.
     */
    esp_err_t calibrate_all_ranges(uint32_t duration_ms_per_channel = 5000);

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

    // --- Raw & Calibrated Data Getters ---

    /**
     * @brief Get the most recent raw (but averaged) ADC value.
     * @param channel The ADC channel to read from.
     * @return The 12-bit ADC value after oversampling and averaging.
     */
    uint32_t get_value(adc_channel_t channel);

    /**
     * @brief Get the calibrated voltage for a channel using ESP-IDF's
     * calibration scheme.
     * @param channel The ADC channel to read from.
     * @return The voltage in millivolts (mV). Returns 0 if channel is not
     * active or not calibrated.
     */
    int get_voltage_mv(adc_channel_t channel);

    /**
     * @brief Get the duration of the last oversampling window.
     * @param channel The ADC channel to query.
     * @return The time in microseconds it took to collect the samples.
     */
    int64_t get_measurement_duration_us(adc_channel_t channel);

    // --- AS5600 Configuration ---
    /**
     * @brief Set the output stage configuration for the AS5600, the options
     * are:
     * - AS5600_OUTPUT_STAGE_ANALOG_FULL
     * - AS5600_OUTPUT_STAGE_ANALOG_REDUCED
     * - AS5600_OUTPUT_STAGE_DIGITAL_PWM
     * @param stage The desired output stage configuration.
     * @return ESP_OK on success, otherwise an error code.
     */
    esp_err_t setOutputStage(as5600_output_stage_t stage);

    /**
     * @brief Set the slow filter configuration for the AS5600 connected, this
     * filter is used to smooth out the output signal after rapid changes.
     * @param filter The desired slow filter configuration.
     * @return ESP_OK on success, otherwise an error code.
     */
    esp_err_t setSlowFilter(as5600_slow_filter_t filter);

    /**
     * @brief Set the fast filter configuration for the AS5600 connected, this
     * filter is used to modify the output stage while in motion.
     * @param threshold The desired fast filter threshold configuration.
     * @return ESP_OK on success, otherwise an error code.
     */
    esp_err_t setFastFilter(as5600_fast_filter_thresh_t threshold);

    /**
     * @brief Permanently burns the current configuration to the AS5600's OTP
     * memory.
     * @warning This is a PERMANENT operation and can only be done a maximum of
     * 3 times. Use with extreme caution.
     * @return ESP_OK on successful burn command.
     */
    esp_err_t burn_settings();

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

    std::array<SamplingState, SOC_ADC_CHANNEL_NUM(ADC_UNIT)> m_sampling_states;
    std::array<Measurement, SOC_ADC_CHANNEL_NUM(ADC_UNIT)> m_current_measurements;
    std::array<std::unique_ptr<KalmanState>, SOC_ADC_CHANNEL_NUM(ADC_UNIT)> m_kalman_states;
    std::array<bool, SOC_ADC_CHANNEL_NUM(ADC_UNIT)> m_active_channels;

    // --- ADC Calibration Data ---
    std::array<adc_cali_handle_t, SOC_ADC_CHANNEL_NUM(ADC_UNIT)> m_cali_handles;
    std::array<bool, SOC_ADC_CHANNEL_NUM(ADC_UNIT)> m_channel_calibrated;
    std::array<int, SOC_ADC_CHANNEL_NUM(ADC_UNIT)> m_min_voltage_mv;
    std::array<int, SOC_ADC_CHANNEL_NUM(ADC_UNIT)> m_max_voltage_mv;

    // --- Private Methods ---
    esp_err_t read_register(uint8_t reg_addr, uint8_t * data, size_t len);
    esp_err_t write_register(uint8_t reg_addr, uint8_t * data, size_t len);
    esp_err_t read_config_register(uint16_t * config);
    esp_err_t write_config_register(uint16_t config);

    static bool IRAM_ATTR s_adc_callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t * edata, void * user_data);
    static void s_adc_task_wrapper(void * param);
    void adc_task();
};

#endif  // DRIVER_WHEEL_STATE_ESTIMATOR_HPP
