/**
 * @file AS5600.h
 * @brief Driver for the AS5600 magnetic rotary encoder, supporting both I2C configuration and Analog reading.
 */
#ifndef AS5600_H
#define AS5600_H

#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <standard_constants.h>
#include <stdint.h>

#include <algorithm>
#include <array>
#include <memory>
#include <numeric>

#include "I2Cdev.h"
#include "helper_func.h"
#include "wheel_ekf.h"

class AS5600
{
   public:
    // --- I2C Enums ---
    enum class OutputStage : uint8_t
    {
        ANALOG_FULL = 0x00,
        ANALOG_REDUCED = 0x01,
        DIGITAL_PWM = 0x02,
    };
    enum class SlowFilter : uint8_t
    {
        FILTER_16X = 0x00,
        FILTER_8X = 0x01,
        FILTER_4X = 0x02,
        FILTER_2X = 0x03
    };
    enum class FastFilter : uint8_t
    {
        THRESH_SLOW_ONLY = 0x00,
        THRESH_6LSB = 0x01,
        THRESH_7LSB = 0x02,
        THRESH_9LSB = 0x03,
        THRESH_18LSB = 0x04,
        THRESH_21LSB = 0x05,
        THRESH_24LSB = 0x06,
        THRESH_10LSB = 0x07
    };

    static constexpr uint8_t AS5600_ADDR = 0x36;

    /**
     * @brief Constructor for AS5600.
     * @param channel ADC channel for analog reading.
     * @param cali_handle ADC calibration handle.
     * @param voltage_calibrated Whether the ADC is voltage-calibrated.
     * @param unit ADC unit (default ADC_UNIT_1).
     * @param bitwidth ADC bitwidth (default ADC_BITWIDTH_12).
     */
    AS5600(adc_channel_t channel, adc_cali_handle_t cali_handle, bool voltage_calibrated, adc_unit_t unit = ADC_UNIT_1,
           adc_bitwidth_t bitwidth = ADC_BITWIDTH_12);
    ~AS5600() = default;

    /**
     * @brief Initializes the AS5600 device via I2C.
     * @return ESP_OK on success
     */
    esp_err_t init_i2c();

    // --- I2C Configuration Methods ---
    esp_err_t set_output_stage(OutputStage stage);
    esp_err_t set_slow_filter(SlowFilter filter);
    esp_err_t set_fast_filter(FastFilter threshold);
    esp_err_t burn_settings();

    /**
     * @brief Reads the current configuration from the sensor.
     * @param stage [out] Current output stage setting.
     * @param slow [out] Current slow filter setting.
     * @param fast [out] Current fast filter setting.
     * @return ESP_OK on success.
     */
    esp_err_t read_configuration(OutputStage * stage, SlowFilter * slow, FastFilter * fast);

    /**
     * @brief Get the raw angle from the sensor via I2C.
     * @param angle Pointer to store the 12-bit raw angle (0-4095).
     * @return ESP_OK on success.
     */
    esp_err_t get_i2c_raw_angle(uint16_t * angle);

    // --- Analog Reading Methods ---
    /**
     * @brief Processes a new raw ADC value from the hardware.
     * This method handles oversampling and feeding the result to the EKF.
     * @param raw_adc_value The new 12-bit ADC reading.
     */
    void process_new_reading(uint16_t raw_adc_value);

    /**
     * @brief Calibrates the min/max voltage range for angle conversion.
     * @param duration_ms The duration for the calibration process.
     * @return ESP_OK on success.
     */
    esp_err_t calibrate_range(uint32_t duration_ms);

    /**
     * @brief Sets the calibration range for voltage-to-angle conversion.
     * @param min_mv Minimum voltage in millivolts corresponding to 0 degrees.
     * @param max_mv Maximum voltage in millivolts corresponding to 360 degrees.
     */
    esp_err_t set_calibration_range(int min_mv = 0, int max_mv = 3300);

    /**
     * @brief Resets the internal min/max trackers to their inverse extremes.
     * Call this before starting a new calibration motion.
     */
    void reset_calibration_min_max();

    /**
     * @brief Enables the "learning" mode where new ADC readings expand the min/max range.
     */
    void start_calibration_mode();

    /**
     * @brief Disables the learning mode.
     */
    void stop_calibration_mode();

    // --- Getters ---
    float get_angle_rad() const;
    float get_angle_deg() const;
    float get_rpm() const;
    float get_acceleration_rps2() const;
    int get_last_voltage_mv() const;
    uint16_t get_last_raw_value() const;
    int get_calib_min() const;
    int get_calib_max() const;

   private:
    // I2C Registers and Commands
    enum class Register : uint8_t
    {
        CONF_H = 0x07,
        RAW_ANGLE_H = 0x0C,
        BURN = 0xFF
    };

    enum class Cmd : uint8_t
    {
        BURN = 0x40
    };

    bool m_i2c_initialized = false;

    esp_err_t read_config_register(uint16_t * config);
    esp_err_t write_config_register(uint16_t config);

    // Analog Reading Members
    const adc_bitwidth_t ADC_BITWIDTH;
    static constexpr int MIN_VALID_VOLTAGE_RANGE_MV = 500;
    static constexpr double RAD_S_TO_RPM = 60.0 / (2.0 * PI);
    static constexpr double RAD_S2_TO_RPS2 = 1.0 / (2.0 * PI);
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
    volatile bool m_is_calibrating = false;
    volatile int m_min_voltage_mv = 5000;
    volatile int m_max_voltage_mv = 0;

    SamplingState m_sampling_state;
    uint16_t m_last_avg_value = 0;
};

#endif  // AS5600_H
