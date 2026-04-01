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
#include "NVSManager.h"
#include "helper_func.h"
#include "wheel_ekf.h"

class AS5600
{
   public:
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

    AS5600(adc_channel_t channel, adc_cali_handle_t cali_handle, bool voltage_calibrated, adc_unit_t unit = ADC_UNIT_1,
           adc_bitwidth_t bitwidth = ADC_BITWIDTH_12);
    ~AS5600() = default;

    // I2C methods
    esp_err_t init_i2c();

    esp_err_t set_output_stage(OutputStage stage);
    esp_err_t set_slow_filter(SlowFilter filter);
    esp_err_t set_fast_filter(FastFilter threshold);
    esp_err_t burn_settings();

    esp_err_t read_configuration(OutputStage * stage, SlowFilter * slow, FastFilter * fast);
    esp_err_t get_i2c_raw_angle(uint16_t * angle);

    // Analog methods
    void process_new_reading(uint16_t raw_adc_value);
    esp_err_t calibrate_range(uint32_t duration_ms);
    esp_err_t set_calibration_range(int min_mv = 0, int max_mv = 3300);

    // Calibration
    void reset_calibration_min_max();
    void start_calibration_mode();
    void stop_calibration_mode();

    esp_err_t save_calibration_to_nvs();
    esp_err_t load_calibration_from_nvs();

    // Getters
    float get_angle_rad() const;
    float get_angle_deg() const;
    float get_rpm() const;
    float get_acceleration_rps2() const;
    int get_last_voltage_mv() const;
    uint16_t get_last_raw_value() const;
    int get_calib_min() const;
    int get_calib_max() const;

   private:
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

    const adc_bitwidth_t ADC_BITWIDTH;
    static constexpr int MIN_VALID_VOLTAGE_RANGE_MV = 500;
    static constexpr double RAD_S_TO_RPM = 60.0 / (2.0 * PI);
    static constexpr double RAD_S2_TO_RPS2 = 1.0 / (2.0 * PI);
    static constexpr size_t OVERSAMPLE_COUNT = 16;
    static constexpr const char * NVS_NS = "wheel_calib";

    struct SamplingState
    {
        std::array<uint16_t, OVERSAMPLE_COUNT> samples;
        size_t count = 0;
    };

    adc_channel_t m_channel;
    std::unique_ptr<WheelKalmanFilter> m_filter;

    adc_cali_handle_t m_cali_handle = nullptr;
    bool m_is_voltage_calibrated = false;
    volatile bool m_is_calibrating = false;
    volatile int m_min_voltage_mv = 5000;
    volatile int m_max_voltage_mv = 0;

    SamplingState m_sampling_state;
    uint16_t m_last_avg_value = 0;
};

#endif  // AS5600_H
