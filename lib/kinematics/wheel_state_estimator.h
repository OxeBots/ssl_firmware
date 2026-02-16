/**
 * @file wheel_state_estimator.h
 * @brief High-level manager for multiple AS5600 encoders, handling ADC reading, EKF filtering, and calibration.
 */
#ifndef KINEMATICS_WHEEL_STATE_ESTIMATOR_H
#define KINEMATICS_WHEEL_STATE_ESTIMATOR_H

#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_continuous.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <nvs.h>
#include <nvs_flash.h>

#include <array>
#include <cstdio>
#include <map>
#include <memory>
#include <vector>

#include "AS5600.h"

static constexpr adc_unit_t ADC_UNIT = ADC_UNIT_1;
static constexpr uint8_t NUM_ENC_CHANNELS = 4;
static constexpr size_t ADC_BUFFER_SIZE = 512;
static constexpr adc_bitwidth_t ADC_BITWIDTH = ADC_BITWIDTH_12;
static constexpr int MIN_VALID_SWING_MV = 2000;
static constexpr int FULL_RANGE_THRESHOLD_MV = 3300 * 0.9;  // 90% of 3.3V

struct CalibrationData
{
    int min_voltage_mv;
    int max_voltage_mv;
    bool valid;
};

struct AS5600Settings
{
    AS5600::OutputStage output_stage = AS5600::OutputStage::ANALOG_FULL;  // 0-3.3V
    AS5600::SlowFilter slow_filter = AS5600::SlowFilter::FILTER_2X;
    AS5600::FastFilter fast_filter = AS5600::FastFilter::THRESH_6LSB;
    bool burn_settings = false;  // Dangerous! Keep false by default
    bool verify_write = true;    // NEW: Verify after write
};

class WheelStateEstimator
{
   private:
    struct EncoderChannel
    {
        std::unique_ptr<AS5600> encoder;
        adc_cali_handle_t cali_handle;
    };

    WheelStateEstimator();
    ~WheelStateEstimator();

    // --- Private Methods ---
    void adc_task();
    static void s_adc_task_wrapper(void * param);
    static bool IRAM_ATTR s_adc_callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t * edata,
                                         void * user_data);

    esp_err_t save_calibration_to_nvs(size_t channel_idx, int min, int max);
    esp_err_t load_calibration_from_nvs(size_t channel_idx, int * min, int * max);

    // --- Member Variables ---
    bool m_initialized = false;
    adc_continuous_handle_t m_adc_handle;
    TaskHandle_t m_task_handle;

    std::array<EncoderChannel, NUM_ENC_CHANNELS> m_enc_channels;
    std::array<const EncoderChannel *, SOC_ADC_CHANNEL_NUM(ADC_UNIT)> m_channel_lookup;
    SemaphoreHandle_t m_data_mutex;

    const char * nvs_namespace = "wheel_calib";

   public:
    // Singleton access
    static WheelStateEstimator & get_instance();

    WheelStateEstimator(const WheelStateEstimator &) = delete;
    WheelStateEstimator & operator=(const WheelStateEstimator &) = delete;

    /**
     * @brief Initializes the underlying ADC hardware and creates AS5600 objects.
     * @param channels Vector of ADC channels to initialize.
     * @return ESP_OK on success.
     */
    esp_err_t init(const std::array<adc_channel_t, NUM_ENC_CHANNELS> & channels,
                   adc_atten_t attenuation = ADC_ATTEN_DB_12);

    /**
     * @brief Pauses the ADC task to allow for maintenance/I2C operations.
     */
    void suspend();

    /**
     * @brief Resumes the ADC task.
     */
    void resume();

    /**
     * @brief Loads calibration from NVS. If missing, runs the interactive
     * calibration routine which blocks for 'duration_ms'.
     */
    esp_err_t load_or_calibrate(uint32_t duration_ms = 5000);

    /**
     * @brief Performs range calibration (Min/Max Voltage) and saves to NVS. Blocks for 'duration_ms' or until all
     * sensors hit FULL_RANGE_THRESHOLD_MV.
     * @param stop_on_stable If true, stops early if full range (0-3.3V) is detected.
     */
    esp_err_t force_calibration(uint32_t duration_ms, bool stop_on_stable = true);

    /**
     * @brief Configures the I2C registers for a SPECIFIC sensor channel.
     * @warning Stops the ADC task during execution.
     * @warning YOU MUST ENSURE ONLY ONE SENSOR IS CONNECTED TO I2C BUS (Addr 0x36).
     * @param channel_idx The index (0-3) of the wheel to configure.
     * @param settings The configuration settings to apply.
     * @return ESP_OK on success, ESP_ERR_TIMEOUT if sensor not found.
     */
    esp_err_t configure_encoder_i2c(uint8_t channel_idx, const AS5600Settings & settings = AS5600Settings());

    // --- Getters for filtered state ---
    std::array<float, NUM_ENC_CHANNELS> get_filtered_angle_rad();
    std::array<float, NUM_ENC_CHANNELS> get_filtered_angle_deg();
    std::array<float, NUM_ENC_CHANNELS> get_filtered_rpm();
    std::array<float, NUM_ENC_CHANNELS> get_filtered_acceleration_rps2();
};

#endif  // KINEMATICS_WHEEL_STATE_ESTIMATOR_H
