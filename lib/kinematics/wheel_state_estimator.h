#ifndef KINEMATICS_WHEEL_STATE_ESTIMATOR_H
#define KINEMATICS_WHEEL_STATE_ESTIMATOR_H

#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_continuous.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <array>
#include <cstdio>
#include <memory>

#include "AS5600.h"

static constexpr adc_unit_t ADC_UNIT = ADC_UNIT_1;
static constexpr uint8_t NUM_ENC_CHANNELS = 4;
static constexpr size_t ADC_BUFFER_SIZE = 512;
static constexpr adc_bitwidth_t ADC_BITWIDTH = ADC_BITWIDTH_12;
static constexpr int MIN_VALID_SWING_MV = 2000;
static constexpr int FULL_RANGE_THRESHOLD_MV = 3300 * 0.9;  // 90% of 3.3V

struct AS5600Settings
{
    AS5600::OutputStage output_stage =
      AS5600::OutputStage::ANALOG_REDUCED;  // 0.1 VCC to 0.9 VCC to be in the ESP ADC linear
                                            // response range
    AS5600::SlowFilter slow_filter =
      AS5600::SlowFilter::FILTER_2X;  // Fast response to slow changes, which is good for wheel
                                      // encoders
    AS5600::FastFilter fast_filter =
      AS5600::FastFilter::THRESH_6LSB;  // Fast filter to catch sudden spikes,
                                        // which can happen with noisy readings
    bool burn_settings = false;         // Dangerous! Keep false by default
};

class WheelStateEstimator
{
   public:
    static WheelStateEstimator & get_instance();
    WheelStateEstimator(const WheelStateEstimator &) = delete;
    WheelStateEstimator & operator=(const WheelStateEstimator &) = delete;

    // Lifecycle
    esp_err_t init(const std::array<adc_channel_t, NUM_ENC_CHANNELS> & channels,
                   adc_atten_t attenuation = ADC_ATTEN_DB_12);

    void suspend();
    void resume();

    // Calibration loading (non-blocking)
    esp_err_t load_calibration();

    // Calibration execution (blocking)
    esp_err_t calibrate_encoders(uint32_t duration_ms = 5000, bool stop_on_full_range = true);

    // Calibration state queries
    bool is_calibrated() const;
    bool is_channel_calibrated(size_t channel) const;

    // I2C configuration (advanced)
    esp_err_t configure_encoder(uint8_t channel_idx,
                                const AS5600Settings & settings = AS5600Settings());

    // Data accessors
    std::array<float, NUM_ENC_CHANNELS> get_filtered_angle_rad();
    std::array<float, NUM_ENC_CHANNELS> get_filtered_angle_deg();
    std::array<float, NUM_ENC_CHANNELS> get_filtered_velocity_rad_s();
    std::array<float, NUM_ENC_CHANNELS> get_filtered_rpm();
    std::array<float, NUM_ENC_CHANNELS> get_filtered_acceleration_rps2();

   private:
    struct EncoderChannel
    {
        std::unique_ptr<AS5600> encoder;
        adc_cali_handle_t cali_handle;
    };

    WheelStateEstimator();
    ~WheelStateEstimator();

    // Task and ADC handling
    void adc_task();
    static void s_adc_task_wrapper(void * param);
    static bool IRAM_ATTR s_adc_callback(adc_continuous_handle_t handle,
                                         const adc_continuous_evt_data_t * edata,
                                         void * user_data);

    // Config helpers
    esp_err_t apply_i2c_settings(AS5600 * enc, const AS5600Settings & settings);
    esp_err_t verify_i2c_settings(AS5600 * enc, const AS5600Settings & settings);
    esp_err_t process_otp_burn(AS5600 * enc, const AS5600Settings & settings);
    bool all_channels_full_range() const;

    // State variables
    bool m_initialized = false;
    bool m_is_suspended = false;
    bool m_calibrated = false;
    adc_continuous_handle_t m_adc_handle;
    TaskHandle_t m_task_handle;

    std::array<EncoderChannel, NUM_ENC_CHANNELS> m_enc_channels;
    std::array<const EncoderChannel *, SOC_ADC_CHANNEL_NUM(ADC_UNIT)> m_channel_lookup;
    SemaphoreHandle_t m_data_mutex;
};

#endif  // KINEMATICS_WHEEL_STATE_ESTIMATOR_H
