/**
 * @file wheel_odometry.hpp
 * @brief High-level manager for multiple AS5600 encoders.
 */
#ifndef KINEMATICS_WHEEL_ODOMETRY_HPP
#define KINEMATICS_WHEEL_ODOMETRY_HPP

#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_continuous.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <array>
#include <map>
#include <memory>
#include <vector>

#include "driver/as5600_analog.hpp"

class WheelOdometry
{
   private:
    static constexpr adc_unit_t ADC_UNIT = ADC_UNIT_1;
    static constexpr uint8_t NUM_ENC_CHANNELS = 4;
    static constexpr size_t ADC_BUFFER_SIZE = 512;
    static constexpr adc_bitwidth_t ADC_BITWIDTH = ADC_BITWIDTH_12;

    struct EncoderChannel
    {
        adc_channel_t channel_num;
        std::unique_ptr<AS5600_analog> encoder;
        adc_cali_handle_t cali_handle;
    };

    WheelOdometry();
    ~WheelOdometry();

    // --- Private Methods ---
    void adc_task();
    static void s_adc_task_wrapper(void * param);
    static bool IRAM_ATTR s_adc_callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t * edata,
                                         void * user_data);

    // --- Member Variables ---
    bool m_initialized = false;
    adc_continuous_handle_t m_adc_handle;
    TaskHandle_t m_task_handle;

    std::vector<EncoderChannel> m_enc_channels;
    std::array<const EncoderChannel *, SOC_ADC_CHANNEL_NUM(ADC_UNIT)> m_channel_lookup;

    SemaphoreHandle_t m_data_mutex;

   public:
    // Singleton access
    static WheelOdometry & get_instance();

    WheelOdometry(const WheelOdometry &) = delete;
    WheelOdometry & operator=(const WheelOdometry &) = delete;

    /**
     * @brief Initializes the underlying ADC hardware and creates AS5600 objects.
     * @param channels Vector of ADC channels to initialize.
     * @return ESP_OK on success.
     */
    esp_err_t init(const std::vector<adc_channel_t> & channels, adc_atten_t attenuation = ADC_ATTEN_DB_12);

    /**
     * @brief Calibrates the operational voltage range for all wheel encoders.
     * @param duration_ms Time for calibration.
     * @return ESP_OK on success.
     */
    esp_err_t calibrate_wheel_encoders(uint32_t duration_ms = 5000);

    // --- Getters for filtered state ---
    std::vector<float> get_filtered_angle_rad();
    std::vector<float> get_filtered_angle_deg();
    std::vector<float> get_filtered_rpm();
    std::vector<float> get_filtered_acceleration_rps2();
};

#endif  // KINEMATICS_WHEEL_ODOMETRY_HPP
