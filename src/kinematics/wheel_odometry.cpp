#include "kinematics/wheel_odometry.hpp"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char * TAG = "WheelOdometry";

// --- Singleton Implementation ---
WheelOdometry & WheelOdometry::get_instance()
{
    static WheelOdometry instance;
    return instance;
}
WheelOdometry::WheelOdometry() : m_initialized(false), m_adc_handle(nullptr), m_task_handle(nullptr)
{
    m_data_mutex = xSemaphoreCreateMutex();
    m_channel_lookup.fill(nullptr);
}

WheelOdometry::~WheelOdometry()
{
    if (m_initialized)
    {
        adc_continuous_stop(m_adc_handle);
        adc_continuous_deinit(m_adc_handle);
    }

    for (auto const & channel_data : m_enc_channels)
        if (channel_data.cali_handle)
            adc_cali_delete_scheme_line_fitting(channel_data.cali_handle);

    if (m_task_handle)
        vTaskDelete(m_task_handle);
}

// --- Public Methods ---

esp_err_t WheelOdometry::init(const std::vector<adc_channel_t> & channels, adc_atten_t attenuation)
{
    if (m_initialized)
        return ESP_ERR_INVALID_STATE;

    m_enc_channels.reserve(channels.size());

    // Create a AS5600_analog object for each specified ADC channel
    for (const auto & ch : channels)
    {
        if (static_cast<int>(ch) >= SOC_ADC_CHANNEL_NUM(ADC_UNIT))
        {
            ESP_LOGW(TAG, "Skipping invalid ADC channel: %d", ch);
            continue;
        }

        adc_cali_line_fitting_config_t cali_config = {
          .unit_id = ADC_UNIT,
          .atten = attenuation,
          .bitwidth = ADC_BITWIDTH,
        };
        adc_cali_handle_t handle = nullptr;
        esp_err_t ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        bool is_calibrated = (ret == ESP_OK);

        if (is_calibrated)
            ESP_LOGI(TAG, "ADC voltage calibration for channel %d successful.", ch);
        else
            ESP_LOGE(TAG, "ADC voltage calibration failed for channel %d with error %d", ch, ret);

        auto encoder = std::make_unique<AS5600_analog>(ch, handle, is_calibrated, ADC_UNIT, ADC_BITWIDTH);
        m_enc_channels.emplace_back(EncoderChannel{ch, std::move(encoder), handle});
        m_channel_lookup[ch] = &m_enc_channels.back();
        ESP_LOGI(TAG, "Created AS5600 for channel %d", ch);
    }

    adc_continuous_handle_cfg_t handle_config = {
      .max_store_buf_size = ADC_BUFFER_SIZE * channels.size(),
      .conv_frame_size = channels.size() * SOC_ADC_DIGI_RESULT_BYTES,
      .flags = {.flush_pool = true},
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_config, &m_adc_handle));

    std::vector<adc_digi_pattern_config_t> pattern_config;

    pattern_config.reserve(channels.size());

    for (const auto & ch : channels)
    {
        adc_digi_pattern_config_t pattern = {
          .atten = attenuation,
          .channel = (uint8_t)(ch & 0x7),
          .unit = ADC_UNIT,
          .bit_width = ADC_BITWIDTH,
        };
        pattern_config.push_back(pattern);
    }

    adc_continuous_config_t adc_config = {
      .pattern_num = (uint32_t)channels.size(),
      .adc_pattern = pattern_config.data(),
      .sample_freq_hz = SOC_ADC_SAMPLE_FREQ_THRES_LOW,
      .conv_mode = ADC_CONV_SINGLE_UNIT_1,
      .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    ESP_ERROR_CHECK(adc_continuous_config(m_adc_handle, &adc_config));

    adc_continuous_evt_cbs_t cb_config = {.on_conv_done = s_adc_callback, .on_pool_ovf = nullptr};
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(m_adc_handle, &cb_config, this));

    xTaskCreate(s_adc_task_wrapper, "ADCReaderTask", 4096, this, 15, &m_task_handle);

    ESP_ERROR_CHECK(adc_continuous_start(m_adc_handle));
    m_initialized = true;

    ESP_LOGI(TAG, "Wheel Odometry initialized with %d channels.", m_enc_channels.size());
    return ESP_OK;
}

esp_err_t WheelOdometry::calibrate_wheel_encoders(uint32_t duration_ms)
{
    // Iterate over the vector of active channels
    for (auto const & channel_data : m_enc_channels)
    {
        if (channel_data.encoder->calibrate_range(duration_ms) != ESP_OK)
        {
            ESP_LOGE(TAG, "Calibration failed for channel: %d", channel_data.channel_num);
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

std::vector<float> WheelOdometry::get_filtered_angle_rad()
{
    std::vector<float> angles;
    angles.reserve(m_enc_channels.size());

    if (xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        for (auto const & channel_data : m_enc_channels) angles.push_back(channel_data.encoder->get_angle_rad());

        xSemaphoreGive(m_data_mutex);
    }
    return angles;
}

std::vector<float> WheelOdometry::get_filtered_angle_deg()
{
    std::vector<float> angles;
    angles.reserve(m_enc_channels.size());

    if (xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        for (auto const & channel_data : m_enc_channels) angles.push_back(channel_data.encoder->get_angle_deg());

        xSemaphoreGive(m_data_mutex);
    }
    return angles;
}

std::vector<float> WheelOdometry::get_filtered_rpm()
{
    std::vector<float> rpms;
    rpms.reserve(m_enc_channels.size());
    if (xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        for (auto const & channel_data : m_enc_channels) rpms.push_back(channel_data.encoder->get_rpm());

        xSemaphoreGive(m_data_mutex);
    }
    return rpms;
}

std::vector<float> WheelOdometry::get_filtered_acceleration_rps2()
{
    std::vector<float> accelerations;
    accelerations.reserve(m_enc_channels.size());
    if (xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        for (auto const & channel_data : m_enc_channels)
            accelerations.push_back(channel_data.encoder->get_acceleration_rps2());

        xSemaphoreGive(m_data_mutex);
    }
    return accelerations;
}

// --- Private Methods ---

void WheelOdometry::s_adc_task_wrapper(void * param)
{
    static_cast<WheelOdometry *>(param)->adc_task();
}

bool IRAM_ATTR WheelOdometry::s_adc_callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t * edata,
                                             void * user_data)
{
    auto * estimator = static_cast<WheelOdometry *>(user_data);
    BaseType_t mustYield = pdFALSE;
    vTaskNotifyGiveFromISR(estimator->m_task_handle, &mustYield);
    return (mustYield == pdTRUE);
}

void WheelOdometry::adc_task()
{
    static uint8_t result_buffer[ADC_BUFFER_SIZE * NUM_ENC_CHANNELS];

    while (true)
    {
        // Wait indefinitely for a notification from the ADC ISR
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint32_t bytes_read = 0;
        esp_err_t ret;

        do
        {
            ret = adc_continuous_read(m_adc_handle, result_buffer, sizeof(result_buffer), &bytes_read, 0);

            if (ret == ESP_OK && xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                for (size_t i = 0; i < bytes_read; i += SOC_ADC_DIGI_RESULT_BYTES)
                {
                    adc_digi_output_data_t * p = (adc_digi_output_data_t *)&result_buffer[i];
                    adc_channel_t chan_num = (adc_channel_t)p->type1.channel;
                    uint16_t data = p->type1.data;

                    if (chan_num < SOC_ADC_CHANNEL_NUM(ADC_UNIT) && m_channel_lookup[chan_num])
                    {
                        m_channel_lookup[chan_num]->encoder->process_new_reading(data);
                    }
                }
                xSemaphoreGive(m_data_mutex);
            }
        } while (ret == ESP_OK && bytes_read > 0);
    }
}
