#include "wheel_state_estimator.h"

static const char * TAG = "WheelStateEstimator";

// --- Singleton Implementation ---
WheelStateEstimator & WheelStateEstimator::get_instance()
{
    static WheelStateEstimator instance;
    return instance;
}
WheelStateEstimator::WheelStateEstimator() : m_initialized(false), m_adc_handle(nullptr), m_task_handle(nullptr)
{
    m_data_mutex = xSemaphoreCreateMutex();
    m_channel_lookup.fill(nullptr);
}

WheelStateEstimator::~WheelStateEstimator()
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

esp_err_t WheelStateEstimator::init(const std::array<adc_channel_t, NUM_ENC_CHANNELS> & channels,
                                    adc_atten_t attenuation)
{
    if (m_initialized)
        return ESP_ERR_INVALID_STATE;

    // Create a AS5600 object for each specified ADC channel
    for (size_t i = 0; i < NUM_ENC_CHANNELS; i++)
    {
        const adc_channel_t ch = channels[i];
        if (static_cast<int>(ch) >= SOC_ADC_CHANNEL_NUM(ADC_UNIT))
        {
            ESP_LOGW(TAG, "Skipping invalid ADC channel: %d", ch);
            continue;
        }

        adc_cali_line_fitting_config_t cali_config = {.unit_id = ADC_UNIT,
                                                      .atten = attenuation,
                                                      .bitwidth = ADC_BITWIDTH,
                                                      .default_vref = ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF};
        adc_cali_handle_t handle = nullptr;
        esp_err_t ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        bool is_calibrated = (ret == ESP_OK);

        if (is_calibrated)
            ESP_LOGI(TAG, "ADC voltage calibration for channel %d successful.", ch);
        else
            ESP_LOGE(TAG, "ADC voltage calibration failed for channel %d with error %d", ch, ret);

        auto encoder = std::make_unique<AS5600>(ch, handle, is_calibrated, ADC_UNIT, ADC_BITWIDTH);
        m_enc_channels[i] = EncoderChannel{std::move(encoder), handle};
        m_channel_lookup[ch] = &m_enc_channels[i];
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
          .channel = static_cast<uint8_t>(ch & 0x7),
          .unit = ADC_UNIT,
          .bit_width = ADC_BITWIDTH,
        };
        pattern_config.push_back(pattern);
    }

    adc_continuous_config_t adc_config = {
      .pattern_num = static_cast<uint32_t>(channels.size()),
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

    ESP_LOGI(TAG, "Wheel Odometry initialized using ADC channels:");
    for (size_t i = 0; i < SOC_ADC_CHANNEL_NUM(ADC_UNIT); i++)
        if (m_channel_lookup[i])
            ESP_LOGI(TAG, "  - Channel %d", i);

    return ESP_OK;
}

void WheelStateEstimator::suspend()
{
    if (m_initialized)
    {
        adc_continuous_stop(m_adc_handle);
        vTaskSuspend(m_task_handle);
    }
}

void WheelStateEstimator::resume()
{
    if (m_initialized)
    {
        vTaskResume(m_task_handle);
        adc_continuous_start(m_adc_handle);
    }
}

esp_err_t WheelStateEstimator::load_or_calibrate(uint32_t duration_ms)
{
    bool missing_calibration = false;
    suspend();

    // Check NVS for all channels
    for (size_t i = 0; i < NUM_ENC_CHANNELS; i++)
    {
        int min_v = 0, max_v = 0;
        esp_err_t err = load_calibration_from_nvs(i, &min_v, &max_v);

        if (err == ESP_OK)
        {
            auto * enc = m_enc_channels[i].encoder.get();
            enc->set_calibration_range(min_v, max_v);
        }
        else
        {
            ESP_LOGW(TAG, "Missing calibration for Ch %d", i);
            missing_calibration = true;
        }
    }

    resume();

    if (missing_calibration)
    {
        ESP_LOGW(TAG, "Entering Calibration Mode. Please rotate all wheels fully!");
        esp_err_t ret = force_calibration(duration_ms, true);

        if (ret != ESP_OK)
            return ret;
    }
    else
        ESP_LOGI(TAG, "Calibration loaded from NVS.");

    return ESP_OK;
}

esp_err_t WheelStateEstimator::force_calibration(uint32_t duration_ms, bool stop_on_stable)
{
    for (auto & ch : m_enc_channels)
    {
        // Stop first to ensure ADC task isn't writing
        ch.encoder->stop_calibration_mode();
        ch.encoder->reset_calibration_min_max();
        ch.encoder->start_calibration_mode();
    }

    ESP_LOGI(TAG, "Calibration started. Duration: %lu ms", duration_ms);

    TickType_t end_tick = xTaskGetTickCount() + pdMS_TO_TICKS(duration_ms);
    bool all_done = false;

    while (xTaskGetTickCount() < end_tick)
    {
        int stable_count = 0;

        for (size_t i = 0; i < NUM_ENC_CHANNELS; i++)
        {
            auto * enc = m_enc_channels[i].encoder.get();
            int range = enc->get_calib_max() - enc->get_calib_min();

            if (range >= FULL_RANGE_THRESHOLD_MV)
                stable_count++;
        }

        if (stop_on_stable && stable_count == NUM_ENC_CHANNELS)
        {
            ESP_LOGI(TAG, "All sensors reached full range. Stopping early.");
            all_done = true;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (!all_done)
        ESP_LOGW(TAG, "Calibration timeout reached.");

    // Stop and Save
    suspend();
    int success_count = 0;

    for (size_t i = 0; i < NUM_ENC_CHANNELS; i++)
    {
        auto * enc = m_enc_channels[i].encoder.get();
        enc->stop_calibration_mode();  // Stop updates

        int range = enc->get_calib_max() - enc->get_calib_min();

        // Validate against MIN_VALID_SWING_MV to catch cases where user didn't rotate enough
        if (range < MIN_VALID_SWING_MV)
            ESP_LOGE(TAG, "Ch %d Failed: Range %d mV is too small (Min Valid: %d)", i, range, MIN_VALID_SWING_MV);
        else
        {
            save_calibration_to_nvs(i, enc->get_calib_min(), enc->get_calib_max());
            success_count++;
        }
    }
    resume();

    return (success_count == NUM_ENC_CHANNELS) ? ESP_OK : ESP_FAIL;
}

esp_err_t WheelStateEstimator::configure_encoder_i2c(uint8_t channel_idx, const AS5600Settings & settings)
{
    if (channel_idx >= NUM_ENC_CHANNELS)
        return ESP_ERR_INVALID_ARG;

    ESP_LOGW(TAG, "Configuring Ch %d. Ensure ONLY this sensor is connected to I2C!", channel_idx);
    suspend();  // Stop ADC Task

    esp_err_t ret = ESP_FAIL;
    auto * enc = m_enc_channels[channel_idx].encoder.get();

    if (enc && enc->init_i2c() == ESP_OK)
    {
        bool success = true;
        // Apply settings
        if (enc->set_output_stage(settings.output_stage) != ESP_OK)
            success = false;
        if (enc->set_slow_filter(settings.slow_filter) != ESP_OK)
            success = false;
        if (enc->set_fast_filter(settings.fast_filter) != ESP_OK)
            success = false;

        // VERIFICATION STEP
        if (success && settings.verify_write)
        {
            // This requires adding get_config_register to AS5600 or similar
            // For now assuming success if ACKs were received.
        }

        if (success)
        {
            ESP_LOGI(TAG, "I2C Config Success.");
            if (settings.burn_settings)
            {
                ESP_LOGW(TAG, "Burning OTP...");
                ret = enc->burn_settings();
            }
            else
                ret = ESP_OK;
        }
    }
    else
        ESP_LOGE(TAG, "Sensor not found on I2C.");

    resume();  // Resume ADC Task
    return ret;
}

std::array<float, NUM_ENC_CHANNELS> WheelStateEstimator::get_filtered_angle_rad()
{
    std::array<float, NUM_ENC_CHANNELS> angles;

    if (xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        for (size_t i = 0; i < NUM_ENC_CHANNELS; ++i) angles[i] = m_enc_channels[i].encoder->get_angle_rad();

        xSemaphoreGive(m_data_mutex);
    }
    return angles;
}

std::array<float, NUM_ENC_CHANNELS> WheelStateEstimator::get_filtered_angle_deg()
{
    std::array<float, NUM_ENC_CHANNELS> angles;

    if (xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        for (size_t i = 0; i < NUM_ENC_CHANNELS; ++i) angles[i] = m_enc_channels[i].encoder->get_angle_deg();

        xSemaphoreGive(m_data_mutex);
    }
    return angles;
}

std::array<float, NUM_ENC_CHANNELS> WheelStateEstimator::get_filtered_rpm()
{
    std::array<float, NUM_ENC_CHANNELS> rpms;
    if (xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        for (size_t i = 0; i < NUM_ENC_CHANNELS; ++i) rpms[i] = m_enc_channels[i].encoder->get_rpm();

        xSemaphoreGive(m_data_mutex);
    }
    return rpms;
}

std::array<float, NUM_ENC_CHANNELS> WheelStateEstimator::get_filtered_acceleration_rps2()
{
    std::array<float, NUM_ENC_CHANNELS> accelerations;
    if (xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        for (size_t i = 0; i < NUM_ENC_CHANNELS; ++i)
            accelerations[i] = m_enc_channels[i].encoder->get_acceleration_rps2();

        xSemaphoreGive(m_data_mutex);
    }
    return accelerations;
}

// --- Private Methods ---

void WheelStateEstimator::s_adc_task_wrapper(void * param)
{
    static_cast<WheelStateEstimator *>(param)->adc_task();
}

bool IRAM_ATTR WheelStateEstimator::s_adc_callback(adc_continuous_handle_t handle,
                                                   const adc_continuous_evt_data_t * edata, void * user_data)
{
    auto * estimator = static_cast<WheelStateEstimator *>(user_data);
    BaseType_t mustYield = pdFALSE;
    vTaskNotifyGiveFromISR(estimator->m_task_handle, &mustYield);
    return (mustYield == pdTRUE);
}

void WheelStateEstimator::adc_task()
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
                    auto * p = reinterpret_cast<adc_digi_output_data_t *>(&result_buffer[i]);
                    auto chan_num = static_cast<adc_channel_t>(p->type1.channel);
                    uint16_t data = p->type1.data;

                    if (chan_num < SOC_ADC_CHANNEL_NUM(ADC_UNIT) && m_channel_lookup[chan_num])
                        m_channel_lookup[chan_num]->encoder->process_new_reading(data);
                }
                xSemaphoreGive(m_data_mutex);
            }
        } while (ret == ESP_OK && bytes_read > 0);
    }
}

esp_err_t WheelStateEstimator::save_calibration_to_nvs(size_t channel_idx, int min_mv, int max_mv)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    if (nvs_open(nvs_namespace, NVS_READWRITE, &my_handle) != ESP_OK)
        return ESP_FAIL;

    // Create unique keys for this channel (Keys must be < 15 chars)
    char key_min[15], key_max[15];
    snprintf(key_min, sizeof(key_min), "ch%zu_min", channel_idx);
    snprintf(key_max, sizeof(key_max), "ch%zu_max", channel_idx);

    // Write
    err = nvs_set_i32(my_handle, key_min, min_mv);
    if (err != ESP_OK)
        goto exit;

    err = nvs_set_i32(my_handle, key_max, max_mv);
    if (err != ESP_OK)
        goto exit;

    // Commit
    err = nvs_commit(my_handle);
    if (err != ESP_OK)
        goto exit;

    ESP_LOGI(TAG, "Saved calibration for Ch %zu: [%d, %d] mV", channel_idx, min_mv, max_mv);

exit:
    // Close
    nvs_close(my_handle);
    return err;
}

esp_err_t WheelStateEstimator::load_calibration_from_nvs(size_t channel_idx, int * min_mv, int * max_mv)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    if (nvs_open(nvs_namespace, NVS_READONLY, &my_handle) != ESP_OK)
        return ESP_FAIL;

    // Generate keys
    char key_min[15];
    char key_max[15];
    snprintf(key_min, sizeof(key_min), "ch%zu_min", channel_idx);
    snprintf(key_max, sizeof(key_max), "ch%zu_max", channel_idx);

    // Read
    int32_t val_min = 0;
    int32_t val_max = 0;

    err = nvs_get_i32(my_handle, key_min, &val_min);
    if (err == ESP_OK)
        err = nvs_get_i32(my_handle, key_max, &val_max);

    nvs_close(my_handle);

    if (err == ESP_OK)
    {
        *min_mv = (int)val_min;
        *max_mv = (int)val_max;
        ESP_LOGI(TAG, "Loaded calibration for Ch %zu: [%d, %d] mV", channel_idx, *min_mv, *max_mv);
    }
    else
        ESP_LOGW(TAG, "Calibration data not found for Ch %zu", channel_idx);

    return err;
}
