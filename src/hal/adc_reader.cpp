#include "hal/adc_reader.hpp"

ADC_Reader::ADC_Reader()
: m_adc_handle(NULL), m_task_handle(NULL), m_initialized(false)
{
    // Initialization is deferred to the init()
    // method.
}

ADC_Reader::~ADC_Reader()
{
    if (m_initialized)
    {
        adc_continuous_stop(m_adc_handle);
        adc_continuous_deinit(m_adc_handle);
    }

    // Clean up by deleting the FreeRTOS task if it was created.
    if (m_task_handle) vTaskDelete(m_task_handle);
}

esp_err_t ADC_Reader::init(const std::vector<adc_channel_t> & channels)
{
    // Prevent re-initialization.
    if (m_initialized) return ESP_ERR_INVALID_STATE;

    // Initialize the ADC data map with all channels set to a default of 0.
    for (const auto & ch : channels) m_adc_data[ch] = 0;

    // Configure the ADC continuous mode handle.
    adc_continuous_handle_cfg_t handle_config = {
      .max_store_buf_size =
        static_cast<uint32_t>(channels.size() * SOC_ADC_DIGI_RESULT_BYTES * 16),
      .conv_frame_size =
        static_cast<uint32_t>(channels.size() * SOC_ADC_DIGI_RESULT_BYTES),
      .flags = {.flush_pool = true},
    };

    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_config, &m_adc_handle));

    // Due to hardware limitations, only ADC1 can be used reliably within DMA mode
    std::vector<adc_digi_pattern_config_t> pattern_config(channels.size());
    for (size_t i = 0; i < channels.size(); ++i)
    {
        pattern_config[i] = {
          .atten = ADC_ATTEN_DB_12,
          .channel = (uint8_t)channels[i],
          .unit = ADC_UNIT_1,
          .bit_width = ADC_BITWIDTH_12,
        };
    }

    adc_continuous_config_t adc_config = {
      .pattern_num = (uint32_t)channels.size(),
      .adc_pattern = pattern_config.data(),
      .sample_freq_hz = SOC_ADC_SAMPLE_FREQ_THRES_LOW * channels.size(),
      .conv_mode = ADC_CONV_SINGLE_UNIT_1,
      .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    ESP_ERROR_CHECK(adc_continuous_config(m_adc_handle, &adc_config));

    adc_continuous_evt_cbs_t cb_config = {
      .on_conv_done = s_adc_callback,
      .on_pool_ovf = NULL,
    };

    ESP_ERROR_CHECK(
      adc_continuous_register_event_callbacks(m_adc_handle, &cb_config, this));

    xTaskCreate(s_adc_task_wrapper, "Task_ADC_Reader", 4096, this, 5, &m_task_handle);

    m_initialized = true;
    ESP_ERROR_CHECK(adc_continuous_start(m_adc_handle));

    return ESP_OK;
}

bool IRAM_ATTR ADC_Reader::s_adc_callback(
  adc_continuous_handle_t handle, const adc_continuous_evt_data_t * edata,
  void * user_data)
{
    ADC_Reader * driver = static_cast<ADC_Reader *>(user_data);
    BaseType_t mustYield = pdFALSE;
    vTaskNotifyGiveFromISR(driver->m_task_handle, &mustYield);
    return (mustYield == pdTRUE);
}

void ADC_Reader::s_adc_task_wrapper(void * param)
{
    static_cast<ADC_Reader *>(param)->adc_task();
}

void ADC_Reader::adc_task()
{
    uint8_t buf[SOC_ADC_DIGI_RESULT_BYTES * 4];
    uint32_t rxLen = 0;

    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (adc_continuous_read(m_adc_handle, buf, sizeof(buf), &rxLen,
                                   0) == ESP_OK)
        {
            for (uint32_t i = 0; i < rxLen; i += SOC_ADC_DIGI_RESULT_BYTES)
            {
                auto * p = reinterpret_cast<adc_digi_output_data_t *>(&buf[i]);
                m_adc_data[static_cast<adc_channel_t>(p->type1.channel)] =
                  p->type1.data;
            }
        }
    }
}
