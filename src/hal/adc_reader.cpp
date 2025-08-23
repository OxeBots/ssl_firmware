#include "hal/adc_reader.hpp"

#define ADC_GET_CHANNEL(p_data) ((p_data)->type1.channel)
#define ADC_GET_DATA(p_data) ((p_data)->type1.data)

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
        static_cast<uint32_t>(channels.size() * SOC_ADC_DIGI_RESULT_BYTES * 4),
      .conv_frame_size =
        static_cast<uint32_t>(channels.size() * SOC_ADC_DIGI_RESULT_BYTES),
      .flags = {.flush_pool = true},
    };

    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_config, &m_adc_handle));

    // Due to hardware limitations, only ADC1 can be used reliably within DMA
    // mode
    std::vector<adc_digi_pattern_config_t> pattern_config(channels.size());
    for (size_t i = 0; i < channels.size(); ++i)
    {
        pattern_config[i] = {
          .atten = ADC_ATTEN_DB_12,
          .channel = (uint8_t)(channels[i] & 0x7),
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

    xTaskCreate(s_adc_task_wrapper, "Task_ADC_Reader", 4096, this, 15,
                &m_task_handle);

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
    // Use larger buffer to handle higher data rates
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t buf[ADC_BUFFER_SIZE] = {0};
    memset(buf, 0xcc, ADC_BUFFER_SIZE);

    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Process all available data in the buffer
        do
        {
            ret =
              adc_continuous_read(m_adc_handle, buf, sizeof(buf), &ret_num, 0);

            if (ret == ESP_OK)
            {
                for (uint32_t i = 0; i < ret_num;
                     i += SOC_ADC_DIGI_RESULT_BYTES)
                {
                    adc_digi_output_data_t * p =
                      (adc_digi_output_data_t *)&buf[i];

                    uint32_t chan_num = ADC_GET_CHANNEL(p);
                    uint32_t data = ADC_GET_DATA(p);

                    if (chan_num < SOC_ADC_CHANNEL_NUM(ADC_UNIT_1))
                        m_adc_data[(adc_channel_t)chan_num] = data;
                }
            }
        } while (ret == ESP_OK);
    }
}
