/**
 * @file AdcStream.cpp
 * @brief Shared ADC1 continuous (DMA) acquisition for all analog consumers.
 *
 * One stream owns the ADC1 unit because the oneshot and continuous ADC
 * drivers cannot run on the same unit at the same time. The conversion frame
 * carries one slot per sampled channel (the four wheel encoder channels plus
 * the VMON supply channel). Every channel is oversampled with a block mean of
 * ADC_OVERSAMPLE consecutive conversions; the VMON block mean is line-fitting
 * calibrated and published to the PowerMonitor, the encoder block means are
 * dispatched to the registered handlers, so consumers never read the ADC
 * hardware directly.
 */

#include "AdcStream.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <sdkconfig.h>

#include "PowerMonitor.h"

// DMA buffer pool: large enough to hold several conversion frames between
// task wakeups without overflowing the ring buffer.
static constexpr size_t ADC_STREAM_BUFFER_SIZE = 512;

// A block whose sample spread exceeds half of the 12-bit range straddled a
// discontinuity of a wrapping channel (the AS5600 analog output spans only
// 0.1..0.9 VCC and wraps once per motor revolution). Publishing such a mean
// would report a mid-scale angle, so the block is discarded instead. Real
// signals never move that far within one block: at the 4000 RPM motor limit
// a full encoder block spans about 21 degrees, roughly 100 counts.
static constexpr uint16_t ADC_STREAM_WRAP_GUARD = 4096 / 2;

static const char * TAG = "AdcStream";

AdcStream & AdcStream::get_instance()
{
    static AdcStream instance;
    return instance;
}

AdcStream::AdcStream() = default;

AdcStream::~AdcStream()
{
    if (m_initialized && !m_suspended)
    {
        adc_continuous_stop(m_adc_handle);
    }
    if (m_initialized)
    {
        adc_continuous_deinit(m_adc_handle);
    }
    for (auto & handle : m_cali_handles)
    {
        if (handle != nullptr)
            adc_cali_delete_scheme_line_fitting(handle);
    }
    if (m_task_handle != nullptr)
        vTaskDelete(m_task_handle);
}

/**
 * @brief Creates the shared continuous stream and starts it.
 * @param encoder_channels The four wheel encoder channels to sample.
 * @param attenuation Attenuation applied to every sampled channel.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE when already initialized.
 */
esp_err_t AdcStream::init(
  const std::array<adc_channel_t, ADC_STREAM_CHANNEL_COUNT - 1> & encoder_channels,
  adc_atten_t attenuation)
{
    if (m_initialized)
        return ESP_ERR_INVALID_STATE;

    m_vmon_channel = static_cast<adc_channel_t>(CONFIG_VMON_ADC_CHANNEL);

    // One line-fitting calibration per sampled channel; consumers borrow the
    // handle of their channel through get_cali_handle().
    adc_cali_line_fitting_config_t cali_config = {
      .unit_id = ADC_STREAM_UNIT,
      .atten = attenuation,
      .bitwidth = ADC_BITWIDTH_12,
      .default_vref = ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF};
    for (const adc_channel_t ch : encoder_channels)
    {
        adc_cali_handle_t handle = nullptr;
        if (adc_cali_create_scheme_line_fitting(&cali_config, &handle) == ESP_OK)
        {
            m_cali_handles[ch] = handle;
            ESP_LOGI(TAG, "ADC voltage calibration for channel %d successful.", ch);
        }
        else
        {
            ESP_LOGE(TAG, "ADC voltage calibration failed for channel %d.", ch);
        }
    }

    ESP_LOGI(
      TAG,
      "Scan: %u slots (%u encoders x%u + VMON x%u) at %u Hz: encoder %u Hz per channel, VMON %u "
      "Hz.",
      (unsigned)ADC_STREAM_SCAN_SLOTS,
      (unsigned)(ADC_STREAM_CHANNEL_COUNT - 1),
      (unsigned)ADC_STREAM_ENCODER_WEIGHT,
      (unsigned)ADC_STREAM_VMON_WEIGHT,
      (unsigned)ADC_STREAM_SAMPLE_FREQ_HZ,
      (unsigned)(ADC_STREAM_SAMPLE_FREQ_HZ * ADC_STREAM_ENCODER_WEIGHT / ADC_STREAM_SCAN_SLOTS),
      (unsigned)(ADC_STREAM_SAMPLE_FREQ_HZ * ADC_STREAM_VMON_WEIGHT / ADC_STREAM_SCAN_SLOTS));

    // The VMON calibration is independent of the encoder channels; without it
    // the block mean falls back to an approximate conversion.
    adc_cali_handle_t vmon_handle = nullptr;
    if (adc_cali_create_scheme_line_fitting(&cali_config, &vmon_handle) == ESP_OK)
    {
        m_cali_handles[m_vmon_channel] = vmon_handle;
    }
    else
    {
        ESP_LOGW(TAG, "VMON calibration unavailable, raw counts are used.");
    }

    adc_continuous_handle_cfg_t handle_config = {
      .max_store_buf_size = ADC_STREAM_BUFFER_SIZE * ADC_STREAM_SCAN_SLOTS,
      // Each scan produces one 2-byte sample per channel, but the DMA moves
      // 4-byte words, so the frame size must count those words (which also
      // keeps whole scans inside every conversion frame).
      .conv_frame_size = ADC_STREAM_SCAN_SLOTS * SOC_ADC_DIGI_DATA_BYTES_PER_CONV,
      .flags = {.flush_pool = true},
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_config, &m_adc_handle));

    std::array<adc_digi_pattern_config_t, ADC_STREAM_SCAN_SLOTS> pattern_config;
    size_t pattern_idx = 0;

    // Encoder slots first: each encoder channel repeats its weight, so the
    // round-robin scan gives the encoder channels the larger conversion
    // share while the VMON channel takes one slot per scan round.
    for (size_t rep = 0; rep < ADC_STREAM_ENCODER_WEIGHT; ++rep)
    {
        for (const adc_channel_t ch : encoder_channels)
        {
            adc_digi_pattern_config_t pattern = {
              .atten = attenuation,
              .channel = static_cast<uint8_t>(ch & 0x7),
              .unit = ADC_STREAM_UNIT,
              .bit_width = ADC_BITWIDTH_12,
            };
            pattern_config[pattern_idx++] = pattern;
        }
    }
    for (size_t rep = 0; rep < ADC_STREAM_VMON_WEIGHT; ++rep)
    {
        pattern_config[pattern_idx++] = {
          .atten = attenuation,
          .channel = static_cast<uint8_t>(CONFIG_VMON_ADC_CHANNEL),
          .unit = ADC_STREAM_UNIT,
          .bit_width = ADC_BITWIDTH_12,
        };
    }

    adc_continuous_config_t adc_config = {
      .pattern_num = ADC_STREAM_SCAN_SLOTS,
      .adc_pattern = pattern_config.data(),
      .sample_freq_hz = ADC_STREAM_SAMPLE_FREQ_HZ,
      .conv_mode = ADC_CONV_SINGLE_UNIT_1,
      .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    ESP_ERROR_CHECK(adc_continuous_config(m_adc_handle, &adc_config));

    adc_continuous_evt_cbs_t cb_config = {.on_conv_done = s_conv_done_callback,
                                          .on_pool_ovf = nullptr};
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(m_adc_handle, &cb_config, this));

    // 8 KiB: the consumer handlers (EKF update and logging) run on this
    // task, so leave headroom above their fixed-size working arrays. It is
    // pinned to core 1 so the boot/init task on core 0 cannot be starved when
    // the acquisition rate is high; the wheel control task shares core 1 and
    // has the higher priority.
    if (xTaskCreatePinnedToCore(
          s_task_wrapper, "ADCReaderTask", 8192, this, 5, &m_task_handle, 1) != pdPASS)
    {
        ESP_LOGE(TAG, "ADC dispatch task creation failed.");
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(adc_continuous_start(m_adc_handle));
    m_initialized = true;

    return ESP_OK;
}

/**
 * @brief Pauses the stream and the dispatch task. Partial oversampling
 *        blocks are flushed so a resumed stream publishes fresh means.
 */
void AdcStream::suspend()
{
    if (m_initialized && !m_suspended)
    {
        vTaskSuspend(m_task_handle);
        adc_continuous_stop(m_adc_handle);
        m_sample_sum.fill(0);
        m_sample_count.fill(0);
        m_block_min.fill(0xFFFF);
        m_block_max.fill(0);
        m_suspended = true;
    }
}

/**
 * @brief Resumes the paused stream.
 */
void AdcStream::resume()
{
    if (m_initialized && m_suspended)
    {
        adc_continuous_start(m_adc_handle);
        xTaskNotifyStateClear(m_task_handle);
        vTaskResume(m_task_handle);
        m_suspended = false;
    }
}

/**
 * @brief Registers the published-value handler of one channel.
 */
void AdcStream::set_sample_handler(adc_channel_t channel, SampleHandler handler, void * user_data)
{
    if (static_cast<int>(channel) >= SOC_ADC_CHANNEL_NUM(ADC_STREAM_UNIT))
    {
        ESP_LOGW(TAG, "Sample handler for invalid channel %d ignored.", channel);
        return;
    }
    m_handlers[channel] = handler;
    m_handler_data[channel] = user_data;
}

/**
 * @brief Returns the calibration handle of one sampled channel.
 */
adc_cali_handle_t AdcStream::get_cali_handle(adc_channel_t channel) const
{
    if (static_cast<int>(channel) >= SOC_ADC_CHANNEL_NUM(ADC_STREAM_UNIT))
        return nullptr;
    return m_cali_handles[channel];
}

/**
 * @brief Static wrapper to launch the dispatch task from FreeRTOS.
 */
void AdcStream::s_task_wrapper(void * param)
{
    static_cast<AdcStream *>(param)->task_loop();
}

/**
 * @brief ISR callback for ADC conversion complete events.
 *
 * Executed from IRAM context when the DMA buffer is filled. Notifies the
 * dispatch task to wake up and process the data.
 *
 * @return true if a high-priority task (the dispatch task) was woken up.
 */
bool IRAM_ATTR AdcStream::s_conv_done_callback(adc_continuous_handle_t handle,
                                               const adc_continuous_evt_data_t * edata,
                                               void * user_data)
{
    auto * stream = static_cast<AdcStream *>(user_data);
    BaseType_t must_yield = pdFALSE;
    vTaskNotifyGiveFromISR(stream->m_task_handle, &must_yield);
    return (must_yield == pdTRUE);
}

/**
 * @brief Dispatch task loop.
 *
 * Waits for notifications from the ISR, drains the DMA buffers and feeds
 * every conversion into its channel's oversampling block; a completed block
 * publishes one mean value.
 */
void AdcStream::task_loop()
{
    static uint8_t result_buffer[ADC_STREAM_BUFFER_SIZE * ADC_STREAM_SCAN_SLOTS];

    while (true)
    {
        // Wait indefinitely for a notification from the ADC ISR
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint32_t bytes_read = 0;
        esp_err_t ret;

        do
        {
            ret = adc_continuous_read(
              m_adc_handle, result_buffer, sizeof(result_buffer), &bytes_read, 0);

            if (ret == ESP_OK)
            {
                for (size_t i = 0; i < bytes_read; i += SOC_ADC_DIGI_RESULT_BYTES)
                {
                    auto * p = reinterpret_cast<adc_digi_output_data_t *>(&result_buffer[i]);
                    const adc_channel_t chan_num = static_cast<adc_channel_t>(p->type1.channel);
                    if (static_cast<int>(chan_num) >= SOC_ADC_CHANNEL_NUM(ADC_STREAM_UNIT))
                        continue;
                    dispatch_sample(chan_num, p->type1.data);
                }
            }
        } while (ret == ESP_OK && bytes_read > 0);
    }
}

/**
 * @brief Accumulates one conversion into its channel's oversampling block;
 *        a completed block publishes the mean to the channel's consumer.
 *        Blocks that straddled a wrapping channel discontinuity are
 *        discarded instead of published.
 *
 * The VMON channel mean is line-fitting calibrated and published to the
 * PowerMonitor; every other channel mean goes to the registered handler.
 */
void AdcStream::dispatch_sample(adc_channel_t channel, uint16_t raw)
{
    if (raw < m_block_min[channel])
        m_block_min[channel] = raw;
    if (raw > m_block_max[channel])
        m_block_max[channel] = raw;

    m_sample_sum[channel] += raw;
    if (++m_sample_count[channel] < CONFIG_ADC_OVERSAMPLE)
        return;

    const uint16_t mean_raw = static_cast<uint16_t>(m_sample_sum[channel] / CONFIG_ADC_OVERSAMPLE);
    const uint16_t spread = m_block_max[channel] - m_block_min[channel];
    m_sample_sum[channel] = 0;
    m_sample_count[channel] = 0;
    m_block_min[channel] = 0xFFFF;
    m_block_max[channel] = 0;

    if (spread > ADC_STREAM_WRAP_GUARD)
    {
        // The block straddled a wrap: its mean is meaningless, skip it. The
        // consumer only sees one fewer value at its (already high) rate.
        return;
    }

    if (channel == m_vmon_channel)
    {
        int millivolt = 0;
        if (m_cali_handles[channel] == nullptr ||
            adc_cali_raw_to_voltage(m_cali_handles[channel], mean_raw, &millivolt) != ESP_OK)
            millivolt = mean_raw * 3300 / 4095;
        PowerMonitor::get_instance().update_adc_voltage(
          static_cast<float>(millivolt) / 1000.0f, static_cast<uint64_t>(esp_timer_get_time()));
    }
    else if (m_handlers[channel] != nullptr)
    {
        SampleEvent event = {channel, mean_raw, m_handler_data[channel]};
        m_handlers[channel](&event);
    }
}
