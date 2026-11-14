#ifndef _ADC_STREAM_H_
#define _ADC_STREAM_H_

#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_continuous.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <array>

static constexpr adc_unit_t ADC_STREAM_UNIT = ADC_UNIT_1;
// Sampled channels: the four wheel encoder channels plus the VMON supply
// channel, all scanned together in every conversion round.
static constexpr size_t ADC_STREAM_CHANNEL_COUNT = 5;
// Scan-table weights: each encoder channel occupies ENCODER slots of the
// conversion table per VMON slot. The supply channel moves slowly, so it
// takes the smallest share of the conversion budget while the encoder
// channels get the faster, more frequent conversions.
static constexpr size_t ADC_STREAM_ENCODER_WEIGHT = 3;
static constexpr size_t ADC_STREAM_VMON_WEIGHT = 1;
// Conversion-frame slots: every slot converts once per scan round.
static constexpr size_t ADC_STREAM_SCAN_SLOTS =
  (ADC_STREAM_CHANNEL_COUNT - 1) * ADC_STREAM_ENCODER_WEIGHT + ADC_STREAM_VMON_WEIGHT;
// Aggregate conversion rate (Kconfig: ADC_SAMPLE_FREQ_HZ). The scan table
// weights split this budget between the encoder channels and the VMON
// channel; the dispatch task cost scales with it.
static constexpr uint32_t ADC_STREAM_SAMPLE_FREQ_HZ = CONFIG_ADC_SAMPLE_FREQ_HZ;
static_assert(ADC_STREAM_SCAN_SLOTS <= SOC_ADC_PATT_LEN_MAX,
              "scan table exceeds the ADC pattern length limit");

// Owns the ADC1 continuous (DMA) stream shared by every analog consumer. The
// oneshot and continuous ADC drivers cannot run on the same unit at the same
// time, so all analog sampling goes through this one stream: the scan table
// repeats the encoder channels ADC_STREAM_ENCODER_WEIGHT times per VMON slot,
// every channel is oversampled with a block mean of ADC_OVERSAMPLE
// conversions, and the result is dispatched to the consumer registered for
// that channel. The VMON channel is calibrated here and published to the
// PowerMonitor; the other channels deliver raw block means.
class AdcStream
{
   public:
    // One published value of a registered channel (the block mean of
    // ADC_OVERSAMPLE conversions), run in the acquisition task.
    struct SampleEvent
    {
        adc_channel_t channel;
        uint16_t raw;
        void * user_data;
    };
    // FreeRTOS-shaped handler: one opaque parameter, a SampleEvent.
    using SampleHandler = void (*)(void * pvParameters);

    static AdcStream & get_instance();
    AdcStream(const AdcStream &) = delete;
    AdcStream & operator=(const AdcStream &) = delete;

    // Samples the four encoder channels plus the VMON supply channel, which
    // the stream appends from its own configuration.
    esp_err_t init(const std::array<adc_channel_t, ADC_STREAM_CHANNEL_COUNT - 1> & encoder_channels,
                   adc_atten_t attenuation);

    // Pause the stream and the dispatch task (flash/I2C critical sections).
    // Must be called from the task that called init(): the ADC continuous
    // driver acquires a hardware lock in the starting task and
    // adc_continuous_stop() releases it, which FreeRTOS rejects from any
    // other task.
    void suspend();
    // Resume the paused stream (same task-ownership rule as suspend()).
    void resume();

    // Route one channel's published values to a handler (task context).
    void set_sample_handler(adc_channel_t channel, SampleHandler handler, void * user_data);

    // Line-fitting calibration handle of a sampled channel (null if the
    // calibration scheme is unavailable and raw counts are delivered).
    adc_cali_handle_t get_cali_handle(adc_channel_t channel) const;

   private:
    AdcStream();
    ~AdcStream();

    void task_loop();
    void dispatch_sample(adc_channel_t channel, uint16_t raw);
    static void s_task_wrapper(void * param);
    static bool IRAM_ATTR s_conv_done_callback(adc_continuous_handle_t handle,
                                               const adc_continuous_evt_data_t * edata,
                                               void * user_data);

    bool m_initialized = false;
    bool m_suspended = false;
    adc_continuous_handle_t m_adc_handle = nullptr;
    TaskHandle_t m_task_handle = nullptr;

    std::array<SampleHandler, SOC_ADC_CHANNEL_NUM(ADC_STREAM_UNIT)> m_handlers = {};
    std::array<void *, SOC_ADC_CHANNEL_NUM(ADC_STREAM_UNIT)> m_handler_data = {};
    std::array<adc_cali_handle_t, SOC_ADC_CHANNEL_NUM(ADC_STREAM_UNIT)> m_cali_handles = {};
    // Per-channel oversampling accumulators: the block mean of
    // ADC_OVERSAMPLE consecutive conversions is what gets dispatched. The
    // min/max spread of each block detects discontinuities of wrapping
    // channels (blocks that straddled a wrap are discarded, not published).
    std::array<uint32_t, SOC_ADC_CHANNEL_NUM(ADC_STREAM_UNIT)> m_sample_sum = {};
    std::array<uint16_t, SOC_ADC_CHANNEL_NUM(ADC_STREAM_UNIT)> m_sample_count = {};
    std::array<uint16_t, SOC_ADC_CHANNEL_NUM(ADC_STREAM_UNIT)> m_block_min = {};
    std::array<uint16_t, SOC_ADC_CHANNEL_NUM(ADC_STREAM_UNIT)> m_block_max = {};

    adc_channel_t m_vmon_channel;
};

#endif  // _ADC_STREAM_H_