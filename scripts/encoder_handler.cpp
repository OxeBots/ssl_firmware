#include "hal/encoder_handler.hpp"

#include "hal/adc_reader.hpp"

EncoderHandler::EncoderHandler() = default;

EncoderHandler::~EncoderHandler()
{
    if (m_task_handle) vTaskDelete(m_task_handle);
}

EncoderHandler & EncoderHandler::get_instance()
{
    static EncoderHandler instance;
    return instance;
}

esp_err_t EncoderHandler::init(const std::vector<adc_channel_t> & channels)
{
    if (m_initialized) return ESP_ERR_INVALID_STATE;

    for (auto channel : channels)
        m_encoders.push_back({AS5600_Sensor(channel)});

    xTaskCreate(s_encoder_task_wrapper, "Task_EncoderHandler", 4096, this, 5,
                &m_task_handle);
    m_initialized = true;
    return ESP_OK;
}

void EncoderHandler::s_encoder_task_wrapper(void * param)
{
    static_cast<EncoderHandler *>(param)->encoder_task();
}

void EncoderHandler::encoder_task()
{
    constexpr TickType_t delay_ticks = pdMS_TO_TICKS(10);  // Faster scheduling
    auto & adc_reader = ADC_Reader::get_instance();
    size_t current_encoder = 0;

    while (true)
    {
        vTaskDelay(delay_ticks);

        if (m_encoders.empty()) continue;

        auto & enc = m_encoders[current_encoder];
        current_encoder = (current_encoder + 1) % m_encoders.size();

        // Process only one encoder per iteration
        // int raw = adc_reader.get_raw_data(enc.sensor.get_channel());
        // if (raw == -1) continue;

        // double angle = enc.sensor.convert_to_angle(raw);
        int64_t now = esp_timer_get_time();

        // Handle angle unwrapping and revolution counting
        // double delta = angle - enc.last_angle;
        // if (delta > 180.0)
        // {
        //     enc.revolution_count--;
        //     delta -= 360.0;
        // }
        // else if (delta < -180.0)
        // {
        //     enc.revolution_count++;
        //     delta += 360.0;
        // }

        // double unwrapped_angle = angle + (360.0 * enc.revolution_count);
        // enc.last_angle = angle;

        // portENTER_CRITICAL(&enc.spinlock);
        // enc.buffer[enc.buffer_index] = {now, unwrapped_angle};
        // enc.buffer_index = (enc.buffer_index + 1) % enc.buffer.size();
        // if (enc.buffer_index == 0) enc.buffer_full = true;
        // portEXIT_CRITICAL(&enc.spinlock);
    }
}

double EncoderHandler::calculate_rpm(EncoderData & enc)
{
    portENTER_CRITICAL(&enc.spinlock);
    auto buffer = enc.buffer;
    size_t index = enc.buffer_index;
    bool full = enc.buffer_full;
    portEXIT_CRITICAL(&enc.spinlock);

    if (!full && index < 2) return 0.0;

    const size_t count = full ? buffer.size() : index;
    if (count < 10) return 0.0;  // Require minimum 10 samples

    // Find oldest and newest samples
    size_t oldest_idx = full ? index : 0;
    size_t newest_idx = (index == 0) ? buffer.size() - 1 : index - 1;

    double angle_start = buffer[oldest_idx].second;
    double angle_end = buffer[newest_idx].second;
    int64_t time_start = buffer[oldest_idx].first;
    int64_t time_end = buffer[newest_idx].first;

    double total_angle_change = angle_end - angle_start;
    double total_time_sec = (time_end - time_start) / 1e6;

    if (total_time_sec < 0.001) return 0.0;  // Prevent division by zero

    return (total_angle_change / 360.0) * (60.0 / total_time_sec);
}

double EncoderHandler::get_rpm(size_t encoder_index)
{
    return (encoder_index < m_encoders.size())
             ? calculate_rpm(m_encoders[encoder_index])
             : 0.0;
}
