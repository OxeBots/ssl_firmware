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
    constexpr TickType_t delay_ticks = pdMS_TO_TICKS(10);
    auto & adc_reader = ADC_Reader::get_instance();

    while (true)
    {
        vTaskDelay(delay_ticks);

        for (auto & enc : m_encoders)
        {
            int raw = adc_reader.get_raw_data(enc.sensor.get_channel());
            if (raw == -1) continue;

            double angle = enc.sensor.convert_to_angle(raw);
            int64_t now = esp_timer_get_time();

            portENTER_CRITICAL(&enc.spinlock);
            enc.buffer[enc.buffer_index] = {now, angle};
            enc.buffer_index = (enc.buffer_index + 1) % enc.buffer.size();
            if (enc.buffer_index == 0) enc.buffer_full = true;
            portEXIT_CRITICAL(&enc.spinlock);
        }
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
    size_t count = full ? buffer.size() : index;
    if (count < 10) return 0.0;

    std::vector<std::pair<int64_t, double>> sorted;
    sorted.reserve(count);
    for (size_t i = 0; i < count; ++i)
    {
        sorted.push_back(buffer[(index + i) % buffer.size()]);
    }

    double total_angle_change = 0.0;
    for (size_t i = 0; i < count - 1; ++i)
    {
        double delta = sorted[i + 1].second - sorted[i].second;
        if (delta > 180.0)
            delta -= 360.0;
        else if (delta < -180.0)
            delta += 360.0;
        total_angle_change += delta;
    }

    double total_time_sec = (sorted.back().first - sorted.front().first) / 1e6;
    if (total_time_sec < 1e-6) return 0.0;

    return (total_angle_change / 360.0) * (60.0 / total_time_sec);
}

double EncoderHandler::get_rpm(size_t encoder_index)
{
    return (encoder_index < m_encoders.size())
             ? calculate_rpm(m_encoders[encoder_index])
             : 0.0;
}
