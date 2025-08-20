#ifndef HAL_ENCODER_HANDLER_HPP
#define HAL_ENCODER_HANDLER_HPP

#include <array>
#include <utility>
#include <vector>

#include "driver/as5600_sensor.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

class EncoderHandler
{
   public:
    static EncoderHandler & get_instance();
    esp_err_t init(const std::vector<adc_channel_t> & channels);
    double get_rpm(size_t encoder_index);

    EncoderHandler(const EncoderHandler &) = delete;
    EncoderHandler & operator=(const EncoderHandler &) = delete;

   private:
    EncoderHandler();
    ~EncoderHandler();

    struct EncoderData
    {
        AS5600_Sensor sensor;
        std::array<std::pair<int64_t, double>, 20> buffer;
        size_t buffer_index = 0;
        bool buffer_full = false;
        double last_angle = 0.0;
        int revolution_count = 0;
        portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
    };

    static void s_encoder_task_wrapper(void * param);
    void encoder_task();
    double calculate_rpm(EncoderData & enc);

    std::vector<EncoderData> m_encoders;
    TaskHandle_t m_task_handle = nullptr;
    bool m_initialized = false;
};

#endif  // HAL_ENCODER_HANDLER_HPP
