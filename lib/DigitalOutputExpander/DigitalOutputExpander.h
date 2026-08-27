#ifndef DIGITAL_OUTPUT_EXPANDER_H
#define DIGITAL_OUTPUT_EXPANDER_H

#include <driver/gpio.h>
#include <esp_err.h>
#include <stdint.h>

class DigitalOutputExpander
{
   public:
    static constexpr uint8_t DEFAULT_DEVICE_KEY = 0xFE;

    struct Config
    {
        gpio_num_t latch_gpio;
        uint8_t device_key = DEFAULT_DEVICE_KEY;
        uint32_t clock_hz = 1000000;
    };

    DigitalOutputExpander() = default;

    esp_err_t init(const Config & config, uint8_t initial_value = 0);
    esp_err_t write(uint8_t value);
    esp_err_t update_mask(uint8_t mask, uint8_t value);

    uint8_t get_output() const;
    bool is_initialized() const;

   private:
    Config m_config = {};
    uint8_t m_output = 0;
    bool m_initialized = false;
};

#endif  // DIGITAL_OUTPUT_EXPANDER_H
