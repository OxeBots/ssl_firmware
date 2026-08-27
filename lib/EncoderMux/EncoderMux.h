#ifndef ENCODER_MUX_H
#define ENCODER_MUX_H

#include <esp_err.h>
#include <stdint.h>

#include "DigitalOutputExpander.h"

class EncoderMux
{
   public:
    struct Config
    {
        uint8_t select_a_bit;
        uint8_t select_b_bit;
        uint8_t inhibit_bit;
    };

    EncoderMux() = default;

    esp_err_t init(DigitalOutputExpander * output_expander, const Config & config);
    esp_err_t select(uint8_t channel);
    esp_err_t disable();

    uint8_t selected_channel() const;
    bool is_enabled() const;

   private:
    DigitalOutputExpander * m_output_expander = nullptr;
    Config m_config = {};
    uint8_t m_selected_channel = 0;
    bool m_enabled = false;
};

#endif  // ENCODER_MUX_H
