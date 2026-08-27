#ifndef CD4052B_H
#define CD4052B_H

#include <esp_err.h>
#include <stdint.h>

#include "74HC595.h"

class CD4052B
{
   public:
    struct Config
    {
        uint8_t select_a_bit;
        uint8_t select_b_bit;
        uint8_t inhibit_bit;
    };

    CD4052B() = default;

    esp_err_t init(HC595 * shift_register, const Config & config);
    esp_err_t select(uint8_t channel);
    esp_err_t disable();

    uint8_t selected_channel() const;
    bool is_enabled() const;

   private:
    HC595 * m_shift_register = nullptr;
    Config m_config = {};
    uint8_t m_selected_channel = 0;
    bool m_enabled = false;
};

#endif  // CD4052B_H
