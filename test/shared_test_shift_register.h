// The 74HC595 output register is shared by every hardware suite that writes
// its outputs (74HC595, CD4052B select lines and the radio CE line). The latch
// is one SPI device, so the suites share this instance instead of each trying
// to register it again. Suites that assert on the output byte call write()
// with the value they need before testing.
//
// Kept out of shared_test_bus.h so the suites that do not touch the shift
// register do not need its header.

#ifndef _SHARED_TEST_SHIFT_REGISTER_H_
#define _SHARED_TEST_SHIFT_REGISTER_H_

#include <driver/gpio.h>
#include <esp_err.h>
#include <sdkconfig.h>

#include "74HC595.h"
#include "shared_test_bus.h"

inline HC595 & shared_test_shift_register()
{
    static HC595 expander;
    static bool initialized = false;
    if (!initialized)
    {
        shared_test_spi_bus();
        HC595::Config config = {.latch_gpio = (gpio_num_t)CONFIG_SHIFT_REGISTER_LATCH_GPIO,
                                .device_key = CONFIG_SHIFT_REGISTER_DEVICE_KEY,
                                .clock_hz = 1000000};
        ESP_ERROR_CHECK(expander.init(config));
        initialized = true;
    }
    return expander;
}

#endif  // _SHARED_TEST_SHIFT_REGISTER_H_
