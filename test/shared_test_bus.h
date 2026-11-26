// Shared I2C bus for suites that run back-to-back in the merged test binary
// (test/test_all). Every hardware suite wires the same I2C port, so the bus is
// created once and reused instead of claiming the port again per suite. The
// shared SPI bus lives in shared_test_spi.h. Standalone suites behave
// identically: the first call creates what a fresh boot would have created anyway.

#ifndef _SHARED_TEST_BUS_H_
#define _SHARED_TEST_BUS_H_

#include <esp_err.h>

#include "I2Cdev.h"

// Returns the shared I2C master bus, creating it on first use.
inline i2c_master_bus_handle_t shared_test_i2c_bus()
{
    ESP_ERROR_CHECK(I2Cdev::init());
    return I2Cdev::get_bus_handle();
}

#endif  // _SHARED_TEST_BUS_H_
