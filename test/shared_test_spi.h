// Shared SPI bus for test suites that use the 74HC595, the CD4052B or the radio.
// Suites that only use the I2C bus include shared_test_bus.h instead so they do
// not pull the SPI driver into their build.

#ifndef _SHARED_TEST_SPI_H_
#define _SHARED_TEST_SPI_H_

#include <esp_err.h>

#include "SPIdev.h"

inline void shared_test_spi_bus()
{
    ESP_ERROR_CHECK(SPIdev::init());
}

#endif  // _SHARED_TEST_SPI_H_
