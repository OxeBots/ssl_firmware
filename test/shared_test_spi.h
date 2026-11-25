// Shared SPI bus for test suites that use the 74HC595, the CD4052B or the radio.
// Suites that only use the I2C bus include shared_test_bus.h instead so they do
// not pull the SPI driver into their build.

#ifndef _SHARED_TEST_SPI_H_
#define _SHARED_TEST_SPI_H_

#include "SPIdev.h"

inline void shared_test_spi_bus()
{
    static bool initialized = false;
    if (initialized)
        return;

    // SPIdev::init() tolerates a host already claimed by an earlier suite.
    SPIdev::init();
    initialized = true;
}

#endif  // _SHARED_TEST_SPI_H_
