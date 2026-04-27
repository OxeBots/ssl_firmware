// I2Cdev library collection - Main I2C device class
// Abstracts bit and byte I2C R/W functions into a convenient class
// EFM32 stub port by Nicolas Baldeck <nicolas@pioupiou.fr>
// Ported to ESP-IDF i2c_master driver by Erick Suzart <ericksuzart@gmail.com>
// Based on Arduino's I2Cdev by Jeff Rowberg <jeff@rowberg.net>
//
// Changelog:
//      2015-01-02 - Initial release
//      2025-11-29 - Ported to ESP-IDF i2c_master driver

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2015 Jeff Rowberg, Nicolas Baldeck

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/
#ifndef I2CDEV_H
#define I2CDEV_H

#include <driver/i2c_master.h>
#include <esp_err.h>

#define I2CDEV_DEFAULT_READ_TIMEOUT 1000
#define I2CDEV_MAX_CACHED_DEVICES 8

typedef struct
{
    uint8_t addr;
    i2c_master_dev_handle_t handle;
} i2c_dev_cache_entry_t;

class I2Cdev
{
   public:
    I2Cdev();

    static void init(i2c_master_bus_handle_t bus_handle);

    static int8_t readBit(uint8_t devAddr,
                          uint8_t regAddr,
                          uint8_t bitNum,
                          uint8_t * data,
                          uint16_t timeout = I2Cdev::readTimeout);
    static int8_t readBits(uint8_t devAddr,
                           uint8_t regAddr,
                           uint8_t bitStart,
                           uint8_t length,
                           uint8_t * data,
                           uint16_t timeout = I2Cdev::readTimeout);
    static int8_t readByte(uint8_t devAddr,
                           uint8_t regAddr,
                           uint8_t * data,
                           uint16_t timeout = I2Cdev::readTimeout);
    static int8_t readWord(uint8_t devAddr,
                           uint8_t regAddr,
                           uint16_t * data,
                           uint16_t timeout = I2Cdev::readTimeout);
    static int8_t readBytes(uint8_t devAddr,
                            uint8_t regAddr,
                            uint8_t length,
                            uint8_t * data,
                            uint16_t timeout = I2Cdev::readTimeout);

    static bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
    static bool writeBits(
      uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
    static bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
    static bool writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data);
    static bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t * data);

    static uint16_t readTimeout;

   private:
    static i2c_master_bus_handle_t global_bus_handle;

    static i2c_dev_cache_entry_t device_cache[I2CDEV_MAX_CACHED_DEVICES];
    static uint8_t cached_count;

    static i2c_master_dev_handle_t get_device_handle(uint8_t devAddr);

    static esp_err_t perform_transaction(uint8_t devAddr,
                                         uint8_t * write_buffer,
                                         size_t write_len,
                                         uint8_t * read_buffer,
                                         size_t read_len);
};

#endif /* I2CDEV_H */
