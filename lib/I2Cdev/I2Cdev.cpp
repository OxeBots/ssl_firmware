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

#include "I2Cdev.h"

#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

#include "sdkconfig.h"

#define I2C_NUM I2C_NUM_0

// Static member definition
uint16_t I2Cdev::readTimeout = I2CDEV_DEFAULT_READ_TIMEOUT;
i2c_master_bus_handle_t I2Cdev::global_bus_handle = NULL;
i2c_dev_cache_entry_t I2Cdev::device_cache[I2CDEV_MAX_CACHED_DEVICES];
uint8_t I2Cdev::cached_count = 0;

I2Cdev::I2Cdev()
{
}

/** Initialize I2Cdev class with bus handle
 * @param bus_handle I2C master bus handle
 */
void I2Cdev::init(i2c_master_bus_handle_t bus_handle)
{
    global_bus_handle = bus_handle;
    // Reset cache on init
    cached_count = 0;
    memset(device_cache, 0, sizeof(device_cache));
}

/** Get or create device handle for given address
 * @param devAddr I2C slave device address
 * @return Device handle or NULL on failure
 */
i2c_master_dev_handle_t I2Cdev::get_device_handle(uint8_t devAddr)
{
    if (global_bus_handle == NULL)
        return NULL;

    for (int i = 0; i < cached_count; i++)
    {
        if (device_cache[i].addr == devAddr)
            return device_cache[i].handle;
    }

    if (cached_count >= I2CDEV_MAX_CACHED_DEVICES)
    {
        ESP_LOGE("I2Cdev", "Device cache full! Increase I2CDEV_MAX_CACHED_DEVICES");
        return NULL;
    }

    i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = devAddr,
      .scl_speed_hz = 400000,
      .scl_wait_us = 0,
      .flags = {.disable_ack_check = 0},
    };

    i2c_master_dev_handle_t dev_handle;
    esp_err_t ret = i2c_master_bus_add_device(global_bus_handle, &dev_cfg, &dev_handle);

    if (ret != ESP_OK)
    {
        ESP_LOGE("I2Cdev", "Failed to add device 0x%02x: %s", devAddr, esp_err_to_name(ret));
        return NULL;
    }

    device_cache[cached_count].addr = devAddr;
    device_cache[cached_count].handle = dev_handle;
    cached_count++;

    return dev_handle;
}

/** Perform I2C transaction (write followed by read) or write only
 * @param devAddr I2C slave device address
 * @param write_buffer Buffer with data to write
 * @param write_len Number of bytes to write
 * @param read_buffer Buffer to store read data
 * @param read_len Number of bytes to read
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t I2Cdev::perform_transaction(uint8_t devAddr, uint8_t * write_buffer, size_t write_len, uint8_t * read_buffer,
                                      size_t read_len)
{
    i2c_master_dev_handle_t dev_handle = get_device_handle(devAddr);

    if (dev_handle == NULL)
        return ESP_FAIL;

    if (read_len > 0)
    {
        // Write Register Address -> Restart -> Read Data
        return i2c_master_transmit_receive(dev_handle, write_buffer, write_len, read_buffer, read_len, -1);
    }
    else
    {
        // Write Only
        return i2c_master_transmit(dev_handle, write_buffer, write_len, -1);
    }
}

/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in
 * I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t * data, uint16_t timeout)
{
    uint8_t b;
    uint8_t count = readByte(devAddr, regAddr, &b, timeout);
    *data = b & (1 << bitNum);
    return count;
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in
 * I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t * data,
                        uint16_t timeout)
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t count, b;
    if ((count = readByte(devAddr, regAddr, &b, timeout)) != 0)
    {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in
 * I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t * data, uint16_t timeout)
{
    return readBytes(devAddr, regAddr, 1, data, timeout);
}

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in
 * I2Cdev::readTimeout)
 * @return I2C_TransferReturn_TypeDef http://downloads.energymicro.com/documentation/doxygen/group__I2C.html
 */
int8_t I2Cdev::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t * data, uint16_t timeout)
{
    esp_err_t err = perform_transaction(devAddr, &regAddr, 1, data, length);
    return (err == ESP_OK) ? length : 0;
}

/**
 * read word
 * @param devAddr
 * @param regAddr
 * @param data
 * @param timeout
 * @return
 */
int8_t I2Cdev::readWord(uint8_t devAddr, uint8_t regAddr, uint16_t * data, uint16_t timeout)
{
    uint8_t msb[2] = {0, 0};
    // Note: I2Cdev usually assumes Big Endian words from sensors unless specified
    if (readBytes(devAddr, regAddr, 2, msb, timeout) == 2)
    {
        *data = (int16_t)((msb[0] << 8) | msb[1]);
        return 0;  // Success
    }

    return -1;  // Failure
}

/** Write single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param data New bit value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    uint8_t b;
    readByte(devAddr, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(devAddr, regAddr, b);
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b = 0;
    if (readByte(devAddr, regAddr, &b) != 0)
    {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1);  // shift data into correct position
        data &= mask;                      // zero all non-important bits in data
        b &= ~(mask);                      // zero all important bits in existing byte
        b |= data;                         // combine data with existing byte
        return writeByte(devAddr, regAddr, b);
    }
    return false;
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
    uint8_t buffer[2] = {regAddr, data};
    return perform_transaction(devAddr, buffer, 2, NULL, 0) == ESP_OK;
}

/** Write single word to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data)
{
    uint8_t buffer[3] = {regAddr, (uint8_t)(data >> 8), (uint8_t)(data & 0xff)};
    return perform_transaction(devAddr, buffer, 3, NULL, 0) == ESP_OK;
}

/** Write multiple bytes to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t * data)
{
    // We need to construct a single buffer: [RegAddr, Data0, Data1, ...]
    uint8_t * buffer = (uint8_t *)malloc(length + 1);
    if (!buffer)
        return false;

    buffer[0] = regAddr;
    memcpy(buffer + 1, data, length);

    esp_err_t err = perform_transaction(devAddr, buffer, length + 1, NULL, 0);

    free(buffer);
    return err == ESP_OK;
}
