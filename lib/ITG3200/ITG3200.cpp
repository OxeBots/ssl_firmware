/**
 * @file ITG3200.cpp
 * @brief I2Cdev library collection - ITG3200 I2C device class
 * * Based on InvenSense ITG-3200 datasheet rev. 1.4, 3/30/2010 (PS-ITG-3200A-00-01.4)
 * 7/31/2011 by Jeff Rowberg <jeff@rowberg.net>
 * Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
 * * DISCLAIMER: This code is based on the I2Cdev library collection but has been modified and is not equal to the
 * original.
 *
 * Changelog:
 * 2011-07-31 - initial release
 * 
 * ============================================
 * I2Cdev device library code is placed under the MIT license
 * Copyright (c) 2011 Jeff Rowberg
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ===============================================
 */

#include "ITG3200.h"

static const char * TAG = "ITG3200";
static const char * NVS_NS = "gyro_calib";
static const char * NVS_KEY_BLOB = "calib_blob";

/** Default constructor, uses default I2C address.
 * @param address I2C address, either DEFAULT_ADDRESS or ADDRESS_ALT_HIGH
 *
 * @see DEFAULT_ADDRESS
 * @see ADDRESS_ALT_HIGH
 */
ITG3200::ITG3200(uint8_t address) : m_dev_addr(address)
{
}

/**
 * @brief Power on and prepare for general usage.
 * This will activate the gyroscope, so be sure to adjust the power settings
 * after you call this method if you want it to enter standby mode, or another
 * less demanding mode of operation. This also sets the gyroscope to use the
 * X-axis gyro for a clock source. Note that it doesn't have any delays in the
 * routine, which means you might want to add ~50ms to be safe if you happen
 * to need to read gyro data immediately after initialization. The data will
 * flow in either case, but the first reports may have higher error offsets.
 */
void ITG3200::init()
{
    set_full_scale_range(FullScaleRange::FS_2000);
    set_clock_source(ClockSource::PLL_XGYRO);
}

/**
 * @brief Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool ITG3200::test_connection()
{
    return get_device_id() == 0b110100;
}

/** Get Device ID.
 * This register is used to verify the identity of the device (0b110100).
 * @return Device ID (should be 0x34, 52 dec, 64 oct)
 * @see Register::WHO_AM_I
 * @see RA_DEVID_BIT
 * @see RA_DEVID_LENGTH
 */
uint8_t ITG3200::get_device_id()
{
    I2Cdev::readBits(m_dev_addr, static_cast<uint8_t>(Register::WHO_AM_I), DEVID_BIT, DEVID_LENGTH, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Device ID.
 * Write a new ID into the WHO_AM_I register (no idea why this should ever be
 * necessary though).
 * @param id New device ID to set.
 * @see get_device_id()
 * @see Register::WHO_AM_I
 * @see RA_DEVID_BIT
 * @see RA_DEVID_LENGTH
 */
void ITG3200::set_device_id(uint8_t id)
{
    I2Cdev::writeBits(m_dev_addr, static_cast<uint8_t>(Register::WHO_AM_I), DEVID_BIT, DEVID_LENGTH, id);
}

/**
 * @brief Get sample rate.
 * This register determines the sample rate of the ITG-3200 gyros. The gyros'
 * outputs are sampled internally at either 1kHz or 8kHz, determined by the
 * DLPF_CFG setting (see register 22). This sampling is then filtered digitally
 * and delivered into the sensor registers after the number of cycles determined
 * by this register. The sample rate is given by the following formula:
 *
 * F_sample = F_internal / (divider+1), where F_internal is either 1kHz or 8kHz
 *
 * As an example, if the internal sampling is at 1kHz, then setting this
 * register to 7 would give the following:
 *
 * F_sample = 1kHz / (7 + 1) = 125Hz, or 8ms per sample
 *
 * @return Current sample rate
 * @see set_dlpf_bandwidth()
 * @see Register::SMPLRT_DIV
 */
uint8_t ITG3200::get_rate()
{
    I2Cdev::readByte(m_dev_addr, static_cast<uint8_t>(Register::SMPLRT_DIV), m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set sample rate.
 * @param rate New sample rate
 * @see get_rate()
 * @see set_dlpf_bandwidth()
 * @see Register::SMPLRT_DIV
 */
void ITG3200::set_rate(uint8_t rate)
{
    I2Cdev::writeByte(m_dev_addr, static_cast<uint8_t>(Register::SMPLRT_DIV), rate);
}

/**
 * @brief Set full-scale range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below. The power-on-reset value of FS_SEL is 00h.
 * Set to 03h for proper operation.
 *
 * 0 = Reserved
 * 1 = Reserved
 * 2 = Reserved
 * 3 = +/- 2000 degrees/sec
 *
 * @return Current full-scale range setting
 * @see Register::DLPF_FS
 * @see DF_FS_SEL_BIT
 * @see DF_FS_SEL_LENGTH
 */
ITG3200::FullScaleRange ITG3200::get_full_scale_range()
{
    I2Cdev::readBits(m_dev_addr, static_cast<uint8_t>(Register::DLPF_FS), DF_FS_SEL_BIT, DF_FS_SEL_LENGTH, m_buffer);
    return static_cast<FullScaleRange>(m_buffer[0]);
}

/**
 * @brief full-scale range setting.
 * @param range New full-scale range value
 * @see get_full_scale_range()
 * @see Register::DLPF_FS
 * @see DF_FS_SEL_BIT
 * @see DF_FS_SEL_LENGTH
 */
void ITG3200::set_full_scale_range(FullScaleRange range)
{
    I2Cdev::writeBits(m_dev_addr, static_cast<uint8_t>(Register::DLPF_FS), DF_FS_SEL_BIT, DF_FS_SEL_LENGTH,
                      static_cast<uint8_t>(range));
}

/**
 * @brief digital low-pass filter bandwidth.
 * The DLPF_CFG parameter sets the digital low pass filter configuration. It
 * also determines the internal sampling rate used by the device as shown in
 * the table below.
 *
 * DLPF_CFG | Low-Pass Filter Bandwidth | Internal Sample Rate
 * ---------+---------------------------+---------------------
 * 0        | 256Hz                     | 8kHz
 * 1        | 188Hz                     | 1kHz
 * 2        | 98Hz                      | 1kHz
 * 3        | 42Hz                      | 1kHz
 * 4        | 20Hz                      | 1kHz
 * 5        | 10Hz                      | 1kHz
 * 6        | 5Hz                       | 1kHz
 * 7        | Reserved                  | Reserved
 *
 * @return DLFP bandwidth setting
 * @see Register::DLPF_FS
 * @see DF_DLPF_CFG_BIT
 * @see DF_DLPF_CFG_LENGTH
 */
ITG3200::Bandwidth ITG3200::get_dlpf_bandwidth()
{
    I2Cdev::readBits(m_dev_addr, static_cast<uint8_t>(Register::DLPF_FS), DF_DLPF_CFG_BIT, DF_DLPF_CFG_LENGTH,
                     m_buffer);
    return static_cast<Bandwidth>(m_buffer[0]);
}

/**
 * @brief Set digital low-pass filter bandwidth.
 * @param bandwidth New DLFP bandwidth setting
 * @see get_dlpf_bandwidth()
 * @see Register::DLPF_FS
 * @see DF_DLPF_CFG_BIT
 * @see DF_DLPF_CFG_LENGTH
 */
void ITG3200::set_dlpf_bandwidth(Bandwidth bandwidth)
{
    I2Cdev::writeBits(m_dev_addr, static_cast<uint8_t>(Register::DLPF_FS), DF_DLPF_CFG_BIT, DF_DLPF_CFG_LENGTH,
                      static_cast<uint8_t>(bandwidth));
}

/**
 * @brief Get interrupt logic level mode.
 * Will be set 0 for active-high, 1 for active-low.
 * @return Current interrupt mode (0=active-high, 1=active-low)
 * @see Register::INT_CFG
 * @see INTCFG_ACTL_BIT
 */
bool ITG3200::get_interrupt_mode()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_CFG), INTCFG_ACTL_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set interrupt logic level mode.
 * @param mode New interrupt mode (0=active-high, 1=active-low)
 * @see get_interrupt_mode()
 * @see Register::INT_CFG
 * @see INTCFG_ACTL_BIT
 */
void ITG3200::set_interrupt_mode(bool mode)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::INT_CFG), INTCFG_ACTL_BIT, mode);
}

/**
 * @brief Get interrupt drive mode.
 * Will be set 0 for push-pull, 1 for open-drain.
 * @return Current interrupt drive mode (0=push-pull, 1=open-drain)
 * @see Register::INT_CFG
 * @see INTCFG_OPEN_BIT
 */
bool ITG3200::get_interrupt_drive()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_CFG), INTCFG_OPEN_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set interrupt drive mode.
 * @param drive New interrupt drive mode (0=push-pull, 1=open-drain)
 * @see get_interrupt_drive()
 * @see Register::INT_CFG
 * @see INTCFG_OPEN_BIT
 */
void ITG3200::set_interrupt_drive(bool drive)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::INT_CFG), INTCFG_OPEN_BIT, drive);
}

/**
 * @brief Get interrupt latch mode.
 * Will be set 0 for 50us-pulse, 1 for latch-until-int-cleared.
 * @return Current latch mode (0=50us-pulse, 1=latch-until-int-cleared)
 * @see Register::INT_CFG
 * @see INTCFG_LATCH_INT_EN_BIT
 */
bool ITG3200::get_interrupt_latch()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_CFG), INTCFG_LATCH_INT_EN_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set interrupt latch mode.
 * @param latch New latch mode (0=50us-pulse, 1=latch-until-int-cleared)
 * @see get_interrupt_latch()
 * @see Register::INT_CFG
 * @see INTCFG_LATCH_INT_EN_BIT
 */
void ITG3200::set_interrupt_latch(bool latch)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::INT_CFG), INTCFG_LATCH_INT_EN_BIT, latch);
}

/**
 * @brief Get interrupt latch clear mode.
 * Will be set 0 for status-read-only, 1 for any-register-read.
 * @return Current latch clear mode (0=status-read-only, 1=any-register-read)
 * @see Register::INT_CFG
 * @see INTCFG_INT_ANYRD_2CLEAR_BIT
 */
bool ITG3200::get_interrupt_latch_clear()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_CFG), INTCFG_INT_ANYRD_2CLEAR_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set interrupt latch clear mode.
 * @param clear New latch clear mode (0=status-read-only, 1=any-register-read)
 * @see get_interrupt_latch_clear()
 * @see Register::INT_CFG
 * @see INTCFG_INT_ANYRD_2CLEAR_BIT
 */
void ITG3200::set_interrupt_latch_clear(bool clear)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::INT_CFG), INTCFG_INT_ANYRD_2CLEAR_BIT, clear);
}

/**
 * @brief Get "device ready" interrupt enabled setting.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled setting
 * @see Register::INT_CFG
 * @see INTCFG_ITG_RDY_EN_BIT
 */
bool ITG3200::get_int_device_ready_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_CFG), INTCFG_ITG_RDY_EN_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set "device ready" interrupt enabled setting.
 * @param enabled New interrupt enabled setting
 * @see get_int_device_ready_enabled()
 * @see Register::INT_CFG
 * @see INTCFG_ITG_RDY_EN_BIT
 */
void ITG3200::set_int_device_ready_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::INT_CFG), INTCFG_ITG_RDY_EN_BIT, enabled);
}

/**
 * @brief Get "data ready" interrupt enabled setting.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled setting
 * @see Register::INT_CFG
 * @see INTCFG_RAW_RDY_EN_BIT
 */
bool ITG3200::get_int_data_ready_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_CFG), INTCFG_RAW_RDY_EN_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set "data ready" interrupt enabled setting.
 * @param enabled New interrupt enabled setting
 * @see get_int_data_ready_enabled()
 * @see Register::INT_CFG
 * @see INTCFG_RAW_RDY_EN_BIT
 */
void ITG3200::set_int_data_ready_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::INT_CFG), INTCFG_RAW_RDY_EN_BIT, enabled);
}

/**
 * @brief Device Ready interrupt status.
 * The ITG_RDY interrupt indicates that the PLL is ready and gyroscopic data can
 * be read.
 * @return Device Ready interrupt status
 * @see Register::INT_STATUS
 * @see INTSTAT_ITG_RDY_BIT
 */
bool ITG3200::get_int_device_ready_status()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_STATUS), INTSTAT_ITG_RDY_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Get Data Ready interrupt status.
 * In normal use, the RAW_DATA_RDY interrupt is used to determine when new
 * sensor data is available in and of the sensor registers (27 to 32).
 * @return Data Ready interrupt status
 * @see Register::INT_STATUS
 * @see INTSTAT_RAW_DATA_READY_BIT
 */
bool ITG3200::get_int_data_ready_status()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_STATUS), INTSTAT_RAW_DATA_READY_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Get current internal temperature.
 * @return Temperature reading in 16-bit 2's complement format
 * @see Register::TEMP_OUT_H
 */
int16_t ITG3200::get_temperature()
{
    I2Cdev::readBytes(m_dev_addr, static_cast<uint8_t>(Register::TEMP_OUT_H), sizeof(int16_t), m_buffer);
    return (((int16_t)m_buffer[0]) << 8) | m_buffer[1];
}

/**
 * @brief Get 3-axis gyroscope readings.
 * @param x 16-bit signed integer container for X-axis rotation
 * @param y 16-bit signed integer container for Y-axis rotation
 * @param z 16-bit signed integer container for Z-axis rotation
 * @see Register::GYRO_XOUT_H
 */
void ITG3200::get_rotation(int16_t * x, int16_t * y, int16_t * z)
{
    I2Cdev::readBytes(m_dev_addr, static_cast<uint8_t>(Register::GYRO_XOUT_H), sizeof(int16_t) * 3, m_buffer);
    *x = ((((int16_t)m_buffer[0]) << 8) | m_buffer[1]) - m_x_offset;
    *y = ((((int16_t)m_buffer[2]) << 8) | m_buffer[3]) - m_y_offset;
    *z = ((((int16_t)m_buffer[4]) << 8) | m_buffer[5]) - m_z_offset;
}

/**
 * @brief Get X-axis gyroscope reading.
 * @return X-axis rotation measurement in 16-bit 2's complement format
 * @see Register::GYRO_XOUT_H
 */
int16_t ITG3200::get_rotation_x()
{
    I2Cdev::readBytes(m_dev_addr, static_cast<uint8_t>(Register::GYRO_XOUT_H), sizeof(int16_t), m_buffer);
    return ((((int16_t)m_buffer[0]) << 8) | m_buffer[1]) - m_x_offset;
}

/**
 * @brief Get Y-axis gyroscope reading.
 * @return Y-axis rotation measurement in 16-bit 2's complement format
 * @see Register::GYRO_YOUT_H
 */
int16_t ITG3200::get_rotation_y()
{
    I2Cdev::readBytes(m_dev_addr, static_cast<uint8_t>(Register::GYRO_YOUT_H), sizeof(int16_t), m_buffer);
    return ((((int16_t)m_buffer[0]) << 8) | m_buffer[1]) - m_y_offset;
}

/**
 * @brief Get Z-axis gyroscope reading.
 * @return Z-axis rotation measurement in 16-bit 2's complement format
 * @see Register::GYRO_ZOUT_H
 */
int16_t ITG3200::get_rotation_z()
{
    I2Cdev::readBytes(m_dev_addr, static_cast<uint8_t>(Register::GYRO_ZOUT_H), sizeof(int16_t), m_buffer);
    return ((((int16_t)m_buffer[0]) << 8) | m_buffer[1]) - m_z_offset;
}

/**
 * @brief Trigger a full device reset.
 * A small delay of ~50ms may be desirable after triggering a reset.
 * @see Register::PWR_MGM
 * @see PWR_H_RESET_BIT
 */
void ITG3200::reset()
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::PWR_MGM), PWR_H_RESET_BIT, true);
}

/**
 * @brief Get sleep mode status.
 * Setting the SLEEP bit in the register puts the device into very low power
 * sleep mode. In this mode, only the serial interface and internal registers
 * remain active, allowing for a very low standby current. Clearing this bit
 * puts the device back into normal mode. To save power, the individual standby
 * selections for each of the gyros should be used if any gyro axis is not used
 * by the application.
 * @return Current sleep mode enabled status
 * @see Register::PWR_MGM
 * @see PWR_SLEEP_BIT
 */
bool ITG3200::get_sleep_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::PWR_MGM), PWR_SLEEP_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see getSleepEnabled()
 * @see Register::PWR_MGM
 * @see PWR_SLEEP_BIT
 */
void ITG3200::set_sleep_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::PWR_MGM), PWR_SLEEP_BIT, enabled);
}

/**
 * @brief Get X-axis standby enabled status.
 * If enabled, the X-axis will not gather or report data (or use power).
 * @return Current X-axis standby enabled status
 * @see Register::PWR_MGM
 * @see PWR_STBY_XG_BIT
 */
bool ITG3200::get_standby_x_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::PWR_MGM), PWR_STBY_XG_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set X-axis standby enabled status.
 * @param enabled New X-axis standby enabled status
 * @see get_standby_x_enabled()
 * @see Register::PWR_MGM
 * @see PWR_STBY_XG_BIT
 */
void ITG3200::set_standby_x_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::PWR_MGM), PWR_STBY_XG_BIT, enabled);
}

/**
 * @brief Get Y-axis standby enabled status.
 * If enabled, the Y-axis will not gather or report data (or use power).
 * @return Current Y-axis standby enabled status
 * @see Register::PWR_MGM
 * @see PWR_STBY_YG_BIT
 */
bool ITG3200::get_standby_y_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::PWR_MGM), PWR_STBY_YG_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set Y-axis standby enabled status.
 * @param enabled New Y-axis standby enabled status
 * @see get_standby_y_enabled()
 * @see Register::PWR_MGM
 * @see PWR_STBY_YG_BIT
 */
void ITG3200::set_standby_y_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::PWR_MGM), PWR_STBY_YG_BIT, enabled);
}

/**
 * @brief Get Z-axis standby enabled status.
 * If enabled, the Z-axis will not gather or report data (or use power).
 * @return Current Z-axis standby enabled status
 * @see Register::PWR_MGM
 * @see PWR_STBY_ZG_BIT
 */
bool ITG3200::get_standby_z_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::PWR_MGM), PWR_STBY_ZG_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set Z-axis standby enabled status.
 * @param enabled New Z-axis standby enabled status
 * @see get_standby_z_enabled()
 * @see Register::PWR_MGM
 * @see PWR_STBY_ZG_BIT
 */
void ITG3200::set_standby_z_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::PWR_MGM), PWR_STBY_ZG_BIT, enabled);
}

/**
 * @brief Get clock source setting.
 * @return Current clock source setting
 * @see Register::PWR_MGM
 * @see PWR_CLK_SEL_BIT
 * @see PWR_CLK_SEL_LENGTH
 */
ITG3200::ClockSource ITG3200::get_clock_source()
{
    I2Cdev::readBits(m_dev_addr, static_cast<uint8_t>(Register::PWR_MGM), PWR_CLK_SEL_BIT, PWR_CLK_SEL_LENGTH,
                     m_buffer);
    return static_cast<ClockSource>(m_buffer[0]);
}

/**
 * @brief Set clock source setting.
 * On power up, the ITG-3200 defaults to the internal oscillator. It is highly recommended that the device is configured
 * to use one of the gyros (or an external clock) as the clock reference, due to the improved stability.
 *
 * The CLK_SEL setting determines the device clock source as follows:
 *
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Reserved
 *
 * @param source New clock source setting
 * @see get_clock_source()
 * @see Register::PWR_MGM
 * @see PWR_CLK_SEL_BIT
 * @see PWR_CLK_SEL_LENGTH
 */
void ITG3200::set_clock_source(ClockSource source)
{
    I2Cdev::writeBits(m_dev_addr, static_cast<uint8_t>(Register::PWR_MGM), PWR_CLK_SEL_BIT, PWR_CLK_SEL_LENGTH,
                      static_cast<uint8_t>(source));
}

/**
 * @brief Calibrate the Gyroscope.
 * The sensor must be STATIONARY during this process. It calculates the average bias (zero-rate
 * error) and stores it to be subtracted from future readings.
 * @param samples Number of samples to read for averaging (default 1000)
 */
void ITG3200::calibrate(uint16_t samples)
{
    long sumX = 0;
    long sumY = 0;
    long sumZ = 0;

    m_x_offset = 0;
    m_y_offset = 0;
    m_z_offset = 0;

    int16_t rx, ry, rz;

    for (uint16_t i = 0; i < samples; i++)
    {
        get_rotation(&rx, &ry, &rz);
        sumX += rx;
        sumY += ry;
        sumZ += rz;
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }

    m_x_offset = sumX / samples;
    m_y_offset = sumY / samples;
    m_z_offset = sumZ / samples;
}

/**
 * @brief Set gyroscope offsets.
 * This can be used to manually set the gyroscope offsets, or to restore previously calculated
 * offsets from a prior calibration.
 *
 * @param x X-axis offset to set
 * @param y Y-axis offset to set
 * @param z Z-axis offset to set
 */
void ITG3200::set_offsets(int16_t x, int16_t y, int16_t z)
{
    m_x_offset = x;
    m_y_offset = y;
    m_z_offset = z;
}

/**
 * @brief Get current gyroscope offsets. These are subtracted from the raw gyro readings to get the final output values.
 * @param x Container for X-axis offset
 * @param y Container for Y-axis offset
 * @param z Container for Z-axis offset
 */
void ITG3200::get_offsets(int16_t * x, int16_t * y, int16_t * z) const
{
    *x = m_x_offset;
    *y = m_y_offset;
    *z = m_z_offset;
}

/**
 * @brief Save current gyroscope calibration to NVS.
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t ITG3200::save_calibration_to_nvs()
{
    int16_t data[3] = {m_x_offset, m_y_offset, m_z_offset};
    esp_err_t err = NVSManager::save_blob(NVS_NS, NVS_KEY_BLOB, data, sizeof(data));
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Gyroscope calibration saved to NVS.");
    }
    return err;
}

/**
 * @brief Load gyroscope calibration from NVS.
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t ITG3200::load_calibration_from_nvs()
{
    int16_t data[3];
    size_t req_size = sizeof(data);

    esp_err_t err = NVSManager::load_blob(NVS_NS, NVS_KEY_BLOB, data, &req_size);
    if (err == ESP_OK)
    {
        if (req_size == sizeof(data))
        {
            m_x_offset = data[0];
            m_y_offset = data[1];
            m_z_offset = data[2];
            ESP_LOGI(TAG, "Loaded gyroscope calibration: Off[%d, %d, %d]", m_x_offset, m_y_offset, m_z_offset);
            return ESP_OK;
        }
        else
        {
            ESP_LOGE(TAG, "NVS Blob size mismatch! Expected %zu, got %zu", sizeof(data), req_size);
            return ESP_ERR_NVS_INVALID_LENGTH;
        }
    }

    if (err == ESP_ERR_NVS_NOT_FOUND)
        ESP_LOGW(TAG, "Gyroscope calibration not found in NVS.");

    return err;
}

/**
 * @brief Check if the gyroscope has been calibrated (i.e., if offsets are non-zero).
 * @return True if calibrated, false otherwise
 */
bool ITG3200::is_calibrated() const
{
    return !(m_x_offset == 0 &&  //
             m_y_offset == 0 &&  //
             m_z_offset == 0);
}
