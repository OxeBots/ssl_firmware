/**
 * I2Cdev library collection - ADXL345 I2C device class
 * Based on Analog Devices ADXL345 datasheet rev. C, 5/2011
 * 7/31/2011 by Jeff Rowberg <jeff@rowberg.net>
 * Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
 *
 * DISCLAIMER: This code is based on the I2Cdev library collection but has been modified and is not equal to the
 * original.
 *
 * Changelog:
 *     2011-07-31 - initial release
 *
 *  ============================================
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

#include "ADXL345.h"

#include "NVSManager.h"

static const char * TAG = "ADXL345";
static const char * NVS_NS = "accel_calib";
static const char * NVS_KEY_BLOB = "calib_blob";

/**
 * @brief Specific address constructor.
 * @param address I2C address
 *
 * @see ADXL345::DEFAULT_ADDRESS
 * @see ADXL345::ADDRESS_ALT_HIGH
 */
ADXL345::ADXL345(uint8_t address) : m_dev_addr(address)
{
    clear_calibration();
}

/**
 * @brief Power on and prepare for general usage.
 * This will activate the accelerometer, so be sure to adjust the power settings
 * after you call this method if you want it to enter standby mode, or another
 * less demanding mode of operation.
 */
void ADXL345::init()
{
    I2Cdev::writeByte(m_dev_addr, static_cast<uint8_t>(Register::POWER_CTL), 0);
    set_auto_sleep_enabled(true);
    set_measure_enabled(true);
}

/**
 * @brief Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool ADXL345::test_connection()
{
    return get_device_id() == 0xE5;
}

/**
 * @brief Get Device ID.
 * The DEVID register holds a fixed device ID code of 0xE5 (345 octal).
 * @return Device ID (should be 0xE5, 229 dec, 345 oct)
 * @see Register::DEVID
 */
uint8_t ADXL345::get_device_id()
{
    I2Cdev::readByte(m_dev_addr, static_cast<uint8_t>(Register::DEVID), m_buffer);
    return m_buffer[0];
}

/**
 * @brief Get tap threshold.
 * The THRESH_TAP register is eight bits and holds the threshold value for tap
 * interrupts. The data format is unsigned, therefore, the magnitude of the tap
 * event is compared with the value in THRESH_TAP for normal tap detection. The
 * scale factor is 62.5 mg/LSB (that is, 0xFF = 16 g). A value of 0 may result
 * in undesirable behavior if single tap/double tap interrupts are enabled.
 * @return Tap threshold (scaled at 62.5 mg/LSB)
 * @see Register::THRESH_TAP
 */
uint8_t ADXL345::get_tap_threshold()
{
    I2Cdev::readByte(m_dev_addr, static_cast<uint8_t>(Register::THRESH_TAP), m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set tap threshold.
 * @param threshold Tap magnitude threshold (scaled at 62.5 mg/LSB)
 * @see get_tap_threshold()
 * @see Register::THRESH_TAP
 */
void ADXL345::set_tap_threshold(uint8_t threshold)
{
    I2Cdev::writeByte(m_dev_addr, static_cast<uint8_t>(Register::THRESH_TAP), threshold);
}

/**
 * @brief Get axis offsets.
 * The OFSX, OFSY, and OFSZ registers are each eight bits and offer user-set
 * offset adjustments in twos complement format with a scale factor of 15.6
 * mg/LSB (that is, 0x7F = 2 g). The value stored in the offset registers is
 * automatically added to the acceleration data, and the resulting value is
 * stored in the output data registers. For additional information regarding
 * offset calibration and the use of the offset registers, refer to the Offset
 * Calibration section of the datasheet.
 * @param x X axis offset container
 * @param y Y axis offset container
 * @param z Z axis offset container
 * @see Register::OFSX
 * @see Register::OFSY
 * @see Register::OFSZ
 */
void ADXL345::get_offset(int8_t * x, int8_t * y, int8_t * z)
{
    I2Cdev::readBytes(m_dev_addr, static_cast<uint8_t>(Register::OFSX), 3, m_buffer);
    *x = m_buffer[0];
    *y = m_buffer[1];
    *z = m_buffer[2];
}

/**
 * @brief Set axis offsets.
 * @param x X axis offset value
 * @param y Y axis offset value
 * @param z Z axis offset value
 * @see get_offset()
 * @see Register::OFSX
 * @see Register::OFSY
 * @see Register::OFSZ
 */
void ADXL345::set_offset(int8_t x, int8_t y, int8_t z)
{
    I2Cdev::writeByte(m_dev_addr, static_cast<uint8_t>(Register::OFSX), x);
    I2Cdev::writeByte(m_dev_addr, static_cast<uint8_t>(Register::OFSY), y);
    I2Cdev::writeByte(m_dev_addr, static_cast<uint8_t>(Register::OFSZ), z);
}

/**
 * @brief Get X axis offset.
 * @return X axis offset value
 * @see get_offset()
 * @see Register::OFSX
 */
int8_t ADXL345::get_offset_x()
{
    I2Cdev::readByte(m_dev_addr, static_cast<uint8_t>(Register::OFSX), m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set X axis offset.
 * @param x X axis offset value
 * @see get_offset_x()
 * @see Register::OFSX
 */
void ADXL345::set_offset_x(int8_t x)
{
    I2Cdev::writeByte(m_dev_addr, static_cast<uint8_t>(Register::OFSX), x);
}

/**
 * @brief Get Y axis offset.
 * @return Y axis offset value
 * @see get_offset()
 * @see Register::OFSY
 */
int8_t ADXL345::get_offset_y()
{
    I2Cdev::readByte(m_dev_addr, static_cast<uint8_t>(Register::OFSY), m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set Y axis offset.
 * @param y Y axis offset value
 * @see get_offset_y()
 * @see Register::OFSY
 */
void ADXL345::set_offset_y(int8_t y)
{
    I2Cdev::writeByte(m_dev_addr, static_cast<uint8_t>(Register::OFSY), y);
}

/**
 * @brief Get Z axis offset.
 * @return Z axis offset value
 * @see get_offset()
 * @see Register::OFSZ
 */
int8_t ADXL345::get_offset_z()
{
    I2Cdev::readByte(m_dev_addr, static_cast<uint8_t>(Register::OFSZ), m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set Z axis offset.
 * @param z Z axis offset value
 * @see get_offset_z()
 * @see Register::OFSZ
 */
void ADXL345::set_offset_z(int8_t z)
{
    I2Cdev::writeByte(m_dev_addr, static_cast<uint8_t>(Register::OFSZ), z);
}

/**
 * @brief Get tap duration.
 * The DUR register is eight bits and contains an unsigned time value
 * representing the maximum time that an event must be above the THRESH_TAP
 * threshold to qualify as a tap event. The scale factor is 625 us/LSB. A value
 * of 0 disables the single tap/ double tap functions.
 * @return Tap duration (scaled at 625 us/LSB)
 * @see Register::DUR
 */
uint8_t ADXL345::get_tap_duration()
{
    I2Cdev::readByte(m_dev_addr, static_cast<uint8_t>(Register::DUR), m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set tap duration.
 * @param duration Tap duration (scaled at 625 us/LSB)
 * @see get_tap_duration()
 * @see Register::DUR
 */
void ADXL345::set_tap_duration(uint8_t duration)
{
    I2Cdev::writeByte(m_dev_addr, static_cast<uint8_t>(Register::DUR), duration);
}

/**
 * @brief Get tap latency.
 * The latent register is eight bits and contains an unsigned time value
 * representing the wait time from the detection of a tap event to the start of
 * the time window (defined by the window register) during which a possible
 * second tap event can be detected. The scale factor is 1.25 ms/LSB. A value of
 * 0 disables the double tap function.
 * @return Tap latency (scaled at 1.25 ms/LSB)
 * @see Register::LATENT
 */
uint8_t ADXL345::get_double_tap_latency()
{
    I2Cdev::readByte(m_dev_addr, static_cast<uint8_t>(Register::LATENT), m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set tap latency.
 * @param latency Tap latency (scaled at 1.25 ms/LSB)
 * @see get_double_tap_latency()
 * @see Register::LATENT
 */
void ADXL345::set_double_tap_latency(uint8_t latency)
{
    I2Cdev::writeByte(m_dev_addr, static_cast<uint8_t>(Register::LATENT), latency);
}

/**
 * @brief Get double tap window.
 * The window register is eight bits and contains an unsigned time value
 * representing the amount of time after the expiration of the latency time
 * (determined by the latent register) during which a second valid tap can
 * begin. The scale factor is 1.25 ms/LSB. A value of 0 disables the double tap
 * function.
 * @return Double tap window (scaled at 1.25 ms/LSB)
 * @see Register::WINDOW
 */
uint8_t ADXL345::get_double_tap_window()
{
    I2Cdev::readByte(m_dev_addr, static_cast<uint8_t>(Register::WINDOW), m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set double tap window.
 * @param window Double tap window (scaled at 1.25 ms/LSB)
 * @see get_double_tap_window()
 * @see Register::WINDOW
 */
void ADXL345::set_double_tap_window(uint8_t window)
{
    I2Cdev::writeByte(m_dev_addr, static_cast<uint8_t>(Register::WINDOW), window);
}

/**
 * @brief Get activity threshold.
 * The THRESH_ACT register is eight bits and holds the threshold value for
 * detecting activity. The data format is unsigned, so the magnitude of the
 * activity event is compared with the value in the THRESH_ACT register. The
 * scale factor is 62.5 mg/LSB. A value of 0 may result in undesirable behavior
 * if the activity interrupt is enabled.
 * @return Activity threshold (scaled at 62.5 mg/LSB)
 * @see Register::THRESH_ACT
 */
uint8_t ADXL345::get_activity_threshold()
{
    I2Cdev::readByte(m_dev_addr, static_cast<uint8_t>(Register::THRESH_ACT), m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set activity threshold.
 * @param threshold Activity threshold (scaled at 62.5 mg/LSB)
 * @see get_activity_threshold()
 * @see Register::THRESH_ACT
 */
void ADXL345::set_activity_threshold(uint8_t threshold)
{
    I2Cdev::writeByte(m_dev_addr, static_cast<uint8_t>(Register::THRESH_ACT), threshold);
}

/**
 * @brief Get inactivity threshold.
 * The THRESH_INACT register is eight bits and holds the threshold value for
 * detecting inactivity. The data format is unsigned, so the magnitude of the
 * inactivity event is compared with the value in the THRESH_INACT register. The
 * scale factor is 62.5 mg/LSB. A value of 0 may result in undesirable behavior
 * if the inactivity interrupt is enabled.
 * @return Inactivity threshold (scaled at 62.5 mg/LSB)
 * @see Register::THRESH_INACT
 */
uint8_t ADXL345::get_inactivity_threshold()
{
    I2Cdev::readByte(m_dev_addr, static_cast<uint8_t>(Register::THRESH_INACT), m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set inactivity threshold.
 * @param threshold Inactivity threshold (scaled at 62.5 mg/LSB)
 * @see get_inactivity_threshold()
 * @see Register::THRESH_INACT
 */
void ADXL345::set_inactivity_threshold(uint8_t threshold)
{
    I2Cdev::writeByte(m_dev_addr, static_cast<uint8_t>(Register::THRESH_INACT), threshold);
}

/**
 * @brief Get inactivity time.
 * The TIME_INACT register is eight bits and contains an unsigned time value
 * representing the amount of time that acceleration must be less than the value
 * in the THRESH_INACT register for inactivity to be declared. The scale factor
 * is 1 sec/LSB. Unlike the other interrupt functions, which use unfiltered data
 * (see the Threshold sectionof the datasheet), the inactivity function uses
 * filtered output data. At least one output sample must be generated for the
 * inactivity interrupt to be triggered. This results in the function appearing
 * unresponsive if the TIME_INACT register is set to a value less than the time
 * constant of the output data rate. A value of 0 results in an interrupt when
 * the output data is less than the value in the THRESH_INACT register.
 * @return Inactivity time (scaled at 1 sec/LSB)
 * @see Register::TIME_INACT
 */
uint8_t ADXL345::get_inactivity_time()
{
    I2Cdev::readByte(m_dev_addr, static_cast<uint8_t>(Register::TIME_INACT), m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set inactivity time.
 * @param time Inactivity time (scaled at 1 sec/LSB)
 * @see get_inactivity_time()
 * @see Register::TIME_INACT
 */
void ADXL345::set_inactivity_time(uint8_t time)
{
    I2Cdev::writeByte(m_dev_addr, static_cast<uint8_t>(Register::TIME_INACT), time);
}

/**
 * @brief Get activity AC/DC coupling.
 * A setting of 0 selects dc-coupled operation, and a setting of 1 enables
 * ac-coupled operation. In dc-coupled operation, the current acceleration
 * magnitude is compared directly with THRESH_ACT and THRESH_INACT to determine
 * whether activity or inactivity is detected.
 *
 * In ac-coupled operation for activity detection, the acceleration value at the
 * start of activity detection is taken as a reference value. New samples of
 * acceleration are then compared to this reference value, and if the magnitude
 * of the difference exceeds the THRESH_ACT value, the device triggers an
 * activity interrupt.
 *
 * Similarly, in ac-coupled operation for inactivity detection, a reference
 * value is used for comparison and is updated whenever the device exceeds the
 * inactivity threshold. After the reference value is selected, the device
 * compares the magnitude of the difference between the reference value and the
 * current acceleration with THRESH_INACT. If the difference is less than the
 * value in THRESH_INACT for the time in TIME_INACT, the device is considered
 * inactive and the inactivity interrupt is triggered.
 *
 * @return Activity coupling (0 = DC, 1 = AC)
 * @see Register::ACT_INACT_CTL
 * @see AIC_ACT_AC_BIT
 */
bool ADXL345::get_activity_ac()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::ACT_INACT_CTL), AIC_ACT_AC_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set activity AC/DC coupling.
 * @param enabled Activity AC/DC coupling (TRUE for AC, FALSE for DC)
 * @see get_activity_ac()
 * @see Register::ACT_INACT_CTL
 * @see AIC_ACT_AC_BIT
 */
void ADXL345::set_activity_ac(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::ACT_INACT_CTL), AIC_ACT_AC_BIT, enabled);
}

/**
 * @brief Get X axis activity monitoring inclusion.
 * For all "get[In]Activity*Enabled()" methods: a setting of 1 enables x-, y-,
 * or z-axis participation in detecting activity or inactivity. A setting of 0
 * excludes the selected axis from participation. If all axes are excluded, the
 * function is disabled. For activity detection, all participating axes are
 * logically ORded, causing the activity function to trigger when any of the
 * participating axes exceeds the threshold. For inactivity detection, all
 * participating axes are logically ANDded, causing the inactivity function to
 * trigger only if all participating axes are below the threshold for the
 * specified time.
 * @return X axis activity monitoring enabled value
 * @see get_activity_ac()
 * @see Register::ACT_INACT_CTL
 * @see AIC_ACT_X_BIT
 */
bool ADXL345::get_activity_x_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::ACT_INACT_CTL), AIC_ACT_X_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set X axis activity monitoring inclusion.
 * @param enabled X axis activity monitoring inclusion value
 * @see get_activity_ac()
 * @see get_activity_x_enabled()
 * @see Register::ACT_INACT_CTL
 * @see AIC_ACT_X_BIT
 */
void ADXL345::set_activity_x_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::ACT_INACT_CTL), AIC_ACT_X_BIT, enabled);
}

/**
 * @brief Get Y axis activity monitoring.
 * @return Y axis activity monitoring enabled value
 * @see get_activity_ac()
 * @see get_activity_x_enabled()
 * @see Register::ACT_INACT_CTL
 * @see AIC_ACT_Y_BIT
 */
bool ADXL345::get_activity_y_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::ACT_INACT_CTL), AIC_ACT_Y_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set Y axis activity monitoring inclusion.
 * @param enabled Y axis activity monitoring inclusion value
 * @see get_activity_ac()
 * @see get_activity_x_enabled()
 * @see Register::ACT_INACT_CTL
 * @see AIC_ACT_Y_BIT
 */
void ADXL345::set_activity_y_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::ACT_INACT_CTL), AIC_ACT_Y_BIT, enabled);
}

/**
 * @brief Get Z axis activity monitoring.
 * @return Z axis activity monitoring enabled value
 * @see get_activity_ac()
 * @see get_activity_x_enabled()
 * @see Register::ACT_INACT_CTL
 * @see AIC_ACT_Z_BIT
 */
bool ADXL345::get_activity_z_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::ACT_INACT_CTL), AIC_ACT_Z_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set Z axis activity monitoring inclusion.
 * @param enabled Z axis activity monitoring inclusion value
 * @see get_activity_ac()
 * @see get_activity_x_enabled()
 * @see Register::ACT_INACT_CTL
 * @see AIC_ACT_Z_BIT
 */
void ADXL345::set_activity_z_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::ACT_INACT_CTL), AIC_ACT_Z_BIT, enabled);
}

/**
 * @brief Get inactivity AC/DC coupling.
 * @return Inctivity coupling (0 = DC, 1 = AC)
 * @see get_activity_ac()
 * @see Register::ACT_INACT_CTL
 * @see AIC_INACT_AC_BIT
 */
bool ADXL345::get_inactivity_ac()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::ACT_INACT_CTL), AIC_INACT_AC_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set inctivity AC/DC coupling.
 * @param enabled Inactivity AC/DC coupling (TRUE for AC, FALSE for DC)
 * @see get_activity_ac()
 * @see Register::ACT_INACT_CTL
 * @see AIC_INACT_AC_BIT
 */
void ADXL345::set_inactivity_ac(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::ACT_INACT_CTL), AIC_INACT_AC_BIT, enabled);
}

/**
 * @brief Get X axis inactivity monitoring.
 * @return X axis inactivity monitoring enabled value
 * @see get_activity_ac()
 * @see get_activity_x_enabled()
 * @see Register::ACT_INACT_CTL
 * @see AIC_INACT_X_BIT
 */
bool ADXL345::get_inactivity_x_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::ACT_INACT_CTL), AIC_INACT_X_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set X axis inactivity monitoring inclusion.
 * @param enabled X axis inactivity monitoring inclusion value
 * @see get_activity_ac()
 * @see get_activity_x_enabled()
 * @see Register::ACT_INACT_CTL
 * @see AIC_INACT_X_BIT
 */
void ADXL345::set_inactivity_x_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::ACT_INACT_CTL), AIC_INACT_X_BIT, enabled);
}

/**
 * @brief Get Y axis inactivity monitoring.
 * @return Y axis inactivity monitoring enabled value
 * @see get_activity_ac()
 * @see get_activity_x_enabled()
 * @see Register::ACT_INACT_CTL
 * @see AIC_INACT_Y_BIT
 */
bool ADXL345::get_inactivity_y_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::ACT_INACT_CTL), AIC_INACT_Y_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set Y axis inactivity monitoring inclusion.
 * @param enabled Y axis inactivity monitoring inclusion value
 * @see get_activity_ac()
 * @see get_activity_x_enabled()
 * @see Register::ACT_INACT_CTL
 * @see AIC_INACT_Y_BIT
 */
void ADXL345::set_inactivity_y_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::ACT_INACT_CTL), AIC_INACT_Y_BIT, enabled);
}

/**
 * @brief Get Z axis inactivity monitoring.
 * @return Z axis inactivity monitoring enabled value
 * @see get_activity_ac()
 * @see get_activity_x_enabled()
 * @see Register::ACT_INACT_CTL
 * @see AIC_INACT_Z_BIT
 */
bool ADXL345::get_inactivity_z_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::ACT_INACT_CTL), AIC_INACT_Z_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set Z axis inactivity monitoring inclusion.
 * @param enabled Z axis inactivity monitoring inclusion value
 * @see get_activity_ac()
 * @see get_activity_x_enabled()
 * @see Register::ACT_INACT_CTL
 * @see AIC_INACT_Z_BIT
 */
void ADXL345::set_inactivity_z_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::ACT_INACT_CTL), AIC_INACT_Z_BIT, enabled);
}

/**
 * @brief Get freefall threshold value.
 * The THRESH_FF register is eight bits and holds the threshold value, in
 * unsigned format, for free-fall detection. The acceleration on all axes is
 * compared with the value in THRESH_FF to determine if a free-fall event
 * occurred. The scale factor is 62.5 mg/LSB. Note that a value of 0 mg may
 * result in undesirable behavior if the free-fall interrupt is enabled. Values
 * between 300 mg and 600 mg (0x05 to 0x09) are recommended.
 * @return Freefall threshold value (scaled at 62.5 mg/LSB)
 * @see Register::THRESH_FF
 */
uint8_t ADXL345::get_freefall_threshold()
{
    I2Cdev::readByte(m_dev_addr, static_cast<uint8_t>(Register::THRESH_FF), m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set freefall threshold value.
 * @param threshold Freefall threshold value (scaled at 62.5 mg/LSB)
 * @see get_freefall_threshold()
 * @see Register::THRESH_FF
 */
void ADXL345::set_freefall_threshold(uint8_t threshold)
{
    I2Cdev::writeByte(m_dev_addr, static_cast<uint8_t>(Register::THRESH_FF), threshold);
}

/**
 * @brief Get freefall time value.
 * The TIME_FF register is eight bits and stores an unsigned time value
 * representing the minimum time that the value of all axes must be less than
 * THRESH_FF to generate a free-fall interrupt. The scale factor is 5 ms/LSB. A
 * value of 0 may result in undesirable behavior if the free-fall interrupt is
 * enabled. Values between 100 ms and 350 ms (0x14 to 0x46) are recommended.
 * @return Freefall time value (scaled at 5 ms/LSB)
 * @see get_freefall_threshold()
 * @see Register::TIME_FF
 */
uint8_t ADXL345::get_freefall_time()
{
    I2Cdev::readByte(m_dev_addr, static_cast<uint8_t>(Register::TIME_FF), m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set freefall time value.
 * @param threshold Freefall time value (scaled at 5 ms/LSB)
 * @see get_freefall_time()
 * @see Register::TIME_FF
 */
void ADXL345::set_freefall_time(uint8_t time)
{
    I2Cdev::writeByte(m_dev_addr, static_cast<uint8_t>(Register::TIME_FF), time);
}

/**
 * @brief Get double-tap fast-movement suppression.
 * Setting the suppress bit suppresses double tap detection if acceleration
 * greater than the value in THRESH_TAP is present between taps. See the Tap
 * Detection section in the datasheet for more details.
 * @return Double-tap fast-movement suppression value
 * @see get_tap_threshold()
 * @see Register::TAP_AXES
 * @see TAPAXIS_SUP_BIT
 */
bool ADXL345::get_tap_axis_suppress()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::TAP_AXES), TAPAXIS_SUP_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set double-tap fast-movement suppression.
 * @param enabled Double-tap fast-movement suppression value
 * @see get_tap_axis_suppress()
 * @see Register::TAP_AXES
 * @see TAPAXIS_SUP_BIT
 */
void ADXL345::set_tap_axis_suppress(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::TAP_AXES), TAPAXIS_SUP_BIT, enabled);
}

/**
 * @brief Get double-tap fast-movement suppression.
 * A setting of 1 in the TAP_X enable bit enables x-axis participation in tap
 * detection. A setting of 0 excludes the selected axis from participation in
 * tap detection.
 * @return Double-tap fast-movement suppression value
 * @see get_tap_threshold()
 * @see Register::TAP_AXES
 * @see TAPAXIS_X_BIT
 */
bool ADXL345::get_tap_axis_x_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::TAP_AXES), TAPAXIS_X_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set tap detection X axis inclusion.
 * @param enabled X axis tap detection enabled value
 * @see get_tap_axis_x_enabled()
 * @see Register::TAP_AXES
 * @see TAPAXIS_X_BIT
 */
void ADXL345::set_tap_axis_x_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::TAP_AXES), TAPAXIS_X_BIT, enabled);
}

/**
 * @brief Get tap detection Y axis inclusion.
 * A setting of 1 in the TAP_Y enable bit enables y-axis participation in tap
 * detection. A setting of 0 excludes the selected axis from participation in
 * tap detection.
 * @return Double-tap fast-movement suppression value
 * @see get_tap_threshold()
 * @see Register::TAP_AXES
 * @see TAPAXIS_Y_BIT
 */
bool ADXL345::get_tap_axis_y_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::TAP_AXES), TAPAXIS_Y_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set tap detection Y axis inclusion.
 * @param enabled Y axis tap detection enabled value
 * @see get_tap_axis_y_enabled()
 * @see Register::TAP_AXES
 * @see TAPAXIS_Y_BIT
 */
void ADXL345::set_tap_axis_y_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::TAP_AXES), TAPAXIS_Y_BIT, enabled);
}

/**
 * @brief Get tap detection Z axis inclusion.
 * A setting of 1 in the TAP_Z enable bit enables z-axis participation in tap
 * detection. A setting of 0 excludes the selected axis from participation in
 * tap detection.
 * @return Double-tap fast-movement suppression value
 * @see get_tap_threshold()
 * @see Register::TAP_AXES
 * @see TAPAXIS_Z_BIT
 */
bool ADXL345::get_tap_axis_z_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::TAP_AXES), TAPAXIS_Z_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set tap detection Z axis inclusion.
 * @param enabled Z axis tap detection enabled value
 * @see get_tap_axis_z_enabled()
 * @see Register::TAP_AXES
 * @see TAPAXIS_Z_BIT
 */
void ADXL345::set_tap_axis_z_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::TAP_AXES), TAPAXIS_Z_BIT, enabled);
}

/**
 * @brief Get X axis activity source flag.
 * These bits indicate the first axis involved in a tap or activity event. A
 * setting of 1 corresponds to involvement in the event, and a setting of 0
 * corresponds to no involvement. When new data is available, these bits are not
 * cleared but are overwritten by the new data. The ACT_TAP_STATUS register
 * should be read before clearing the interrupt. Disabling an axis from
 * participation clears the corresponding source bit when the next activity or
 * single tap/double tap event occurs.
 * @return X axis activity source flag
 * @see Register::ACT_TAP_STATUS
 * @see TAPSTAT_ACTX_BIT
 */
bool ADXL345::get_activity_source_x()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::ACT_TAP_STATUS), TAPSTAT_ACTX_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Get Y axis activity source flag.
 * @return Y axis activity source flag
 * @see get_activity_source_x()
 * @see Register::ACT_TAP_STATUS
 * @see TAPSTAT_ACTY_BIT
 */
bool ADXL345::get_activity_source_y()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::ACT_TAP_STATUS), TAPSTAT_ACTY_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Get Z axis activity source flag.
 * @return Z axis activity source flag
 * @see get_activity_source_x()
 * @see Register::ACT_TAP_STATUS
 * @see TAPSTAT_ACTZ_BIT
 */
bool ADXL345::get_activity_source_z()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::ACT_TAP_STATUS), TAPSTAT_ACTZ_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Get sleep mode flag.
 * A setting of 1 in the asleep bit indicates that the part is asleep, and a
 * setting of 0 indicates that the part is not asleep. This bit toggles only if
 * the device is configured for auto sleep. See the AUTO_SLEEP Bit section of
 * the datasheet for more information on autosleep mode.
 * @return Sleep mode enabled flag
 * @see Register::ACT_TAP_STATUS
 * @see TAPSTAT_ASLEEP_BIT
 */
bool ADXL345::get_asleep()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::ACT_TAP_STATUS), TAPSTAT_ASLEEP_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Get X axis tap source flag.
 * @return X axis tap source flag
 * @see get_tap_source_x()
 * @see Register::ACT_TAP_STATUS
 * @see TAPSTAT_TAPX_BIT
 */
bool ADXL345::get_tap_source_x()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::ACT_TAP_STATUS), TAPSTAT_TAPX_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Get Y axis tap source flag.
 * @return Y axis tap source flag
 * @see get_tap_source_x()
 * @see Register::ACT_TAP_STATUS
 * @see TAPSTAT_TAPY_BIT
 */
bool ADXL345::get_tap_source_y()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::ACT_TAP_STATUS), TAPSTAT_TAPY_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Get Z axis tap source flag.
 * @return Z axis tap source flag
 * @see get_tap_source_x()
 * @see Register::ACT_TAP_STATUS
 * @see TAPSTAT_TAPZ_BIT
 */
bool ADXL345::get_tap_source_z()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::ACT_TAP_STATUS), TAPSTAT_TAPZ_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Get low power enabled status.
 * A setting of 0 in the LOW_POWER bit selects normal operation, and a setting
 * of 1 selects reduced power operation, which has somewhat higher noise (see
 * the Power Modes section of the datasheet for details).
 * @return Low power enabled status
 * @see Register::BW_RATE
 * @see BW_LOWPOWER_BIT
 */
bool ADXL345::get_low_power_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::BW_RATE), BW_LOWPOWER_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set low power enabled status.
 * @see get_low_power_enabled()
 * @param enabled Low power enable setting
 * @see Register::BW_RATE
 * @see BW_LOWPOWER_BIT
 */
void ADXL345::set_low_power_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::BW_RATE), BW_LOWPOWER_BIT, enabled);
}

/**
 * @brief Get measurement data rate.
 * These bits select the device bandwidth and output data rate (see Table 7 and
 * Table 8 in the datasheet for details). The default value is 0x0A, which
 * translates to a 100 Hz output data rate. An output data rate should be
 * selected that is appropriate for the communication protocol and frequency
 * selected. Selecting too high of an output data rate with a low communication
 * speed results in samples being discarded.
 * @return Data rate (0x0 - 0xF)
 * @see Register::BW_RATE
 * @see BW_RATE_BIT
 * @see BW_RATE_LENGTH
 */
uint8_t ADXL345::get_rate()
{
    I2Cdev::readBits(m_dev_addr, static_cast<uint8_t>(Register::BW_RATE), BW_RATE_BIT, BW_RATE_LENGTH, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set measurement data rate.
 * 0x7 =  12.5Hz
 * 0x8 =  25Hz, increasing or decreasing by factors of 2, so:
 * 0x9 =  50Hz
 * 0xA = 100Hz
 * @param rate New data rate (0x0 - 0xF)
 * @see Register::BW_RATE
 * @see BW_RATE_BIT
 * @see BW_RATE_LENGTH
 */
void ADXL345::set_rate(uint8_t rate)
{
    I2Cdev::writeBits(m_dev_addr, static_cast<uint8_t>(Register::BW_RATE), BW_RATE_BIT, BW_RATE_LENGTH, rate);
}

/**
 * @brief Get activity/inactivity serial linkage status.
 * A setting of 1 in the link bit with both the activity and inactivity
 * functions enabled delays the start of the activity function until
 * inactivity is detected. After activity is detected, inactivity detection
 * begins, preventing the detection of activity. This bit serially links the
 * activity and inactivity functions. When this bit is set to 0, the inactivity
 * and activity functions are concurrent. Additional information can be found
 * in the Link Mode section of the datasheet.
 *
 * When clearing the link bit, it is recommended that the part be placed into
 * standby mode and then set back to measurement mode with a subsequent write.
 * This is done to ensure that the device is properly biased if sleep mode is
 * manually disabled; otherwise, the first few samples of data after the link
 * bit is cleared may have additional noise, especially if the device was asleep
 * when the bit was cleared.
 *
 * @return Link status
 * @see Register::POWER_CTL
 * @see PCTL_LINK_BIT
 */
bool ADXL345::get_link_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::POWER_CTL), PCTL_LINK_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set activity/inactivity serial linkage status.
 * @param enabled New link status
 * @see Register::POWER_CTL
 * @see PCTL_LINK_BIT
 */
void ADXL345::set_link_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::POWER_CTL), PCTL_LINK_BIT, enabled);
}

/**
 * @brief Get auto-sleep enabled status.
 * If the link bit is set, a setting of 1 in the AUTO_SLEEP bit enables the
 * auto-sleep functionality. In this mode, the ADXL345 auto-matically switches
 * to sleep mode if the inactivity function is enabled and inactivity is
 * detected (that is, when acceleration is below the THRESH_INACT value for at
 * least the time indicated by TIME_INACT). If activity is also enabled, the
 * ADXL345 automatically wakes up from sleep after detecting activity and
 * returns to operation at the output data rate set in the BW_RATE register. A
 * setting of 0 in the AUTO_SLEEP bit disables automatic switching to sleep
 * mode. See the description of the Sleep Bit in this section of the datasheet
 * for more information on sleep mode.
 *
 * If the link bit is not set, the AUTO_SLEEP feature is disabled and setting
 * the AUTO_SLEEP bit does not have an impact on device operation. Refer to the
 * Link Bit section or the Link Mode section for more information on utilization
 * of the link feature.
 *
 * When clearing the AUTO_SLEEP bit, it is recommended that the part be placed
 * into standby mode and then set back to measure-ment mode with a subsequent
 * write. This is done to ensure that the device is properly biased if sleep
 * mode is manually disabled; otherwise, the first few samples of data after the
 * AUTO_SLEEP bit is cleared may have additional noise, especially if the device
 * was asleep when the bit was cleared.
 *
 * @return Auto-sleep enabled status
 * @see get_activity_threshold()
 * @see get_inactivity_threshold()
 * @see get_inactivity_time()
 * @see Register::POWER_CTL
 * @see PCTL_AUTOSLEEP_BIT
 */
bool ADXL345::get_auto_sleep_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::POWER_CTL), PCTL_AUTOSLEEP_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set auto-sleep enabled status.
 * @param enabled New auto-sleep status
 * @see get_auto_sleep_enabled()
 * @see Register::POWER_CTL
 * @see PCTL_AUTOSLEEP_BIT
 */
void ADXL345::set_auto_sleep_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::POWER_CTL), PCTL_AUTOSLEEP_BIT, enabled);
}

/**
 * @brief Get measurement enabled status.
 * A setting of 0 in the measure bit places the part into standby mode, and a
 * setting of 1 places the part into measurement mode. The ADXL345 powers up in
 * standby mode with minimum power consumption.
 * @return Measurement enabled status
 * @see Register::POWER_CTL
 * @see PCTL_MEASURE_BIT
 */
bool ADXL345::get_measure_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::POWER_CTL), PCTL_MEASURE_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set measurement enabled status.
 * @param enabled Measurement enabled status
 * @see get_measure_enabled()
 * @see Register::POWER_CTL
 * @see PCTL_MEASURE_BIT
 */
void ADXL345::set_measure_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::POWER_CTL), PCTL_MEASURE_BIT, enabled);
}

/**
 * @brief Get sleep mode enabled status.
 * A setting of 0 in the sleep bit puts the part into the normal mode of
 * operation, and a setting of 1 places the part into sleep mode. Sleep mode
 * suppresses DATA_READY, stops transmission of data to FIFO, and switches the
 * sampling rate to one specified by the wakeup bits. In sleep mode, only the
 * activity function can be used. When the DATA_READY interrupt is suppressed,
 * the output data registers (Register 0x32 to Register 0x37) are still updated
 * at the sampling rate set by the wakeup bits (D1:D0).
 *
 * When clearing the sleep bit, it is recommended that the part be placed into
 * standby mode and then set back to measurement mode with a subsequent write.
 * This is done to ensure that the device is properly biased if sleep mode is
 * manually disabled; otherwise, the first few samples of data after the sleep
 * bit is cleared may have additional noise, especially if the device was asleep
 * when the bit was cleared.
 *
 * @return Sleep enabled status
 * @see Register::POWER_CTL
 * @see PCTL_SLEEP_BIT
 */
bool ADXL345::get_sleep_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::POWER_CTL), PCTL_SLEEP_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set sleep mode enabled status.
 * @param enabled Sleep mode enabled status
 * @see get_sleep_enabled()
 * @see Register::POWER_CTL
 * @see PCTL_SLEEP_BIT
 */
void ADXL345::set_sleep_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::POWER_CTL), PCTL_SLEEP_BIT, enabled);
}

/**
 * @brief Get wakeup frequency.
 * These bits control the frequency of readings in sleep mode as described in
 * Table 20 in the datasheet. (That is, 0 = 8Hz, 1 = 4Hz, 2 = 2Hz, 3 = 1Hz)
 * @return Wakeup frequency (0x0 - 0x3, indicating 8/4/2/1Hz respectively)
 * @see Register::POWER_CTL
 * @see PCTL_WAKEUP_BIT
 */
uint8_t ADXL345::get_wakeup_frequency()
{
    I2Cdev::readBits(m_dev_addr, static_cast<uint8_t>(Register::POWER_CTL), PCTL_WAKEUP_BIT, PCTL_WAKEUP_LENGTH,
                     m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set wakeup frequency.
 * @param frequency Wakeup frequency (0x0 - 0x3, indicating 8/4/2/1Hz respectively)
 * @see get_wakeup_frequency()
 * @see Register::POWER_CTL
 * @see PCTL_WAKEUP_BIT
 */
void ADXL345::set_wakeup_frequency(uint8_t frequency)
{
    I2Cdev::writeBits(m_dev_addr, static_cast<uint8_t>(Register::POWER_CTL), PCTL_WAKEUP_BIT, PCTL_WAKEUP_LENGTH,
                      frequency);
}

/**
 * @brief Get DATA_READY interrupt enabled status.
 * Setting bits in this register to a value of 1 enables their respective
 * functions to generate interrupts, whereas a value of 0 prevents the functions
 * from generating interrupts. The DATA_READY, watermark, and overrun bits
 * enable only the interrupt output; the functions are always enabled. It is
 * recommended that interrupts be configured before enabling their outputs.
 * @return DATA_READY interrupt enabled status.
 * @see Register::INT_ENABLE
 * @see INT_DATA_READY_BIT
 */
bool ADXL345::get_int_data_ready_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_ENABLE), INT_DATA_READY_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set DATA_READY interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see get_int_data_ready_enabled()
 * @see Register::INT_ENABLE
 * @see INT_DATA_READY_BIT
 */
void ADXL345::set_int_data_ready_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::INT_ENABLE), INT_DATA_READY_BIT, enabled);
}

/**
 * @brief Set SINGLE_TAP interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see get_int_single_tap_enabled()
 * @see Register::INT_ENABLE
 * @see INT_SINGLE_TAP_BIT
 */
bool ADXL345::get_int_single_tap_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_ENABLE), INT_SINGLE_TAP_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set SINGLE_TAP interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see get_int_single_tap_enabled()
 * @see Register::INT_ENABLE
 * @see INT_SINGLE_TAP_BIT
 */
void ADXL345::set_int_single_tap_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::INT_ENABLE), INT_SINGLE_TAP_BIT, enabled);
}

/**
 * @brief Get DOUBLE_TAP interrupt enabled status.
 * @return Interrupt enabled status
 * @see get_int_data_ready_enabled()
 * @see Register::INT_ENABLE
 * @see INT_DOUBLE_TAP_BIT
 */
bool ADXL345::get_int_double_tap_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_ENABLE), INT_DOUBLE_TAP_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set DOUBLE_TAP interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see get_int_double_tap_enabled()
 * @see Register::INT_ENABLE
 * @see INT_DOUBLE_TAP_BIT
 */
void ADXL345::set_int_double_tap_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::INT_ENABLE), INT_DOUBLE_TAP_BIT, enabled);
}

/**
 * @brief Get ACTIVITY interrupt enabled status.
 * @return Interrupt enabled status
 * @see Register::INT_ENABLE
 * @see INT_ACTIVITY_BIT
 */
bool ADXL345::get_int_activity_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_ENABLE), INT_ACTIVITY_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set ACTIVITY interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see get_int_data_ready_enabled()
 * @see Register::INT_ENABLE
 * @see INT_ACTIVITY_BIT
 */
void ADXL345::set_int_activity_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::INT_ENABLE), INT_ACTIVITY_BIT, enabled);
}

/**
 * @brief Get INACTIVITY interrupt enabled status.
 * @return Interrupt enabled status
 * @see get_int_data_ready_enabled()
 * @see Register::INT_ENABLE
 * @see INT_INACTIVITY_BIT
 */
bool ADXL345::get_int_inactivity_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_ENABLE), INT_INACTIVITY_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set INACTIVITY interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see get_int_data_ready_enabled()
 * @see Register::INT_ENABLE
 * @see INT_INACTIVITY_BIT
 */
void ADXL345::set_int_inactivity_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::INT_ENABLE), INT_INACTIVITY_BIT, enabled);
}

/**
 * @brief Get FREE_FALL interrupt enabled status.
 * @return Interrupt enabled status
 * @see get_int_data_ready_enabled()
 * @see Register::INT_ENABLE
 * @see INT_FREE_FALL_BIT
 */
bool ADXL345::get_int_freefall_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_ENABLE), INT_FREE_FALL_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set FREE_FALL interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see get_int_data_ready_enabled()
 * @see Register::INT_ENABLE
 * @see INT_FREE_FALL_BIT
 */
void ADXL345::set_int_freefall_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::INT_ENABLE), INT_FREE_FALL_BIT, enabled);
}

/**
 * @brief Get WATERMARK interrupt enabled status.
 * @return Interrupt enabled status
 * @see get_int_data_ready_enabled()
 * @see Register::INT_ENABLE
 * @see INT_WATERMARK_BIT
 */
bool ADXL345::get_int_watermark_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_ENABLE), INT_WATERMARK_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set WATERMARK interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see get_int_data_ready_enabled()
 * @see Register::INT_ENABLE
 * @see INT_WATERMARK_BIT
 */
void ADXL345::set_int_watermark_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::INT_ENABLE), INT_WATERMARK_BIT, enabled);
}

/**
 * @brief Get OVERRUN interrupt enabled status.
 * @return Interrupt enabled status
 * @see get_int_data_ready_enabled()
 * @see Register::INT_ENABLE
 * @see INT_OVERRUN_BIT
 */
bool ADXL345::get_int_overrun_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_ENABLE), INT_OVERRUN_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set OVERRUN interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see get_int_data_ready_enabled()
 * @see Register::INT_ENABLE
 * @see INT_OVERRUN_BIT
 */
void ADXL345::set_int_overrun_enabled(bool enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::INT_ENABLE), INT_OVERRUN_BIT, enabled);
}

/**
 * @brief Get DATA_READY interrupt pin.
 * Any bits set to 0 in this register send their respective interrupts to the
 * INT1 pin, whereas bits set to 1 send their respective interrupts to the INT2
 * pin. All selected interrupts for a given pin are OR'ed.
 * @return Interrupt pin setting
 * @see Register::INT_MAP
 * @see INT_DATA_READY_BIT
 */
uint8_t ADXL345::get_int_data_ready_pin()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_MAP), INT_DATA_READY_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set DATA_READY interrupt pin.
 * @param pin Interrupt pin setting
 * @see get_int_data_ready_pin()
 * @see Register::INT_MAP
 * @see INT_DATA_READY_BIT
 */
void ADXL345::set_int_data_ready_pin(uint8_t pin)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::INT_MAP), INT_DATA_READY_BIT, pin);
}

/**
 * @brief Get SINGLE_TAP interrupt pin.
 * @return Interrupt pin setting
 * @see get_int_data_ready_pin()
 * @see Register::INT_MAP
 * @see INT_SINGLE_TAP_BIT
 */
uint8_t ADXL345::get_int_single_tap_pin()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_MAP), INT_SINGLE_TAP_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set SINGLE_TAP interrupt pin.
 * @param pin Interrupt pin setting
 * @see get_int_data_ready_pin()
 * @see Register::INT_MAP
 * @see INT_SINGLE_TAP_BIT
 */
void ADXL345::set_int_single_tap_pin(uint8_t pin)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::INT_MAP), INT_SINGLE_TAP_BIT, pin);
}

/**
 * @brief Get DOUBLE_TAP interrupt pin.
 * @return Interrupt pin setting
 * @see get_int_data_ready_pin()
 * @see Register::INT_MAP
 * @see INT_DOUBLE_TAP_BIT
 */
uint8_t ADXL345::get_int_double_tap_pin()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_MAP), INT_DOUBLE_TAP_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set DOUBLE_TAP interrupt pin.
 * @param pin Interrupt pin setting
 * @see get_int_data_ready_pin()
 * @see Register::INT_MAP
 * @see INT_DOUBLE_TAP_BIT
 */
void ADXL345::set_int_double_tap_pin(uint8_t pin)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::INT_MAP), INT_DOUBLE_TAP_BIT, pin);
}

/**
 * @brief Get ACTIVITY interrupt pin.
 * @return Interrupt pin setting
 * @see get_int_data_ready_pin()
 * @see Register::INT_MAP
 * @see INT_ACTIVITY_BIT
 */
uint8_t ADXL345::get_int_activity_pin()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_MAP), INT_ACTIVITY_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set ACTIVITY interrupt pin.
 * @param pin Interrupt pin setting
 * @see get_int_data_ready_pin()
 * @see Register::INT_MAP
 * @see INT_ACTIVITY_BIT
 */
void ADXL345::set_int_activity_pin(uint8_t pin)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::INT_MAP), INT_ACTIVITY_BIT, pin);
}

/**
 * @brief Get INACTIVITY interrupt pin.
 * @return Interrupt pin setting
 * @see get_int_data_ready_pin()
 * @see Register::INT_MAP
 * @see INT_INACTIVITY_BIT
 */
uint8_t ADXL345::get_int_inactivity_pin()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_MAP), INT_INACTIVITY_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set INACTIVITY interrupt pin.
 * @param pin Interrupt pin setting
 * @see get_int_data_ready_pin()
 * @see Register::INT_MAP
 * @see INT_INACTIVITY_BIT
 */
void ADXL345::set_int_inactivity_pin(uint8_t pin)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::INT_MAP), INT_INACTIVITY_BIT, pin);
}

/**
 * @brief Get FREE_FALL interrupt pin.
 * @return Interrupt pin setting
 * @see get_int_data_ready_pin()
 * @see Register::INT_MAP
 * @see INT_FREE_FALL_BIT
 */
uint8_t ADXL345::get_int_freefall_pin()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_MAP), INT_FREE_FALL_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set FREE_FALL interrupt pin.
 * @param pin Interrupt pin setting
 * @see get_int_data_ready_pin()
 * @see Register::INT_MAP
 * @see INT_FREE_FALL_BIT
 */
void ADXL345::set_int_freefall_pin(uint8_t pin)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::INT_MAP), INT_FREE_FALL_BIT, pin);
}

/**
 * @brief Get WATERMARK interrupt pin.
 * @return Interrupt pin setting
 * @see get_int_data_ready_pin()
 * @see Register::INT_MAP
 * @see INT_WATERMARK_BIT
 */
uint8_t ADXL345::get_int_watermark_pin()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_MAP), INT_WATERMARK_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set WATERMARK interrupt pin.
 * @param pin Interrupt pin setting
 * @see get_int_data_ready_pin()
 * @see Register::INT_MAP
 * @see INT_WATERMARK_BIT
 */
void ADXL345::set_int_watermark_pin(uint8_t pin)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::INT_MAP), INT_WATERMARK_BIT, pin);
}

/**
 * @brief Get OVERRUN interrupt pin.
 * @return Interrupt pin setting
 * @see get_int_data_ready_pin()
 * @see Register::INT_MAP
 * @see INT_OVERRUN_BIT
 */
uint8_t ADXL345::get_int_overrun_pin()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_MAP), INT_OVERRUN_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set OVERRUN interrupt pin.
 * @param pin Interrupt pin setting
 * @see get_int_data_ready_pin()
 * @see Register::INT_MAP
 * @see INT_OVERRUN_BIT
 */
void ADXL345::set_int_overrun_pin(uint8_t pin)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::INT_MAP), INT_OVERRUN_BIT, pin);
}

/**
 * @brief Get DATA_READY interrupt source flag.
 * Bits set to 1 in this register indicate that their respective functions have
 * triggered an event, whereas a value of 0 indicates that the corresponding
 * event has not occurred. The DATA_READY, watermark, and overrun bits are
 * always set if the corresponding events occur, regardless of the INT_ENABLE
 * register settings, and are cleared by reading data from the DATAX, DATAY, and
 * DATAZ registers. The DATA_READY and watermark bits may require multiple
 * reads, as indicated in the FIFO mode descriptions in the FIFO section. Other
 * bits, and the corresponding interrupts, are cleared by reading the INT_SOURCE
 * register.
 * @return Interrupt source flag
 * @see Register::INT_SOURCE
 * @see INT_DATA_READY_BIT
 */
uint8_t ADXL345::get_int_data_ready_source()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_SOURCE), INT_DATA_READY_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Get SINGLE_TAP interrupt source flag.
 * @return Interrupt source flag
 * @see Register::INT_SOURCE
 * @see INT_SINGLE_TAP_BIT
 */
uint8_t ADXL345::get_int_single_tap_source()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_SOURCE), INT_SINGLE_TAP_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Get DOUBLE_TAP interrupt source flag.
 * @return Interrupt source flag
 * @see Register::INT_SOURCE
 * @see INT_DOUBLE_TAP_BIT
 */
uint8_t ADXL345::get_int_double_tap_source()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_SOURCE), INT_DOUBLE_TAP_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Get ACTIVITY interrupt source flag.
 * @return Interrupt source flag
 * @see Register::INT_SOURCE
 * @see INT_ACTIVITY_BIT
 */
uint8_t ADXL345::get_int_activity_source()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_SOURCE), INT_ACTIVITY_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Get INACTIVITY interrupt source flag.
 * @return Interrupt source flag
 * @see Register::INT_SOURCE
 * @see INT_INACTIVITY_BIT
 */
uint8_t ADXL345::get_int_inactivity_source()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_SOURCE), INT_INACTIVITY_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Get FREE_FALL interrupt source flag.
 * @return Interrupt source flag
 * @see Register::INT_SOURCE
 * @see INT_FREE_FALL_BIT
 */
uint8_t ADXL345::get_int_freefall_source()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_SOURCE), INT_FREE_FALL_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Get WATERMARK interrupt source flag.
 * @return Interrupt source flag
 * @see Register::INT_SOURCE
 * @see INT_WATERMARK_BIT
 */
uint8_t ADXL345::get_int_watermark_source()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_SOURCE), INT_WATERMARK_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Get OVERRUN interrupt source flag.
 * @return Interrupt source flag
 * @see Register::INT_SOURCE
 * @see INT_OVERRUN_BIT
 */
uint8_t ADXL345::get_int_overrun_source()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::INT_SOURCE), INT_OVERRUN_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Get self-test force enabled.
 * A setting of 1 in the SELF_TEST bit applies a self-test force to the sensor,
 * causing a shift in the output data. A value of 0 disables the self-test
 * force.
 * @return Self-test force enabled setting
 * @see Register::DATA_FORMAT
 * @see FORMAT_SELFTEST_BIT
 */
uint8_t ADXL345::get_self_test_enabled()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::DATA_FORMAT), FORMAT_SELFTEST_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set self-test force enabled.
 * @param enabled New self-test force enabled setting
 * @see getSelfTestEnabled()
 * @see Register::DATA_FORMAT
 * @see FORMAT_SELFTEST_BIT
 */
void ADXL345::set_self_test_enabled(uint8_t enabled)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::DATA_FORMAT), FORMAT_SELFTEST_BIT, enabled);
}

/**
 * @brief Get SPI mode setting.
 * A value of 1 in the SPI bit sets the device to 3-wire SPI mode, and a value
 * of 0 sets the device to 4-wire SPI mode.
 * @return SPI mode setting
 * @see Register::DATA_FORMAT
 * @see FORMAT_SELFTEST_BIT
 */
uint8_t ADXL345::get_spi_mode()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::DATA_FORMAT), FORMAT_SPIMODE_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set SPI mode setting.
 * @param mode New SPI mode setting
 * @see get_spi_mode()
 * @see Register::DATA_FORMAT
 * @see FORMAT_SELFTEST_BIT
 */
void ADXL345::set_spi_mode(uint8_t mode)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::DATA_FORMAT), FORMAT_SPIMODE_BIT, mode);
}

/**
 * @brief Get interrupt mode setting.
 * A value of 0 in the INT_INVERT bit sets the interrupts to active high, and a
 * value of 1 sets the interrupts to active low.
 * @return Interrupt mode setting
 * @see Register::DATA_FORMAT
 * @see FORMAT_INTMODE_BIT
 */
uint8_t ADXL345::get_interrupt_mode()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::DATA_FORMAT), FORMAT_INTMODE_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set interrupt mode setting.
 * @param mode New interrupt mode setting
 * @see get_interrupt_mode()
 * @see Register::DATA_FORMAT
 * @see FORMAT_INTMODE_BIT
 */
void ADXL345::set_interrupt_mode(uint8_t mode)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::DATA_FORMAT), FORMAT_INTMODE_BIT, mode);
}

/**
 * @brief Get full resolution mode setting.
 * When this bit is set to a value of 1, the device is in full resolution mode,
 * where the output resolution increases with the g range set by the range bits
 * to maintain a 4 mg/LSB scale factor. When the FULL_RES bit is set to 0, the
 * device is in 10-bit mode, and the range bits determine the maximum g range
 * and scale factor.
 * @return Full resolution enabled setting
 * @see Register::DATA_FORMAT
 * @see FORMAT_FULL_RES_BIT
 */
uint8_t ADXL345::get_full_resolution()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::DATA_FORMAT), FORMAT_FULL_RES_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set full resolution mode setting.
 * @param resolution New full resolution enabled setting
 * @see get_full_resolution()
 * @see Register::DATA_FORMAT
 * @see FORMAT_FULL_RES_BIT
 */
void ADXL345::set_full_resolution(uint8_t resolution)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::DATA_FORMAT), FORMAT_FULL_RES_BIT, resolution);
}

/**
 * @brief Get data justification mode setting.
 * A setting of 1 in the justify bit selects left-justified (MSB) mode, and a
 * setting of 0 selects right-justified mode with sign extension.
 * @return Data justification mode
 * @see Register::DATA_FORMAT
 * @see FORMAT_JUSTIFY_BIT
 */
uint8_t ADXL345::get_data_justification()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::DATA_FORMAT), FORMAT_JUSTIFY_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set data justification mode setting.
 * @param justification New data justification mode
 * @see get_data_justification()
 * @see Register::DATA_FORMAT
 * @see FORMAT_JUSTIFY_BIT
 */
void ADXL345::set_data_justification(uint8_t justification)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::DATA_FORMAT), FORMAT_JUSTIFY_BIT, justification);
}

/**
 * @brief Get data range setting.
 * These bits set the g range as described in Table 21. (That is, 0x0 - 0x3 to
 * indicate 2g/4g/8g/16g respectively)
 * @return Range value (0x0 - 0x3 for 2g/4g/8g/16g)
 * @see Register::DATA_FORMAT
 * @see FORMAT_RANGE_BIT
 * @see FORMAT_RANGE_LENGTH
 */
ADXL345::Range ADXL345::get_range()
{
    I2Cdev::readBits(m_dev_addr, static_cast<uint8_t>(Register::DATA_FORMAT), FORMAT_RANGE_BIT, FORMAT_RANGE_LENGTH,
                     m_buffer);
    return static_cast<Range>(m_buffer[0]);
}

/**
 * @brief Set data range setting.
 * @param range Range value (0x0 - 0x3 for 2g/4g/8g/16g)
 * @see get_range()
 * @see Register::DATA_FORMAT
 * @see FORMAT_RANGE_BIT
 * @see FORMAT_RANGE_LENGTH
 */
void ADXL345::set_range(Range range)
{
    I2Cdev::writeBits(m_dev_addr, static_cast<uint8_t>(Register::DATA_FORMAT), FORMAT_RANGE_BIT, FORMAT_RANGE_LENGTH,
                      static_cast<uint8_t>(range));
}

/**
 * @brief Get 3-axis accleration measurements.
 * These six bytes (Register 0x32 to Register 0x37) are eight bits each and hold
 * the output data for each axis. Register 0x32 and Register 0x33 hold the
 * output data for the x-axis, Register 0x34 and Register 0x35 hold the output
 * data for the y-axis, and Register 0x36 and Register 0x37 hold the output data
 * for the z-axis. The output data is twos complement, with DATAx0 as the least
 * significant byte and DATAx1 as the most significant byte, where x represent
 * X, Y, or Z. The DATA_FORMAT register (Address 0x31) controls the format of
 * the data. It is recommended that a multiple-byte read of all registers be
 * performed to prevent a change in data between reads of sequential registers.
 *
 * The DATA_FORMAT register controls the presentation of data to Register 0x32
 * through Register 0x37. All data, except that for the +/-16 g range, must be
 * clipped to avoid rollover.
 *
 * @param x 16-bit signed integer container for X-axis acceleration
 * @param y 16-bit signed integer container for Y-axis acceleration
 * @param z 16-bit signed integer container for Z-axis acceleration
 * @see Register::DATAX0
 */
void ADXL345::get_acceleration(int16_t * x, int16_t * y, int16_t * z)
{
    I2Cdev::readBytes(m_dev_addr, static_cast<uint8_t>(Register::DATAX0), sizeof(int16_t) * 3, m_buffer);
    *x = (((int16_t)m_buffer[1]) << 8) | m_buffer[0];
    *y = (((int16_t)m_buffer[3]) << 8) | m_buffer[2];
    *z = (((int16_t)m_buffer[5]) << 8) | m_buffer[4];
}

/**
 * @brief Get X-axis accleration measurement.
 * @return 16-bit signed X-axis acceleration value
 * @see Register::DATAX0
 */
int16_t ADXL345::get_acceleration_x()
{
    I2Cdev::readBytes(m_dev_addr, static_cast<uint8_t>(Register::DATAX0), sizeof(int16_t), m_buffer);
    return (((int16_t)m_buffer[1]) << 8) | m_buffer[0];
}

/**
 * @brief Get Y-axis accleration measurement.
 * @return 16-bit signed Y-axis acceleration value
 * @see Register::DATAY0
 */
int16_t ADXL345::get_acceleration_y()
{
    I2Cdev::readBytes(m_dev_addr, static_cast<uint8_t>(Register::DATAY0), sizeof(int16_t), m_buffer);
    return (((int16_t)m_buffer[1]) << 8) | m_buffer[0];
}

/**
 * @brief Get Z-axis accleration measurement.
 * @return 16-bit signed Z-axis acceleration value
 * @see Register::DATAZ0
 */
int16_t ADXL345::get_acceleration_z()
{
    I2Cdev::readBytes(m_dev_addr, static_cast<uint8_t>(Register::DATAZ0), sizeof(int16_t), m_buffer);
    return (((int16_t)m_buffer[1]) << 8) | m_buffer[0];
}

/**
 * @brief Get FIFO mode.
 * These bits set the FIFO mode, as described in Table 22. That is:
 *
 * 0x0 = Bypass (FIFO is bypassed.)
 *
 * 0x1 = FIFO (FIFO collects up to 32 values and then stops collecting data,
 *       collecting new data only when FIFO is not full.)
 *
 * 0x2 = Stream (FIFO holds the last 32 data values. When FIFO is full, the
 *       oldest data is overwritten with newer data.)
 *
 * 0x3 = Trigger (When triggered by the trigger bit, FIFO holds the last data
 *       samples before the trigger event and then continues to collect data
 *       until full. New data is collected only when FIFO is not full.)
 *
 * @return Curent FIFO mode
 * @see Register::FIFO_CTL
 * @see FIFO_MODE_BIT
 * @see FIFO_MODE_LENGTH
 */
ADXL345::FifoMode ADXL345::get_fifo_mode()
{
    I2Cdev::readBits(m_dev_addr, static_cast<uint8_t>(Register::FIFO_CTL), FIFO_MODE_BIT, FIFO_MODE_LENGTH, m_buffer);
    return static_cast<FifoMode>(m_buffer[0]);
}

/**
 * @brief Set FIFO mode.
 * @param mode New FIFO mode
 * @see get_fifo_mode()
 * @see Register::FIFO_CTL
 * @see FIFO_MODE_BIT
 * @see FIFO_MODE_LENGTH
 */
void ADXL345::set_fifo_mode(FifoMode mode)
{
    I2Cdev::writeBits(m_dev_addr, static_cast<uint8_t>(Register::FIFO_CTL), FIFO_MODE_BIT, FIFO_MODE_LENGTH,
                      static_cast<uint8_t>(mode));
}

/**
 * @brief Get FIFO trigger interrupt setting.
 * A value of 0 in the trigger bit links the trigger event of trigger mode to
 * INT1, and a value of 1 links the trigger event to INT2.
 * @return Current FIFO trigger interrupt setting
 * @see Register::FIFO_CTL
 * @see FIFO_TRIGGER_BIT
 */
uint8_t ADXL345::get_fifo_trigger_interrupt_pin()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::FIFO_CTL), FIFO_TRIGGER_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set FIFO trigger interrupt pin setting.
 * @param interrupt New FIFO trigger interrupt pin setting
 * @see Register::FIFO_CTL
 * @see FIFO_TRIGGER_BIT
 */
void ADXL345::set_fifo_trigger_interrupt_pin(uint8_t interrupt)
{
    I2Cdev::writeBit(m_dev_addr, static_cast<uint8_t>(Register::FIFO_CTL), FIFO_TRIGGER_BIT, interrupt);
}

/**
 * @brief Get FIFO samples setting.
 * The function of these bits depends on the FIFO mode selected (see Table 23).
 * Entering a value of 0 in the samples bits immediately sets the watermark
 * status bit in the INT_SOURCE register, regardless of which FIFO mode is
 * selected. Undesirable operation may occur if a value of 0 is used for the
 * samples bits when trigger mode is used.
 *
 * MODE    | EFFECT
 * --------+-------------------------------------------------------------------
 * Bypass  | None.
 * FIFO    | FIFO entries needed to trigger a watermark interrupt.
 * Stream  | FIFO entries needed to trigger a watermark interrupt.
 * Trigger | Samples are retained in the FIFO buffer before a trigger event.
 *
 * @return Current FIFO samples setting
 * @see Register::FIFO_CTL
 * @see FIFO_SAMPLES_BIT
 * @see FIFO_SAMPLES_LENGTH
 */
uint8_t ADXL345::get_fifo_samples()
{
    I2Cdev::readBits(m_dev_addr, static_cast<uint8_t>(Register::FIFO_CTL), FIFO_SAMPLES_BIT, FIFO_SAMPLES_LENGTH,
                     m_buffer);
    return m_buffer[0];
}

/**
 * @brief Set FIFO samples setting.
 * @param size New FIFO samples setting (impact depends on FIFO mode setting)
 * @see get_fifo_samples()
 * @see get_fifo_mode()
 * @see Register::FIFO_CTL
 * @see FIFO_SAMPLES_BIT
 * @see FIFO_SAMPLES_LENGTH
 */
void ADXL345::set_fifo_samples(uint8_t size)
{
    I2Cdev::writeBits(m_dev_addr, static_cast<uint8_t>(Register::FIFO_CTL), FIFO_SAMPLES_BIT, FIFO_SAMPLES_LENGTH,
                      size);
}

/**
 * @brief Get FIFO trigger occurred status.
 * A 1 in the FIFO_TRIG bit corresponds to a trigger event occurring, and a 0
 * means that a FIFO trigger event has not occurred.
 * @return FIFO trigger occurred status
 * @see Register::FIFO_STATUS
 * @see FIFOSTAT_TRIGGER_BIT
 */
bool ADXL345::get_fifo_trigger_occurred()
{
    I2Cdev::readBit(m_dev_addr, static_cast<uint8_t>(Register::FIFO_STATUS), FIFOSTAT_TRIGGER_BIT, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Get FIFO length.
 * These bits report how many data values are stored in FIFO. Access to collect
 * the data from FIFO is provided through the DATAX, DATAY, and DATAZ registers.
 * FIFO reads must be done in burst or multiple-byte mode because each FIFO
 * level is cleared after any read (single- or multiple-byte) of FIFO. FIFO
 * stores a maximum of 32 entries, which equates to a maximum of 33 entries
 * available at any given time because an additional entry is available at the
 * output filter of the I2Cdev::
 * @return Current FIFO length
 * @see Register::FIFO_STATUS
 * @see FIFOSTAT_LENGTH_BIT
 * @see FIFOSTAT_LENGTH_LENGTH
 */
uint8_t ADXL345::get_fifo_length()
{
    I2Cdev::readBits(m_dev_addr, static_cast<uint8_t>(Register::FIFO_STATUS), FIFOSTAT_LENGTH_BIT,
                     FIFOSTAT_LENGTH_LENGTH, m_buffer);
    return m_buffer[0];
}

/**
 * @brief Clear calibration data.
 * This sets all calibration offsets to 0 and all calibration scales to 1.
 */
void ADXL345::clear_calibration()
{
    m_cal_offset[0] = 0.0f;
    m_cal_offset[1] = 0.0f;
    m_cal_offset[2] = 0.0f;
    m_cal_scale[0] = 1.0f;
    m_cal_scale[1] = 1.0f;
    m_cal_scale[2] = 1.0f;
}

/**
 * @brief Set calibration offsets.
 * Calibration offsets are added to raw acceleration values after scaling. This
 * can be used to correct for sensor bias.
 *
 * @param x X-axis offset
 * @param y Y-axis offset
 * @param z Z-axis offset
 */
void ADXL345::set_calibration_offsets(float x, float y, float z)
{
    m_cal_offset[0] = x;
    m_cal_offset[1] = y;
    m_cal_offset[2] = z;
}

/**
 * @brief Set calibration scales.
 * Calibration scales are multiplied with raw acceleration values after adding offsets.
 * This can be used to correct for sensor sensitivity variations.
 *
 * @param x X-axis scale
 * @param y Y-axis scale
 * @param z Z-axis scale
 */
void ADXL345::set_calibration_scales(float x, float y, float z)
{
    m_cal_scale[0] = x;
    m_cal_scale[1] = y;
    m_cal_scale[2] = z;
}

/**
 * @brief Get calibration offset for a specific axis.
 * @param index Axis index (0 for X, 1 for Y, 2 for Z)
 * @return Calibration offset for the specified axis, or 0.0f if index is out of range
 */
float ADXL345::get_calibration_offset(uint8_t index) const
{
    return (index < 3) ? m_cal_offset[index] : 0.0f;
}

/**
 * @brief Get calibration scale for a specific axis.
 * @param index Axis index (0 for X, 1 for Y, 2 for Z)
 * @return Calibration scale for the specified axis, or 1.0f if index is out of range
 */
float ADXL345::get_calibration_scale(uint8_t index) const
{
    return (index < 3) ? m_cal_scale[index] : 1.0f;
}

/**
 * Perform offset calibration. Assumes the sensor is placed flat on a level surface (Z-axis = 1g).
 * Calculates offsets for X, Y, and Z and writes them to the OFS registers.
 * @param samples Number of samples to take for averaging (default 100)
 */
void ADXL345::calibrate()
{
    clear_calibration();

    set_offset_x(0);
    set_offset_y(0);
    set_offset_z(0);

    int16_t x, y, z;
    int16_t minX = 32000, maxX = -32000;
    int16_t minY = 32000, maxY = -32000;
    int16_t minZ = 32000, maxZ = -32000;

    const int16_t SPAN_THRESHOLD = 500;
    bool is_calibrating = true;

    while (is_calibrating)
    {
        get_acceleration(&x, &y, &z);

        if (y == 0 && z == 0)
        {
            if (x < minX)
                minX = x;
            if (x > maxX)
                maxX = x;
        }
        if (x == 0 && z == 0)
        {
            if (y < minY)
                minY = y;
            if (y > maxY)
                maxY = y;
        }
        if (x == 0 && y == 0)
        {
            if (z < minZ)
                minZ = z;
            if (z > maxZ)
                maxZ = z;
        }

        bool xReady = (maxX - minX) > SPAN_THRESHOLD;
        bool yReady = (maxY - minY) > SPAN_THRESHOLD;
        bool zReady = (maxZ - minZ) > SPAN_THRESHOLD;

        if (xReady && yReady && zReady)
            is_calibrating = false;

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    float offX = (maxX + minX) / (256.0f * 2.0f);
    float offY = (maxY + minY) / (256.0f * 2.0f);
    float offZ = (maxZ + minZ) / (256.0f * 2.0f);

    set_calibration_offsets(offX, offY, offZ);

    float target = 256.0f;
    float semiRangeX = (maxX - minX) / 2.0f;
    float semiRangeY = (maxY - minY) / 2.0f;
    float semiRangeZ = (maxZ - minZ) / 2.0f;

    if (semiRangeX > 0)
        m_cal_scale[0] = target / semiRangeX;
    if (semiRangeY > 0)
        m_cal_scale[1] = target / semiRangeY;
    if (semiRangeZ > 0)
        m_cal_scale[2] = target / semiRangeZ;
}

/**
 * @brief Save calibration data to NVS.
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t ADXL345::save_calibration_to_nvs()
{
    float data[6] = {m_cal_offset[0], m_cal_offset[1], m_cal_offset[2], m_cal_scale[0], m_cal_scale[1], m_cal_scale[2]};
    esp_err_t err = NVSManager::save_blob(NVS_NS, NVS_KEY_BLOB, data, sizeof(data));

    if (err == ESP_OK)
        ESP_LOGI(TAG, "Accelerometer calibration saved to NVS.");

    return err;
}

/**
 * @brief Load calibration data from NVS.
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t ADXL345::load_calibration_from_nvs()
{
    float data[6];
    size_t req_size = sizeof(data);

    esp_err_t err = NVSManager::load_blob(NVS_NS, NVS_KEY_BLOB, data, &req_size);
    if (err == ESP_OK)
    {
        if (req_size == sizeof(data))
        {
            m_cal_offset[0] = data[0];
            m_cal_offset[1] = data[1];
            m_cal_offset[2] = data[2];
            m_cal_scale[0] = data[3];
            m_cal_scale[1] = data[4];
            m_cal_scale[2] = data[5];
            ESP_LOGI(TAG, "Loaded accelerometer calibration: Off[%.2f, %.2f, %.2f] Scl[%.2f, %.2f, %.2f]",
                     m_cal_offset[0], m_cal_offset[1], m_cal_offset[2], m_cal_scale[0], m_cal_scale[1], m_cal_scale[2]);
            return ESP_OK;
        }
        else
        {
            ESP_LOGE(TAG, "NVS Blob size mismatch! Expected %zu, got %zu", sizeof(data), req_size);
            return ESP_ERR_INVALID_SIZE;
        }
    }

    if (err == ESP_ERR_NOT_FOUND)
        ESP_LOGW(TAG, "Accelerometer calibration not found in NVS.");

    return err;
}

/**
 * @brief Check if calibration data is set (i.e., not default).
 * @return True if calibration data is set, false if it is still at default values
 */
bool ADXL345::is_calibrated() const
{
    return !(m_cal_scale[0] == 1.0f &&   //
             m_cal_scale[1] == 1.0f &&   //
             m_cal_scale[2] == 1.0f &&   //
             m_cal_offset[0] == 0.0f &&  //
             m_cal_offset[1] == 0.0f &&  //
             m_cal_offset[2] == 0.0f);
}
