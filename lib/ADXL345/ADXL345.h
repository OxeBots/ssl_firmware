/**
 * @file ADXL345.h
 * @brief Based on Analog Devices ADXL345 datasheet rev. C, 5/2011
 * 7/31/2011 by Jeff Rowberg <jeff@rowberg.net>
 * Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
 * 1/20/2013 by Chris Howells <chris@howells.net>
 * Updated to support ESP-IDF v4.0 and later
 * * DISCLAIMER: This code is based on the I2Cdev library collection but has been modified and is not equal to the original.
 */


#ifndef _ADXL345_H_
#define _ADXL345_H_

#include <I2Cdev.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "NVSManager.h"

class ADXL345
{
   public:
    static constexpr uint8_t DEFAULT_ADDRESS = 0x53;   // alt address pin low (GND)
    static constexpr uint8_t ADDRESS_ALT_HIGH = 0x1D;  // alt address pin high (VCC)

    enum class Register : uint8_t
    {
        DEVID = 0x00,
        RESERVED1 = 0x01,
        THRESH_TAP = 0x1D,
        OFSX = 0x1E,
        OFSY = 0x1F,
        OFSZ = 0x20,
        DUR = 0x21,
        LATENT = 0x22,
        WINDOW = 0x23,
        THRESH_ACT = 0x24,
        THRESH_INACT = 0x25,
        TIME_INACT = 0x26,
        ACT_INACT_CTL = 0x27,
        THRESH_FF = 0x28,
        TIME_FF = 0x29,
        TAP_AXES = 0x2A,
        ACT_TAP_STATUS = 0x2B,
        BW_RATE = 0x2C,
        POWER_CTL = 0x2D,
        INT_ENABLE = 0x2E,
        INT_MAP = 0x2F,
        INT_SOURCE = 0x30,
        DATA_FORMAT = 0x31,
        DATAX0 = 0x32,
        DATAX1 = 0x33,
        DATAY0 = 0x34,
        DATAY1 = 0x35,
        DATAZ0 = 0x36,
        DATAZ1 = 0x37,
        FIFO_CTL = 0x38,
        FIFO_STATUS = 0x39
    };

    enum class Range : uint8_t
    {
        RNG_2G = 0x00,
        RNG_4G = 0x01,
        RNG_8G = 0x10,
        RNG_16G = 0x11
    };

    enum class FifoMode : uint8_t
    {
        BYPASS = 0x00,
        FIFO = 0x01,
        STREAM = 0x10,
        TRIGGER = 0x11
    };

    // Constant Bit positions
    static constexpr uint8_t AIC_ACT_AC_BIT = 7;
    static constexpr uint8_t AIC_ACT_X_BIT = 6;
    static constexpr uint8_t AIC_ACT_Y_BIT = 5;
    static constexpr uint8_t AIC_ACT_Z_BIT = 4;
    static constexpr uint8_t AIC_INACT_AC_BIT = 3;
    static constexpr uint8_t AIC_INACT_X_BIT = 2;
    static constexpr uint8_t AIC_INACT_Y_BIT = 1;
    static constexpr uint8_t AIC_INACT_Z_BIT = 0;
    static constexpr uint8_t TAPAXIS_SUP_BIT = 3;
    static constexpr uint8_t TAPAXIS_X_BIT = 2;
    static constexpr uint8_t TAPAXIS_Y_BIT = 1;
    static constexpr uint8_t TAPAXIS_Z_BIT = 0;
    static constexpr uint8_t TAPSTAT_ACTX_BIT = 6;
    static constexpr uint8_t TAPSTAT_ACTY_BIT = 5;
    static constexpr uint8_t TAPSTAT_ACTZ_BIT = 4;
    static constexpr uint8_t TAPSTAT_ASLEEP_BIT = 3;
    static constexpr uint8_t TAPSTAT_TAPX_BIT = 2;
    static constexpr uint8_t TAPSTAT_TAPY_BIT = 1;
    static constexpr uint8_t TAPSTAT_TAPZ_BIT = 0;
    static constexpr uint8_t BW_LOWPOWER_BIT = 4;
    static constexpr uint8_t BW_RATE_BIT = 3;
    static constexpr uint8_t BW_RATE_LENGTH = 4;
    static constexpr uint8_t PCTL_LINK_BIT = 5;
    static constexpr uint8_t PCTL_AUTOSLEEP_BIT = 4;
    static constexpr uint8_t PCTL_MEASURE_BIT = 3;
    static constexpr uint8_t PCTL_SLEEP_BIT = 2;
    static constexpr uint8_t PCTL_WAKEUP_BIT = 1;
    static constexpr uint8_t PCTL_WAKEUP_LENGTH = 2;
    static constexpr uint8_t INT_DATA_READY_BIT = 7;
    static constexpr uint8_t INT_SINGLE_TAP_BIT = 6;
    static constexpr uint8_t INT_DOUBLE_TAP_BIT = 5;
    static constexpr uint8_t INT_ACTIVITY_BIT = 4;
    static constexpr uint8_t INT_INACTIVITY_BIT = 3;
    static constexpr uint8_t INT_FREE_FALL_BIT = 2;
    static constexpr uint8_t INT_WATERMARK_BIT = 1;
    static constexpr uint8_t INT_OVERRUN_BIT = 0;
    static constexpr uint8_t FORMAT_SELFTEST_BIT = 7;
    static constexpr uint8_t FORMAT_SPIMODE_BIT = 6;
    static constexpr uint8_t FORMAT_INTMODE_BIT = 5;
    static constexpr uint8_t FORMAT_FULL_RES_BIT = 3;
    static constexpr uint8_t FORMAT_JUSTIFY_BIT = 2;
    static constexpr uint8_t FORMAT_RANGE_BIT = 1;
    static constexpr uint8_t FORMAT_RANGE_LENGTH = 2;
    static constexpr uint8_t FIFO_MODE_BIT = 7;
    static constexpr uint8_t FIFO_MODE_LENGTH = 2;
    static constexpr uint8_t FIFO_TRIGGER_BIT = 5;
    static constexpr uint8_t FIFO_SAMPLES_BIT = 4;
    static constexpr uint8_t FIFO_SAMPLES_LENGTH = 5;
    static constexpr uint8_t FIFOSTAT_TRIGGER_BIT = 7;
    static constexpr uint8_t FIFOSTAT_LENGTH_BIT = 5;
    static constexpr uint8_t FIFOSTAT_LENGTH_LENGTH = 6;

    ADXL345(uint8_t address = DEFAULT_ADDRESS);

    void init();
    bool test_connection();

    // Calibration
    void calibrate();
    void clear_calibration();
    void set_calibration_offsets(float x, float y, float z);
    void set_calibration_scales(float x, float y, float z);
    float get_calibration_offset(uint8_t index) const;
    float get_calibration_scale(uint8_t index) const;

    // NVS Persistence
    esp_err_t save_calibration_to_nvs();
    esp_err_t load_calibration_from_nvs();
    bool is_calibrated() const;

    // DEVID register
    uint8_t get_device_id();

    // THRESH_TAP register
    uint8_t get_tap_threshold();
    void set_tap_threshold(uint8_t threshold);

    // OFS* registers
    void get_offset(int8_t * x, int8_t * y, int8_t * z);
    void set_offset(int8_t x, int8_t y, int8_t z);
    int8_t get_offset_x();
    void set_offset_x(int8_t x);
    int8_t get_offset_y();
    void set_offset_y(int8_t y);
    int8_t get_offset_z();
    void set_offset_z(int8_t z);

    // DUR register
    uint8_t get_tap_duration();
    void set_tap_duration(uint8_t duration);

    // LATENT register
    uint8_t get_double_tap_latency();
    void set_double_tap_latency(uint8_t latency);

    // WINDOW register
    uint8_t get_double_tap_window();
    void set_double_tap_window(uint8_t window);

    // THRESH_ACT register
    uint8_t get_activity_threshold();
    void set_activity_threshold(uint8_t threshold);

    // THRESH_INACT register
    uint8_t get_inactivity_threshold();
    void set_inactivity_threshold(uint8_t threshold);

    // TIME_INACT register
    uint8_t get_inactivity_time();
    void set_inactivity_time(uint8_t time);

    // ACT_INACT_CTL register
    bool get_activity_ac();
    void set_activity_ac(bool enabled);
    bool get_activity_x_enabled();
    void set_activity_x_enabled(bool enabled);
    bool get_activity_y_enabled();
    void set_activity_y_enabled(bool enabled);
    bool get_activity_z_enabled();
    void set_activity_z_enabled(bool enabled);
    bool get_inactivity_ac();
    void set_inactivity_ac(bool enabled);
    bool get_inactivity_x_enabled();
    void set_inactivity_x_enabled(bool enabled);
    bool get_inactivity_y_enabled();
    void set_inactivity_y_enabled(bool enabled);
    bool get_inactivity_z_enabled();
    void set_inactivity_z_enabled(bool enabled);

    // THRESH_FF register
    uint8_t get_freefall_threshold();
    void set_freefall_threshold(uint8_t threshold);

    // TIME_FF register
    uint8_t get_freefall_time();
    void set_freefall_time(uint8_t time);

    // TAP_AXES register
    bool get_tap_axis_suppress();
    void set_tap_axis_suppress(bool enabled);
    bool get_tap_axis_x_enabled();
    void set_tap_axis_x_enabled(bool enabled);
    bool get_tap_axis_y_enabled();
    void set_tap_axis_y_enabled(bool enabled);
    bool get_tap_axis_z_enabled();
    void set_tap_axis_z_enabled(bool enabled);

    // ACT_TAP_STATUS register
    bool get_activity_source_x();
    bool get_activity_source_y();
    bool get_activity_source_z();
    bool get_asleep();
    bool get_tap_source_x();
    bool get_tap_source_y();
    bool get_tap_source_z();

    // BW_RATE register
    bool get_low_power_enabled();
    void set_low_power_enabled(bool enabled);
    uint8_t get_rate();
    void set_rate(uint8_t rate);

    // POWER_CTL register
    bool get_link_enabled();
    void set_link_enabled(bool enabled);
    bool get_auto_sleep_enabled();
    void set_auto_sleep_enabled(bool enabled);
    bool get_measure_enabled();
    void set_measure_enabled(bool enabled);
    bool get_sleep_enabled();
    void set_sleep_enabled(bool enabled);
    uint8_t get_wakeup_frequency();
    void set_wakeup_frequency(uint8_t frequency);

    // INT_ENABLE register
    bool get_int_data_ready_enabled();
    void set_int_data_ready_enabled(bool enabled);
    bool get_int_single_tap_enabled();
    void set_int_single_tap_enabled(bool enabled);
    bool get_int_double_tap_enabled();
    void set_int_double_tap_enabled(bool enabled);
    bool get_int_activity_enabled();
    void set_int_activity_enabled(bool enabled);
    bool get_int_inactivity_enabled();
    void set_int_inactivity_enabled(bool enabled);
    bool get_int_freefall_enabled();
    void set_int_freefall_enabled(bool enabled);
    bool get_int_watermark_enabled();
    void set_int_watermark_enabled(bool enabled);
    bool get_int_overrun_enabled();
    void set_int_overrun_enabled(bool enabled);

    // INT_MAP register
    uint8_t get_int_data_ready_pin();
    void set_int_data_ready_pin(uint8_t pin);
    uint8_t get_int_single_tap_pin();
    void set_int_single_tap_pin(uint8_t pin);
    uint8_t get_int_double_tap_pin();
    void set_int_double_tap_pin(uint8_t pin);
    uint8_t get_int_activity_pin();
    void set_int_activity_pin(uint8_t pin);
    uint8_t get_int_inactivity_pin();
    void set_int_inactivity_pin(uint8_t pin);
    uint8_t get_int_freefall_pin();
    void set_int_freefall_pin(uint8_t pin);
    uint8_t get_int_watermark_pin();
    void set_int_watermark_pin(uint8_t pin);
    uint8_t get_int_overrun_pin();
    void set_int_overrun_pin(uint8_t pin);

    // INT_SOURCE register
    uint8_t get_int_data_ready_source();
    uint8_t get_int_single_tap_source();
    uint8_t get_int_double_tap_source();
    uint8_t get_int_activity_source();
    uint8_t get_int_inactivity_source();
    uint8_t get_int_freefall_source();
    uint8_t get_int_watermark_source();
    uint8_t get_int_overrun_source();

    // DATA_FORMAT register
    uint8_t get_self_test_enabled();
    void set_self_test_enabled(uint8_t enabled);
    uint8_t get_spi_mode();
    void set_spi_mode(uint8_t mode);
    uint8_t get_interrupt_mode();
    void set_interrupt_mode(uint8_t mode);
    uint8_t get_full_resolution();
    void set_full_resolution(uint8_t resolution);
    uint8_t get_data_justification();
    void set_data_justification(uint8_t justification);
    Range get_range();
    void set_range(Range range);

    // DATA* registers
    void get_acceleration(int16_t * x, int16_t * y, int16_t * z);
    int16_t get_acceleration_x();
    int16_t get_acceleration_y();
    int16_t get_acceleration_z();

    // FIFO_CTL register
    FifoMode get_fifo_mode();
    void set_fifo_mode(FifoMode mode);
    uint8_t get_fifo_trigger_interrupt_pin();
    void set_fifo_trigger_interrupt_pin(uint8_t interrupt);
    uint8_t get_fifo_samples();
    void set_fifo_samples(uint8_t size);

    // FIFO_STATUS register
    bool get_fifo_trigger_occurred();
    uint8_t get_fifo_length();

   private:
    uint8_t m_dev_addr;
    uint8_t m_buffer[6] = {0};
    float m_cal_offset[3] = {0.f, 0.f, 0.f};
    float m_cal_scale[3] = {1.f, 1.f, 1.f};
};

#endif /* _ADXL345_H_ */
