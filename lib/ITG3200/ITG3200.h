#ifndef _ITG3200_H_
#define _ITG3200_H_

#include <I2Cdev.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "NVSManager.h"

class ITG3200
{
   public:
    static constexpr uint8_t DEFAULT_ADDRESS = 0x68;   // Address pin low (GND)
    static constexpr uint8_t ADDRESS_ALT_HIGH = 0x69;  // Address pin high (VCC)

    enum class Register : uint8_t
    {
        WHO_AM_I = 0x00,
        SMPLRT_DIV = 0x15,
        DLPF_FS = 0x16,
        INT_CFG = 0x17,
        INT_STATUS = 0x1A,
        TEMP_OUT_H = 0x1B,
        TEMP_OUT_L = 0x1C,
        GYRO_XOUT_H = 0x1D,
        GYRO_XOUT_L = 0x1E,
        GYRO_YOUT_H = 0x1F,
        GYRO_YOUT_L = 0x20,
        GYRO_ZOUT_H = 0x21,
        GYRO_ZOUT_L = 0x22,
        PWR_MGM = 0x3E
    };

    enum class FullScaleRange : uint8_t
    {
        FS_2000 = 0x03
    };

    enum class Bandwidth : uint8_t
    {
        BW_256 = 0x00,
        BW_188 = 0x01,
        BW_98 = 0x02,
        BW_42 = 0x03,
        BW_20 = 0x04,
        BW_10 = 0x05,
        BW_5 = 0x06
    };

    enum class ClockSource : uint8_t
    {
        INTERNAL = 0x00,
        PLL_XGYRO = 0x01,
        PLL_YGYRO = 0x02,
        PLL_ZGYRO = 0x03,
        PLL_EXT32K = 0x04,
        PLL_EXT19M = 0x05
    };

    // Bit Masks and Positions
    static constexpr uint8_t DEVID_BIT = 6;
    static constexpr uint8_t DEVID_LENGTH = 6;
    static constexpr uint8_t DF_FS_SEL_BIT = 4;
    static constexpr uint8_t DF_FS_SEL_LENGTH = 2;
    static constexpr uint8_t DF_DLPF_CFG_BIT = 2;
    static constexpr uint8_t DF_DLPF_CFG_LENGTH = 3;
    static constexpr uint8_t INTCFG_ACTL_BIT = 7;
    static constexpr uint8_t INTCFG_OPEN_BIT = 6;
    static constexpr uint8_t INTCFG_LATCH_INT_EN_BIT = 5;
    static constexpr uint8_t INTCFG_INT_ANYRD_2CLEAR_BIT = 4;
    static constexpr uint8_t INTCFG_ITG_RDY_EN_BIT = 2;
    static constexpr uint8_t INTCFG_RAW_RDY_EN_BIT = 0;
    static constexpr uint8_t INTSTAT_ITG_RDY_BIT = 2;
    static constexpr uint8_t INTSTAT_RAW_DATA_READY_BIT = 0;
    static constexpr uint8_t PWR_H_RESET_BIT = 7;
    static constexpr uint8_t PWR_SLEEP_BIT = 6;
    static constexpr uint8_t PWR_STBY_XG_BIT = 5;
    static constexpr uint8_t PWR_STBY_YG_BIT = 4;
    static constexpr uint8_t PWR_STBY_ZG_BIT = 3;
    static constexpr uint8_t PWR_CLK_SEL_BIT = 2;
    static constexpr uint8_t PWR_CLK_SEL_LENGTH = 3;

    ITG3200(uint8_t address = DEFAULT_ADDRESS);

    void init();
    bool test_connection();

    // Calibration
    void calibrate(uint16_t samples = 1000);
    void set_offsets(int16_t x, int16_t y, int16_t z);
    void get_offsets(int16_t * x, int16_t * y, int16_t * z) const;

    // NVS Persistence
    esp_err_t save_calibration_to_nvs();
    esp_err_t load_calibration_from_nvs();
    bool is_calibrated() const;

    // WHO_AM_I register
    uint8_t get_device_id();
    void set_device_id(uint8_t id);

    // SMPLRT_DIV register
    uint8_t get_rate();
    void set_rate(uint8_t rate);

    // DLPF_FS register
    FullScaleRange get_full_scale_range();
    void set_full_scale_range(FullScaleRange range);
    Bandwidth get_dlpf_bandwidth();
    void set_dlpf_bandwidth(Bandwidth bandwidth);

    // INT_CFG register
    bool get_interrupt_mode();
    void set_interrupt_mode(bool mode);
    bool get_interrupt_drive();
    void set_interrupt_drive(bool drive);
    bool get_interrupt_latch();
    void set_interrupt_latch(bool latch);
    bool get_interrupt_latch_clear();
    void set_interrupt_latch_clear(bool clear);
    bool get_int_device_ready_enabled();
    void set_int_device_ready_enabled(bool enabled);
    bool get_int_data_ready_enabled();
    void set_int_data_ready_enabled(bool enabled);

    // INT_STATUS register
    bool get_int_device_ready_status();
    bool get_int_data_ready_status();

    // TEMP_OUT_* registers
    int16_t get_temperature();

    // GYRO_*OUT_* registers
    void get_rotation(int16_t * x, int16_t * y, int16_t * z);
    int16_t get_rotation_x();
    int16_t get_rotation_y();
    int16_t get_rotation_z();

    // PWR_MGM register
    void reset();
    bool get_sleep_enabled();
    void set_sleep_enabled(bool enabled);
    bool get_standby_x_enabled();
    void set_standby_x_enabled(bool enabled);
    bool get_standby_y_enabled();
    void set_standby_y_enabled(bool enabled);
    bool get_standby_z_enabled();
    void set_standby_z_enabled(bool enabled);
    ClockSource get_clock_source();
    void set_clock_source(ClockSource source);

   private:
    uint8_t m_dev_addr;
    uint8_t m_buffer[6] = {0};

    // Offsets
    int16_t m_x_offset = 0;
    int16_t m_y_offset = 0;
    int16_t m_z_offset = 0;
};

#endif /* _ITG3200_H_ */
