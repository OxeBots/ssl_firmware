/**
 * @file QMC5883L.h
 * @brief Driver for the QMC5883L I2C magnetometer sensor.
 */
#ifndef _QMC5883L_H_
#define _QMC5883L_H_

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <string.h>

#include "I2Cdev.h"
#include "NVSManager.h"

class QMC5883L
{
   public:
    static constexpr uint8_t DEFAULT_ADDRESS = 0x0D;

    enum class Register : uint8_t
    {
        DATAX_L = 0x00,
        DATAX_H = 0x01,
        DATAY_L = 0x02,
        DATAY_H = 0x03,
        DATAZ_L = 0x04,
        DATAZ_H = 0x05,
        STATUS = 0x06,
        TEMP_L = 0x07,
        TEMP_H = 0x08,
        CONTROL_1 = 0x09,
        CONTROL_2 = 0x0A,
        PERIOD = 0x0B,
        CHIP_ID = 0x0D
    };

    enum class Mode : uint8_t
    {
        STANDBY = 0x00,
        CONTINUOUS = 0x01
    };

    enum class OutputDataRate : uint8_t
    {
        ODR_10HZ = 0x00,
        ODR_50HZ = 0x04,
        ODR_100HZ = 0x08,
        ODR_200HZ = 0x0C
    };

    enum class Range : uint8_t
    {
        RNG_2G = 0x00,
        RNG_8G = 0x10
    };

    enum class Oversampling : uint8_t
    {
        OSR_512 = 0x00,
        OSR_256 = 0x40,
        OSR_128 = 0x80,
        OSR_64 = 0xC0
    };

    QMC5883L(uint8_t address = DEFAULT_ADDRESS);

    void init();
    bool test_connection();
    void set_addr(uint8_t hex);
    void set_mode(Mode mode, OutputDataRate odr, Range rng, Oversampling osr);
    void set_reset();

    void set_range(Range rng);
    Range get_range() const;

    void set_magnetic_declination(int degrees, uint8_t minutes);
    void set_smoothing(uint8_t steps, bool adv);
    void clear_calibration();
    void set_calibration_offsets(float x_offset, float y_offset, float z_offset);
    void set_calibration_scales(float x_scale, float y_scale, float z_scale);
    float get_calibration_offset(uint8_t index) const;
    float get_calibration_scale(uint8_t index) const;

    void read();

    int16_t inline get_x() const { return get_axis(0); }
    int16_t inline get_y() const { return get_axis(1); }
    int16_t inline get_z() const { return get_axis(2); }

    void get_orientation(int16_t * x, int16_t * y, int16_t * z);
    int get_azimuth() const;
    uint8_t get_bearing(int azimuth) const;
    void get_direction(char * myArray, int azimuth) const;
    uint8_t get_chip_id();

    // ========== CALIBRATION ROUTINES ==========

    void start_calibration_mode(uint32_t seconds);
    bool calibration_update();
    void stop_calibration_mode();
    bool is_calibrated() const;

    // ========== NVS PERSISTENCE ==========

    esp_err_t load_calibration_from_nvs();
    esp_err_t save_calibration_to_nvs();

   private:
    uint8_t m_dev_addr;
    uint8_t m_buffer[6] = {0};

    // Settings
    float m_magnetic_declination_degrees = 0.0f;
    bool m_smooth_use = false;
    uint8_t m_smooth_steps = 5;
    bool m_smooth_advanced = false;

    Mode m_mode = Mode::CONTINUOUS;
    OutputDataRate m_odr = OutputDataRate::ODR_200HZ;
    Range m_rng = Range::RNG_8G;
    Oversampling m_osr = Oversampling::OSR_512;

    // Raw Data
    int16_t m_v_raw[3] = {0, 0, 0};
    int16_t get_axis(int index) const;

    // Smoothing Data
    int16_t m_v_history[10][3] = {{0}};
    int m_v_scan = 0;
    int32_t m_v_totals[3] = {0, 0, 0};
    int16_t m_v_smooth[3] = {0, 0, 0};

    void apply_smoothing();

    // Calibration Data
    float m_offset[3] = {0.f, 0.f, 0.f};
    float m_scale[3] = {1.f, 1.f, 1.f};
    int16_t m_v_calibrated[3] = {0, 0, 0};

    void apply_calibration();

    // Calibration State
    bool m_calib_active = false;
    uint64_t m_calib_start_time = 0;
    uint32_t m_calib_duration_us = 0;
    int16_t m_calib_min[3] = {0, 0, 0};
    int16_t m_calib_max[3] = {0, 0, 0};

    // Constants
    const char m_bearings[16][3] = {{' ', ' ', 'N'},  //
                                    {'N', 'N', 'E'},  //
                                    {' ', 'N', 'E'},  //
                                    {'E', 'N', 'E'},  //
                                    {' ', ' ', 'E'},  //
                                    {'E', 'S', 'E'},  //
                                    {' ', 'S', 'E'},  //
                                    {'S', 'S', 'E'},  //
                                    {' ', ' ', 'S'},  //
                                    {'S', 'S', 'W'},  //
                                    {' ', 'S', 'W'},  //
                                    {'W', 'S', 'W'},  //
                                    {' ', ' ', 'W'},  //
                                    {'W', 'N', 'W'},  //
                                    {' ', 'N', 'W'},  //
                                    {'N', 'N', 'W'}};
};

#endif /* _QMC5883L_H_ */
