#ifndef _QMC5883L_H_
#define _QMC5883L_H_

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <string.h>

#include "I2Cdev.h"

// I2C Address
#define QMC5883L_DEFAULT_ADDRESS 0x0D

// Register Map
#define QMC5883L_RA_DATAX_L 0x00
#define QMC5883L_RA_DATAX_H 0x01
#define QMC5883L_RA_DATAY_L 0x02
#define QMC5883L_RA_DATAY_H 0x03
#define QMC5883L_RA_DATAZ_L 0x04
#define QMC5883L_RA_DATAZ_H 0x05
#define QMC5883L_RA_STATUS 0x06
#define QMC5883L_RA_TEMP_L 0x07
#define QMC5883L_RA_TEMP_H 0x08
#define QMC5883L_RA_CONTROL_1 0x09
#define QMC5883L_RA_CONTROL_2 0x0A
#define QMC5883L_RA_PERIOD 0x0B
#define QMC5883L_RA_CHIP_ID 0x0D

// Mode Control (0x09)
#define QMC5883L_MODE_STANDBY 0x00
#define QMC5883L_MODE_CONTINUOUS 0x01

#define QMC5883L_ODR_10HZ 0x00
#define QMC5883L_ODR_50HZ 0x04
#define QMC5883L_ODR_100HZ 0x08
#define QMC5883L_ODR_200HZ 0x0C

#define QMC5883L_RNG_2G 0x00
#define QMC5883L_RNG_8G 0x10

#define QMC5883L_OSR_512 0x00
#define QMC5883L_OSR_256 0x40
#define QMC5883L_OSR_128 0x80
#define QMC5883L_OSR_64 0xC0

class QMC5883L
{
   public:
    QMC5883L();
    QMC5883L(uint8_t address);

    void initialize();
    bool testConnection();

    // Configuration
    void setADDR(uint8_t b);
    void setMode(uint8_t mode, uint8_t odr, uint8_t rng, uint8_t osr);
    void setReset();

    // Calibration & Smoothing Settings
    void setMagneticDeclination(int degrees, uint8_t minutes);
    void setSmoothing(uint8_t steps, bool adv);
    void calibrate();  // Blocking call (10 seconds)
    void clearCalibration();

    // Manual Calibration Access
    void setCalibrationOffsets(float x_offset, float y_offset, float z_offset);
    void setCalibrationScales(float x_scale, float y_scale, float z_scale);
    float getCalibrationOffset(uint8_t index);
    float getCalibrationScale(uint8_t index);

    // Data Access
    void read();  // Must call this to update values
    int16_t inline getX() { return _get(0); }
    int16_t inline getY() { return _get(1); }
    int16_t inline getZ() { return _get(2); }

    void getOrientation(int16_t * x, int16_t * y, int16_t * z);

    // Calculated Data
    int getAzimuth();
    uint8_t getBearing(int azimuth);
    void getDirection(char * myArray, int azimuth);

    uint8_t getChipID();

    void setRange(uint8_t rng);
    uint8_t getRange();

   private:
    uint8_t devAddr;
    uint8_t buffer[6];

    // Settings
    float _magneticDeclinationDegrees = 0;
    bool _smoothUse = false;
    uint8_t _smoothSteps = 5;
    bool _smoothAdvanced = false;

    uint8_t _mode;
    uint8_t _odr;
    uint8_t _rng;
    uint8_t _osr;

    // Raw Data
    int16_t _vRaw[3] = {0, 0, 0};

    // Smoothing Data
    int16_t _vHistory[10][3];
    int _vScan = 0;
    int32_t _vTotals[3] = {0, 0, 0};
    int16_t _vSmooth[3] = {0, 0, 0};
    void _smoothing();

    // Calibration Data
    float _offset[3] = {0., 0., 0.};
    float _scale[3] = {1., 1., 1.};
    int16_t _vCalibrated[3];
    void _applyCalibration();
    int16_t _get(int index);

    // Constants
    const char _bearings[16][3] = {
      {' ', ' ', 'N'},  //
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
      {'N', 'N', 'W'},  //
    };
};

#endif /* _QMC5883L_H_ */
