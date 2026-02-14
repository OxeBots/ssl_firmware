#include "QMC5883L.h"

/**
 * @brief Default Constructor.
 */
QMC5883L::QMC5883L()
{
    devAddr = QMC5883L_DEFAULT_ADDRESS;
}

/**
 * @brief Constructor with specified I2C address.
 * @param address I2C address of the QMC5883L device.
 */
QMC5883L::QMC5883L(uint8_t address)
{
    devAddr = address;
}


/**
 * @brief Initialize the QMC5883L sensor with default settings.
 */
void QMC5883L::initialize()
{
    // Define Set/Reset Period (Recommended by datasheet)
    I2Cdev::writeByte(devAddr, QMC5883L_RA_PERIOD, 0x01);

    // Set Continuous Mode, 200Hz, 8G Range, OSR 512
    setMode(QMC5883L_MODE_CONTINUOUS, QMC5883L_ODR_200HZ, QMC5883L_RNG_8G, QMC5883L_OSR_512);
}

void QMC5883L::setADDR(uint8_t b)
{
    devAddr = b;
}

void QMC5883L::setMode(uint8_t mode, uint8_t odr, uint8_t rng, uint8_t osr)
{
    _mode = mode;
    _odr = odr;
    _rng = rng;
    _osr = osr;

    I2Cdev::writeByte(devAddr, QMC5883L_RA_CONTROL_1, _mode | _odr | _rng | _osr);
}

void QMC5883L::setReset()
{
    I2Cdev::writeByte(devAddr, QMC5883L_RA_CONTROL_2, 0x80);
}

void QMC5883L::setMagneticDeclination(int degrees, uint8_t minutes)
{
    _magneticDeclinationDegrees = degrees + minutes / 60.0f;
}

void QMC5883L::setSmoothing(uint8_t steps, bool adv)
{
    _smoothUse = true;
    _smoothSteps = (steps > 10) ? 10 : steps;
    _smoothAdvanced = adv;

    // Clear history to prevent jumps when enabling
    memset(_vHistory, 0, sizeof(_vHistory));
    _vTotals[0] = 0;
    _vTotals[1] = 0;
    _vTotals[2] = 0;
    _vScan = 0;
}

void QMC5883L::clearCalibration()
{
    setCalibrationOffsets(0.0f, 0.0f, 0.0f);
    setCalibrationScales(1.0f, 1.0f, 1.0f);
}

void QMC5883L::setCalibrationOffsets(float x_offset, float y_offset, float z_offset)
{
    _offset[0] = x_offset;
    _offset[1] = y_offset;
    _offset[2] = z_offset;
}

void QMC5883L::setCalibrationScales(float x_scale, float y_scale, float z_scale)
{
    _scale[0] = x_scale;
    _scale[1] = y_scale;
    _scale[2] = z_scale;
}

float QMC5883L::getCalibrationOffset(uint8_t index)
{
    if (index < 3)
        return _offset[index];
    return 0.0f;
}

float QMC5883L::getCalibrationScale(uint8_t index)
{
    if (index < 3)
        return _scale[index];
    return 1.0f;
}

void QMC5883L::calibrate()
{
    clearCalibration();
    int32_t calibrationData[3][2] = {{65000, -65000}, {65000, -65000}, {65000, -65000}};

    // Initial Read
    read();
    int16_t x = getX();
    int16_t y = getY();
    int16_t z = getZ();

    calibrationData[0][0] = calibrationData[0][1] = x;
    calibrationData[1][0] = calibrationData[1][1] = y;
    calibrationData[2][0] = calibrationData[2][1] = z;

    uint64_t startTime = esp_timer_get_time();  // Microseconds
    const uint64_t duration = 10000000;         // 10 seconds in microseconds

    while ((esp_timer_get_time() - startTime) < duration)
    {
        read();
        x = getX();
        y = getY();
        z = getZ();

        if (x < calibrationData[0][0])
            calibrationData[0][0] = x;
        if (x > calibrationData[0][1])
            calibrationData[0][1] = x;

        if (y < calibrationData[1][0])
            calibrationData[1][0] = y;
        if (y > calibrationData[1][1])
            calibrationData[1][1] = y;

        if (z < calibrationData[2][0])
            calibrationData[2][0] = z;
        if (z > calibrationData[2][1])
            calibrationData[2][1] = z;

        // Yield to prevent watchdog triggers
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // Calculate Offsets
    float x_avg = (calibrationData[0][1] + calibrationData[0][0]) / 2.0f;
    float y_avg = (calibrationData[1][1] + calibrationData[1][0]) / 2.0f;
    float z_avg = (calibrationData[2][1] + calibrationData[2][0]) / 2.0f;

    setCalibrationOffsets(x_avg, y_avg, z_avg);

    // Calculate Scales
    float x_avg_delta = (calibrationData[0][1] - calibrationData[0][0]) / 2.0f;
    float y_avg_delta = (calibrationData[1][1] - calibrationData[1][0]) / 2.0f;
    float z_avg_delta = (calibrationData[2][1] - calibrationData[2][0]) / 2.0f;

    float avg_delta = (x_avg_delta + y_avg_delta + z_avg_delta) / 3.0f;

    // Prevent division by zero
    if (x_avg_delta != 0)
        _scale[0] = avg_delta / x_avg_delta;
    if (y_avg_delta != 0)
        _scale[1] = avg_delta / y_avg_delta;
    if (z_avg_delta != 0)
        _scale[2] = avg_delta / z_avg_delta;
}

void QMC5883L::read()
{
    // Read 6 bytes starting from 0x00 (DATAX_L)
    if (I2Cdev::readBytes(devAddr, QMC5883L_RA_DATAX_L, 6, buffer) == 6)
    {
        // QMC5883L is Little Endian
        _vRaw[0] = (int16_t)(((uint16_t)buffer[1] << 8) | buffer[0]);
        _vRaw[1] = (int16_t)(((uint16_t)buffer[3] << 8) | buffer[2]);
        _vRaw[2] = (int16_t)(((uint16_t)buffer[5] << 8) | buffer[4]);

        _applyCalibration();

        if (_smoothUse)
        {
            _smoothing();
        }
    }
}

void QMC5883L::_applyCalibration()
{
    _vCalibrated[0] = (_vRaw[0] - _offset[0]) * _scale[0];
    _vCalibrated[1] = (_vRaw[1] - _offset[1]) * _scale[1];
    _vCalibrated[2] = (_vRaw[2] - _offset[2]) * _scale[2];
}

/**
 * @brief Read and get the current calibrated orientation values.
 * @param x  Pointer to store the X-axis orientation value.
 * @param y  Pointer to store the Y-axis orientation velue.
 * @param z  Pointer to store the Z-axis orientation value.
 */
void QMC5883L::getOrientation(int16_t * x, int16_t * y, int16_t * z)
{
    read();
    *x = getX();
    *y = getY();
    *z = getZ();
}

void QMC5883L::_smoothing()
{
    int max_idx = 0;
    int min_idx = 0;

    if (_vScan >= _smoothSteps)
    {
        _vScan = 0;
    }

    for (int i = 0; i < 3; i++)
    {
        // Remove old value from total
        if (_vTotals[i] != 0)
        {  // Simple check, technically not perfect if sum is 0
            _vTotals[i] = _vTotals[i] - _vHistory[_vScan][i];
        }

        // Add new value
        _vHistory[_vScan][i] = _vCalibrated[i];
        _vTotals[i] = _vTotals[i] + _vHistory[_vScan][i];

        if (_smoothAdvanced)
        {
            max_idx = 0;
            min_idx = 0;
            // Find min/max in history
            for (int j = 0; j < _smoothSteps; j++)
            {
                if (_vHistory[j][i] > _vHistory[max_idx][i])
                    max_idx = j;
                if (_vHistory[j][i] < _vHistory[min_idx][i])
                    min_idx = j;
            }

            // Average excluding min and max
            int32_t sum = _vTotals[i] - (_vHistory[max_idx][i] + _vHistory[min_idx][i]);
            if (_smoothSteps > 2)
            {
                _vSmooth[i] = sum / (_smoothSteps - 2);
            }
            else
            {
                _vSmooth[i] = _vTotals[i] / _smoothSteps;  // Fallback
            }
        }
        else
        {
            _vSmooth[i] = _vTotals[i] / _smoothSteps;
        }
    }
    _vScan++;
}

int16_t QMC5883L::_get(int index)
{
    if (index < 0 || index > 2)
        return 0;
    if (_smoothUse)
        return _vSmooth[index];
    return _vCalibrated[index];
}

int QMC5883L::getAzimuth()
{
    float heading = atan2((float)getY(), (float)getX()) * 180.0 / M_PI;
    heading += _magneticDeclinationDegrees;

    // Normalize to 0-360
    while (heading < 0)
        heading += 360;
    while (heading >= 360)
        heading -= 360;

    return (int)heading;
}

uint8_t QMC5883L::getBearing(int azimuth)
{
    // azimuth is 0-360
    // 360 / 16 = 22.5 degrees per sector
    // Shift by half sector (11.25) so 0 is centered on N
    float sector = (float)azimuth / 22.5f;
    int bearing = (int)(sector + 0.5f);

    return (uint8_t)(bearing % 16);
}

void QMC5883L::getDirection(char * myArray, int azimuth)
{
    int d = getBearing(azimuth);
    myArray[0] = _bearings[d][0];
    myArray[1] = _bearings[d][1];
    myArray[2] = _bearings[d][2];
    myArray[3] = '\0';  // Null terminator safety
}

bool QMC5883L::testConnection()
{
    return getChipID() == 0xFF;
}

uint8_t QMC5883L::getChipID()
{
    I2Cdev::readByte(devAddr, QMC5883L_RA_CHIP_ID, buffer);
    return buffer[0];
}

/**
 * @brief Set the Magnetometer Range.
 * @param rng One of QMC5883L_RNG_2G or QMC5883L_RNG_8G
 */
void QMC5883L::setRange(uint8_t rng)
{
    // Update only the range, keep other settings (ODR, Mode, OSR)
    _rng = rng;
    setMode(_mode, _odr, _rng, _osr);
}

/**
 * @brief Get the current Magnetometer Range.
 * @return QMC5883L_RNG_2G or QMC5883L_RNG_8G
 */
uint8_t QMC5883L::getRange()
{
    return _rng;
}
