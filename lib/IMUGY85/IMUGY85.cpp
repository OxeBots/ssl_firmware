/**
 * @file IMUGY85.cpp
 * @brief Driver for the GY-85 9-DOF IMU sensor board using Fusion AHRS.
 * @details This library fuses data from the ADXL345 accelerometer, ITG3200 gyroscope, and QMC5883L
 * magnetometer using the Fusion AHRS library to provide stable orientation estimates (Roll, Pitch,
 * Yaw).
 */

#include "IMUGY85.h"

/**
 * @brief Default Constructor.
 */
IMUGY85::IMUGY85()
{
}

/**
 * @brief Initialize all sensors and Fusion Algorithm.
 *
 * * Calls the `initialize()` method for ADXL345, ITG3200, and QMC5883L.
 * * Sets up default identity matrices for calibration (no calibration).
 * * Initializes Fusion Offset and AHRS.
 * @note Requires the I2C Master bus to be already initialized via `I2Cdev::init()`.
 */
void IMUGY85::init()
{
    // 1. Initialize Sensors
    accel.initialize();
    gyro.initialize();
    mag.initialize();

    // Configure ADXL345
    // We set the range based on the Ascale variable (default AFS_16G)
    accel.setRange(Ascale);
    // We enable Full Resolution by default for best precision (4mg/LSB across all ranges)
    accel.setFullResolution(true);
    // Update local variable with actual device setting to ensure they are synced (Cached)
    Ascale = accel.getRange();

    // Calib data:
    // z offset min: -1.14 | z offset max: 0.83
    // x offset min: -1.00 | x offset max: 1.01
    // y offset min: -1.00 | y offset max: 1.01
    // Scale = (Max - Min) / 2
    // Offset = (Max + Min) / 2
    constexpr float Ax_offset = (1.01f - 1.00f) / 2.0f;          // 0.005
    constexpr float Ay_offset = (1.01f - 1.00f) / 2.0f;          // 0.005
    constexpr float Az_offset = (0.83f - 1.14f) / 2.0f;          // -0.155
    constexpr float Ax_scale = 1.0f / ((1.01f + 1.00f) / 2.0f);  // 0.995
    constexpr float Ay_scale = 1.0f / ((1.01f + 1.00f) / 2.0f);  // 0.995
    constexpr float Az_scale = 1.0f / ((0.83f + 1.14f) / 2.0f);  // 0.9804

    accelerometerMisalignment = {1.0f, 0.0f, 0.0f,   //
                                 0.0f, 1.0f, 0.0f,   //
                                 0.0f, 0.0f, 1.0f};  //
    accelerometerSensitivity = {Ax_scale, Ay_scale, Az_scale};
    accelerometerOffset = {Ax_offset, Ay_offset, Az_offset};  // offsets from driver not working?

    // 2. Initialize Calibration Data to "Identity" (No effect)
    // Replace these values if you have specific calibration data
    gyroscopeMisalignment = {1.0f, 0.0f, 0.0f,   //
                             0.0f, 1.0f, 0.0f,   //
                             0.0f, 0.0f, 1.0f};  //
    gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
    gyroscopeOffset = {0.0f, 0.0f, 0.0f};
    gyro.setOffsets(-4, -8, 6);

    // The magnetometer calibration values below were obtained following the calibration method
    // exposed in https://www.instructables.com/Easy-hard-and-soft-iron-magnetometer-calibration/
    // softIronMatrix = {1.891f,  -0.037f, 0.276f,   //
    //                   0.059f,  1.972f,  0.088f,   //
    //                   -0.034f, -0.171f, 1.894f};  //
    // hardIronOffset = {376.305f, -448.673f, 463.261f};

    softIronMatrix = {-1.891f, -0.037f, 0.276f,   //
                      -0.059f, 1.972f,  0.088f,   //
                      0.034f,  -0.171f, 1.894f};  //

    hardIronOffset = {-376.305f * mRes, -448.673f * mRes, 463.261f * mRes};

    mag.setCalibrationOffsets(-1364.00f, -779.00f, 269.50f);
    mag.setCalibrationScales(0.92f, 0.82f, 1.42f);

    // 3. Initialize Fusion Algorithms
    FusionOffsetInitialise(&offset, IMU_SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);

    // 4. Set AHRS algorithm settings
    // settings.convention = FusionConventionNwu;  // NWU (North-West-Up)
    settings.convention = FusionConventionEnu;  // ENU (East-North-Up)
    settings.gain = 0.5f;
    settings.gyroscopeRange = 2000.0f; /* Based on GFS_2000DPS/GFS_CUSTOM default */
    settings.accelerationRejection = 10.0f;
    settings.magneticRejection = 10.0f;
    settings.recoveryTriggerPeriod = 5 * IMU_SAMPLE_RATE; /* 5 seconds */

    FusionAhrsSetSettings(&ahrs, &settings);

    // Initialize timer tracking
    lastUpdate = esp_timer_get_time();
}

/**
 * @brief Main update loop for the IMU.
 *
 * - 1. Reads raw data from Accel, Gyro, and Mag.
 * - 2. Converts raw counts to physical units.
 * - 3. Applies Calibration (FusionCalibrationInertial/Magnetic).
 * - 4. Runs Fusion Offset Update (Gyro drift correction).
 * - 5. Runs Fusion AHRS Update.
 * - 6. Stores Euler angles.
 */
void IMUGY85::update()
{
    // ADXL345 Read
    accel.getAcceleration(&accelCount[0], &accelCount[1], &accelCount[2]);
    // ITG3200 Read
    gyro.getRotation(&gyroCount[0], &gyroCount[1], &gyroCount[2]);
    // QMC5883L Read
    mag.getOrientation(&magCount[0], &magCount[1], &magCount[2]);

    getAres();  // Update aRes
    getGres();  // Update gRes
    getMres();  // Update mRes

    // Convert Gyro counts to deg/s (Fusion expects deg/s)
    FusionVector gyroscopeUncal = {(float)gyroCount[0] * gRes,  //
                                   (float)gyroCount[1] * gRes,  //
                                   (float)gyroCount[2] * gRes};

    // Convert Accel counts to g's
    FusionVector accelerometerUncal = {(float)accelCount[1] * aRes,   //
                                       -(float)accelCount[0] * aRes,  //
                                       (float)accelCount[2] * aRes};

    // Magnetometer (mG or arbitrary units)
    FusionVector magnetometerUncal = {(float)magCount[0] * mRes,  //
                                      (float)magCount[1] * mRes,  //
                                      (float)magCount[2] * mRes};

    // --- 4. Apply Calibration ---
    FusionVector gyroscope =
      FusionCalibrationInertial(gyroscopeUncal, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);

    FusionVector accelerometer = FusionCalibrationInertial(accelerometerUncal, accelerometerMisalignment,
                                                           accelerometerSensitivity, accelerometerOffset);

    FusionVector magnetometer = FusionCalibrationMagnetic(magnetometerUncal, softIronMatrix, hardIronOffset);
    // =================================================================================
    // TRIAL 1: Same alignment as Accelerometer (Most Likely)
    // If chips are mounted with Pin 1 in the same orientation:
    // Body X = Sensor Y | Body Y = -Sensor X
    // =================================================================================
    // magnetometer.axis.x = magnetometer.axis.y;
    // magnetometer.axis.y = -magnetometer.axis.x;
    // magnetometer.axis.z = magnetometer.axis.z;

    // =================================================================================
    // TRIAL 2: Standard / No Rotation
    // Use this if the Mag X aligns with Body X
    // =================================================================================
    magnetometer.axis.x = magnetometer.axis.y;
    magnetometer.axis.y = magnetometer.axis.x;
    magnetometer.axis.z = magnetometer.axis.z;

    // =================================================================================
    // TRIAL 3: 90 Degrees Clockwise
    // Body X = Sensor -Y | Body Y = Sensor X
    // =================================================================================
    // magnetometer.axis.x = -magnetometer.axis.y;
    // magnetometer.axis.y = magnetometer.axis.x;
    // magnetometer.axis.z = magnetometer.axis.z;

    // =================================================================================
    // TRIAL 4: 180 Degrees
    // Body X = -Sensor X | Body Y = -Sensor Y
    // =================================================================================
    // magnetometer.axis.x = -magnetometer.axis.x;
    // magnetometer.axis.y = -magnetometer.axis.y;
    // magnetometer.axis.z = magnetometer.axis.z;

    // =================================================================================
    // --- 5. Update Gyroscope Offset ---
    gyroscope = FusionOffsetUpdate(&offset, gyroscope);

    // Update local variables for getters
    gx = gyroscope.axis.x;
    gy = gyroscope.axis.y;
    gz = gyroscope.axis.z;
    ax = accelerometer.axis.x;
    ay = accelerometer.axis.y;
    az = accelerometer.axis.z;
    mx = magnetometer.axis.x;
    my = magnetometer.axis.y;
    mz = magnetometer.axis.z;

    // --- 6. Time Delta Calculation ---
    Now = esp_timer_get_time();
    dt = ((Now - lastUpdate) / 1000000.0f);  // Convert microseconds to seconds
    lastUpdate = Now;

    // Prevent extremely large dt on startup or glitches
    if (dt > 1.0f)
        dt = 0.01f;

    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, dt);

    FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

    // Store in class members (Fusion outputs in degrees)
    roll = euler.angle.roll;
    pitch = euler.angle.pitch;
    yaw = euler.angle.yaw;
}

/**
 * @brief Get calculated Roll.
 * @return Roll in degrees.
 */
double IMUGY85::getRoll()
{
    return roll;
}

/**
 * @brief Get calculated Pitch.
 * @return Pitch in degrees.
 */
double IMUGY85::getPitch()
{
    return pitch;
}

/**
 * @brief Get calculated Yaw.
 * @return Yaw in degrees (Normalized 0-360).
 */
double IMUGY85::getYaw()
{
    return yaw;
}

/**
 * @brief Populate variables with current acceleration (g).
 */
void IMUGY85::getAcceleration(double * a1, double * a2, double * a3)
{
    *a1 = ax;
    *a2 = ay;
    *a3 = az;
}

/**
 * @brief Populate variables with current gyro rates (deg/s).
 */
void IMUGY85::getGyro(double * m1, double * m2, double * m3)
{
    *m1 = gx;
    *m2 = gy;
    *m3 = gz;
}

/**
 * @brief Populate variables with current magnetometer values (mG).
 */
void IMUGY85::getMagnetometer(double * m1, double * m2, double * m3)
{
    *m1 = mx;
    *m2 = my;
    *m3 = mz;
}

/**
 * @brief Calculate Magnetometer resolution (mG/LSB).
 *
 * * Based on current Mscale setting.
 */
void IMUGY85::getMres()
{
    // Retrieve the actual setting from the driver
    uint8_t currentRange = mag.getRange();

    if (currentRange == QMC5883L_RNG_2G)
    {
        // Range +/- 2G: Sensitivity = 12000 LSB/G
        // Resolution = 1000 / 12000 = 0.0833 mG/LSB
        mRes = 1000.0f / 12000.0f;
    }
    else
    {
        // Range +/- 8G: Sensitivity = 3000 LSB/G
        // Resolution = 1000 / 3000 = 0.3333 mG/LSB
        mRes = 1000.0f / 3000.0f;
    }
}

/**
 * @brief Calculate Gyroscope resolution (deg/s per LSB).
 *
 * The ITG-3200 has a fixed Full Scale Range of +/- 2000 degrees per second.
 * The datasheet specifies a sensitivity of 14.375 LSB per deg/s.
 */
void IMUGY85::getGres()
{
    // We switch on Gscale just to keep the structure consistent,
    // but there is only one valid case for this hardware.
    switch (Gscale)
    {
        case GFS_2000DPS:
        default:
            // Formula: 1 / Sensitivity
            // 1.0 / 14.375 = 0.069565... deg/s per LSB
            gRes = 1.0f / 14.375f;
            break;
    }
}

/**
 * @brief Calculate Accelerometer resolution (g/LSB).
 *
 * * Based on current Ascale setting.
 */
void IMUGY85::getAres()
{
    if (AfullRes)
    {
        // ADXL345 Full Resolution Mode:
        // Scale factor is maintained at ~4 mg/LSB regardless of the selected range.
        // 1g = 256 LSBs
        aRes = 1.0f / 256.0f;  // 0.00390625 g/LSB
    }
    else
    {
        // ADXL345 10-Bit Mode (Fixed Resolution):aa
        switch (Ascale)
        {
            case AFS_2G:
                // Range +/- 2G (Total 4G span) over 1024 steps
                aRes = 4.0f / 1024.0f;  // ~3.9 mg/LSB
                break;
            case AFS_4G:
                // Range +/- 4G (Total 8G span) over 1024 steps
                aRes = 8.0f / 1024.0f;  // ~7.8 mg/LSB
                break;
            case AFS_8G:
                // Range +/- 8G (Total 16G span) over 1024 steps
                aRes = 16.0f / 1024.0f;  // ~15.6 mg/LSB
                break;
            case AFS_16G:
                // Range +/- 16G (Total 32G span) over 1024 steps
                aRes = 32.0f / 1024.0f;  // ~31.2 mg/LSB
                break;
            default:
                aRes = 1.0f / 256.0f;  // Default fallback
                break;
        }
    }
}
