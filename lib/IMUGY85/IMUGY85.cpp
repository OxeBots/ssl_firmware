#include "IMUGY85.h"

static const char * TAG = "IMUGY85";

IMUGY85::IMUGY85()
{
}

void IMUGY85::init()
{
    accel.initialize();
    gyro.initialize();
    mag.init();

    // Configure ADXL345
    accel.setRange(m_a_scale);

    m_a_scale = accel.getRange();
    m_accel_misalignment = {1.f, 0.f, 0.f,  //
                            0.f, 1.f, 0.f,  //
                            0.f, 0.f, 1.f};
    m_accel_sensitivity = {1.f, 1.f, 1.f};
    m_accel_offset = {0.f, 0.f, 0.f};

    m_gyro_misalignment = {1.f, 0.f, 0.f,  //
                           0.f, 1.f, 0.f,  //
                           0.f, 0.f, 1.f};
    m_gyro_sensitivity = {1.f, 1.f, 1.f};
    m_gyro_offset = {0.f, 0.f, 0.f};

    gyro.setOffsets(-4, -8, 6);

    // Identity matrices as internal mag driver applies offset/scale
    m_soft_iron_matrix = {1.f, 0.f, 0.f,  //
                          0.f, 1.f, 0.f,  //
                          0.f, 0.f, 1.f};
    m_hard_iron_offset = {0.f, 0.f, 0.f};

    FusionOffsetInitialise(&m_offset, IMU_SAMPLE_RATE);
    FusionAhrsInitialise(&m_ahrs);

    m_settings.convention = FusionConventionEnu;  // ENU (East-North-Up)
    m_settings.gain = 0.5f;
    m_settings.gyroscopeRange = 2000.0f;
    m_settings.accelerationRejection = 10.0f;
    m_settings.magneticRejection = 10.0f;
    m_settings.recoveryTriggerPeriod = 5 * IMU_SAMPLE_RATE;

    FusionAhrsSetSettings(&m_ahrs, &m_settings);

    m_last_update = esp_timer_get_time();
}

esp_err_t IMUGY85::load_or_calibrate_mag(uint32_t seconds)
{
    if (mag.load_calibration_from_nvs() == ESP_OK)
    {
        ESP_LOGI(TAG, "Magnetometer calibration loaded via NVSManager.");
        return ESP_OK;
    }

    ESP_LOGW(TAG, "No magnetometer calibration found.");
    return calibrate_magnetometer(seconds);
}

esp_err_t IMUGY85::calibrate_magnetometer(uint32_t seconds)
{
    ESP_LOGW(TAG, "Starting Magnetometer Calibration routine. Rotate the robot in its z axis on a full rotation!");

    for (int i = 3; i > 0; i--)
    {
        ESP_LOGI(TAG, "Starting in %d...", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "GO!");

    mag.start_calibration_mode(seconds);
    bool finished = false;

    while (!finished)
    {
        finished = mag.calibration_update();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    mag.stop_calibration_mode();
    return ESP_OK;
}

void IMUGY85::update()
{
    accel.getAcceleration(&m_accel_count[0], &m_accel_count[1], &m_accel_count[2]);
    gyro.getRotation(&m_gyro_count[0], &m_gyro_count[1], &m_gyro_count[2]);
    mag.read();

    m_mag_count[0] = mag.get_x();
    m_mag_count[1] = mag.get_y();
    m_mag_count[2] = mag.get_z();

    update_a_res();
    update_g_res();
    update_m_res();

    FusionVector gyroscopeUncal = {(float)m_gyro_count[0] * m_g_res,  //
                                   (float)m_gyro_count[1] * m_g_res,  //
                                   (float)m_gyro_count[2] * m_g_res};

    FusionVector accelerometerUncal = {(float)m_accel_count[1] * m_a_res,   //
                                       -(float)m_accel_count[0] * m_a_res,  //
                                       (float)m_accel_count[2] * m_a_res};

    FusionVector magnetometerUncal = {(float)m_mag_count[0] * m_m_res,  //
                                      (float)m_mag_count[1] * m_m_res,  //
                                      (float)m_mag_count[2] * m_m_res};

    FusionVector gyroscope =
      FusionCalibrationInertial(gyroscopeUncal, m_gyro_misalignment, m_gyro_sensitivity, m_gyro_offset);
    FusionVector accelerometer =
      FusionCalibrationInertial(accelerometerUncal, m_accel_misalignment, m_accel_sensitivity, m_accel_offset);
    FusionVector magnetometer = FusionCalibrationMagnetic(magnetometerUncal, m_soft_iron_matrix, m_hard_iron_offset);

    magnetometer.axis.x = magnetometer.axis.y;
    magnetometer.axis.y = magnetometer.axis.x;
    magnetometer.axis.z = magnetometer.axis.z;

    gyroscope = FusionOffsetUpdate(&m_offset, gyroscope);

    m_gx = gyroscope.axis.x;
    m_gy = gyroscope.axis.y;
    m_gz = gyroscope.axis.z;
    m_ax = accelerometer.axis.x;
    m_ay = accelerometer.axis.y;
    m_az = accelerometer.axis.z;
    m_mx = magnetometer.axis.x;
    m_my = magnetometer.axis.y;
    m_mz = magnetometer.axis.z;

    m_now = esp_timer_get_time();
    m_dt = ((m_now - m_last_update) / 1000000.0f);
    m_last_update = m_now;

    if (m_dt > 1.0f)
        m_dt = 0.01f;

    FusionAhrsUpdateNoMagnetometer(&m_ahrs, gyroscope, accelerometer, m_dt);
    FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&m_ahrs));

    m_roll = euler.angle.roll;
    m_pitch = euler.angle.pitch;
    m_yaw = euler.angle.yaw;
}

double IMUGY85::get_roll() const
{
    return m_roll;
}

double IMUGY85::get_pitch() const
{
    return m_pitch;
}

double IMUGY85::get_yaw() const
{
    return m_yaw;
}

void IMUGY85::get_acceleration(double * a1, double * a2, double * a3) const
{
    *a1 = m_ax;
    *a2 = m_ay;
    *a3 = m_az;
}

void IMUGY85::get_gyro(double * m1, double * m2, double * m3) const
{
    *m1 = m_gx;
    *m2 = m_gy;
    *m3 = m_gz;
}

void IMUGY85::get_magnetometer(double * m1, double * m2, double * m3) const
{
    *m1 = m_mx;
    *m2 = m_my;
    *m3 = m_mz;
}

void IMUGY85::set_gyroscope_calibration(FusionMatrix misalignment, FusionVector sensitivity, FusionVector offset_vec)
{
    m_gyro_misalignment = misalignment;
    m_gyro_sensitivity = sensitivity;
    m_gyro_offset = offset_vec;
}

void IMUGY85::set_accelerometer_calibration(FusionMatrix misalignment, FusionVector sensitivity,
                                            FusionVector offset_vec)
{
    m_accel_misalignment = misalignment;
    m_accel_sensitivity = sensitivity;
    m_accel_offset = offset_vec;
}

void IMUGY85::set_magnetometer_calibration(FusionMatrix softIron, FusionVector hardIron)
{
    m_soft_iron_matrix = softIron;
    m_hard_iron_offset = hardIron;
}

void IMUGY85::update_m_res()
{
    m_m_res = (mag.get_range() == QMC5883L::Range::RNG_2G) ? (1000.0f / 12000.0f) : (1000.0f / 3000.0f);
}

void IMUGY85::update_g_res()
{
    m_g_res = 1.0f / 14.375f;
}

void IMUGY85::update_a_res()
{
    if (m_a_full_res)
        m_a_res = 1.0f / 256.0f;
    else
    {
        switch (m_a_scale)
        {
            case AFS_2G:
                m_a_res = 4.0f / 1024.0f;
                break;
            case AFS_4G:
                m_a_res = 8.0f / 1024.0f;
                break;
            case AFS_8G:
                m_a_res = 16.0f / 1024.0f;
                break;
            case AFS_16G:
                m_a_res = 32.0f / 1024.0f;
                break;
            default:
                m_a_res = 1.0f / 256.0f;
                break;
        }
    }
}
