
#include "IMUGY85.h"

#include <sdkconfig.h>

static const char * TAG = "IMUGY85";

IMUGY85 & IMUGY85::get_instance()
{
    static IMUGY85 instance;
    return instance;
}

IMUGY85::IMUGY85() : m_i2c_bus(nullptr)
{
}

/**
 * Initialize I2C bus and all IMU sensors.
 * @return ESP_OK on success
 */
esp_err_t IMUGY85::init()
{
    // ---- I2C master bus ----
    i2c_master_bus_config_t i2c_cfg = {
      .i2c_port = static_cast<i2c_port_t>(CONFIG_I2C_PORT_NUM),
      .sda_io_num = static_cast<gpio_num_t>(CONFIG_SDA_GPIO),
      .scl_io_num = static_cast<gpio_num_t>(CONFIG_SCL_GPIO),
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .intr_priority = 0,
      .trans_queue_depth = 0,
      .flags = {.enable_internal_pullup = true, .allow_pd = false},
    };

    esp_err_t err = i2c_new_master_bus(&i2c_cfg, &m_i2c_bus);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(err));
        return err;
    }

    I2Cdev::init(m_i2c_bus);
    ESP_LOGI(TAG, "I2C bus initialized (SDA=%d, SCL=%d).", CONFIG_SDA_GPIO, CONFIG_SCL_GPIO);

    // ---- Initialize individual sensors ----
    m_accel.init();
    ESP_LOGI(TAG, "ADXL345 accelerometer initialized.");

    m_gyro.init();
    ESP_LOGI(TAG, "ITG3200 gyroscope initialized.");

    m_mag.init();
    m_mag.set_smoothing(10, true);
    ESP_LOGI(TAG, "QMC5883L magnetometer initialized.");

    // ---- Calculate resolutions ----
    update_a_res();
    update_g_res();
    update_m_res();

    return ESP_OK;
}

/**
 * Read raw accelerometer data.
 * @param x Output pointer for X axis
 * @param y Output pointer for Y axis
 * @param z Output pointer for Z axis
 */
void IMUGY85::read_acceleration(int16_t * x, int16_t * y, int16_t * z)
{
    m_accel.get_acceleration(x, y, z);
}

/**
 * Read raw gyroscope data.
 * @param x Output pointer for X axis
 * @param y Output pointer for Y axis
 * @param z Output pointer for Z axis
 */
void IMUGY85::read_gyro(int16_t * x, int16_t * y, int16_t * z)
{
    m_gyro.get_rotation(x, y, z);
}

/**
 * Read raw magnetometer data.
 * @param x Output pointer for X axis
 * @param y Output pointer for Y axis
 * @param z Output pointer for Z axis
 */
void IMUGY85::read_magnetometer(int16_t * x, int16_t * y, int16_t * z)
{
    m_mag.get_orientation(x, y, z);
}

/**
 * Read calibrated accelerometer data (in g's).
 * @return Vector3f with calibrated acceleration values
 */
Vector3f IMUGY85::read_acceleration_calibrated()
{
    int16_t raw_x, raw_y, raw_z;
    read_acceleration(&raw_x, &raw_y, &raw_z);

    Vector3f result;
    result.x = (static_cast<float>(raw_x) * m_a_res - m_accel_offset.x) * m_accel_scale.x;
    result.y = (static_cast<float>(raw_y) * m_a_res - m_accel_offset.y) * m_accel_scale.y;
    result.z = (static_cast<float>(raw_z) * m_a_res - m_accel_offset.z) * m_accel_scale.z;
    return result;
}

/**
 * Read calibrated gyroscope data (in deg/s).
 * @return Vector3f with calibrated gyro values
 */
Vector3f IMUGY85::read_gyro_calibrated()
{
    int16_t raw_x, raw_y, raw_z;
    read_gyro(&raw_x, &raw_y, &raw_z);

    Vector3f result;
    result.x = (static_cast<float>(raw_x) * m_g_res - m_gyro_offset.x) * m_gyro_scale.x;
    result.y = (static_cast<float>(raw_y) * m_g_res - m_gyro_offset.y) * m_gyro_scale.y;
    result.z = (static_cast<float>(raw_z) * m_g_res - m_gyro_offset.z) * m_gyro_scale.z;
    return result;
}

/**
 * Read calibrated magnetometer data (in mG).
 * @return Vector3f with calibrated magnetometer values
 */
Vector3f IMUGY85::read_magnetometer_calibrated()
{
    int16_t raw_x, raw_y, raw_z;
    read_magnetometer(&raw_x, &raw_y, &raw_z);

    Vector3f result;
    result.x = (static_cast<float>(raw_x) * m_m_res - m_mag_offset.x) * m_mag_scale.x;
    result.y = (static_cast<float>(raw_y) * m_m_res - m_mag_offset.y) * m_mag_scale.y;
    result.z = (static_cast<float>(raw_z) * m_m_res - m_mag_offset.z) * m_mag_scale.z;
    return result;
}

/**
 * Set accelerometer calibration scale and offset.
 * @param scale Calibration scale factors for each axis
 * @param offset Calibration offsets for each axis
 */
void IMUGY85::set_accel_calibration(const Vector3f & scale, const Vector3f & offset)
{
    m_accel_scale = scale;
    m_accel_offset = offset;
}

/**
 * Set gyroscope calibration scale and offset.
 * @param scale Calibration scale factors for each axis
 * @param offset Calibration offsets for each axis
 */
void IMUGY85::set_gyro_calibration(const Vector3f & scale, const Vector3f & offset)
{
    m_gyro_scale = scale;
    m_gyro_offset = offset;
}

/**
 * Set magnetometer calibration scale and offset.
 * @param scale Calibration scale factors for each axis
 * @param offset Calibration offsets for each axis
 */
void IMUGY85::set_mag_calibration(const Vector3f & scale, const Vector3f & offset)
{
    m_mag_scale = scale;
    m_mag_offset = offset;
}

/**
 * Get accelerometer resolution in g per LSB.
 * @return Resolution value
 */
float IMUGY85::get_accel_resolution() const
{
    return m_a_res;
}

/**
 * Get gyroscope resolution in deg/s per LSB.
 * @return Resolution value
 */
float IMUGY85::get_gyro_resolution() const
{
    return m_g_res;
}

/**
 * Get magnetometer resolution in mG per LSB.
 * @return Resolution value
 */
float IMUGY85::get_mag_resolution() const
{
    return m_m_res;
}

ADXL345 & IMUGY85::get_accel()
{
    return m_accel;
}

ITG3200 & IMUGY85::get_gyro()
{
    return m_gyro;
}

QMC5883L & IMUGY85::get_mag()
{
    return m_mag;
}

void IMUGY85::update_a_res()
{
    // ADXL345 full resolution mode: 4mg/LSB at +/-16g range
    m_a_res = 4.0f / 1024.0f;  // g per LSB
}

void IMUGY85::update_g_res()
{
    // ITG3200 at +/-2000 deg/s: 14.375 LSB/deg/s
    m_g_res = 1.0f / 14.375f;  // deg/s per LSB
}

void IMUGY85::update_m_res()
{
    // QMC5883L at +/-8G range: 3000 LSB/G = 3 mG/LSB
    m_m_res = 1000.0f / 3000.0f;  // mG per LSB
}
