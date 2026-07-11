#ifndef ORIENTATION_HANDLER_H
#define ORIENTATION_HANDLER_H

#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "Fusion.h"
#include "IMUGY85.h"
#include "NVSManager.h"

class OrientationHandler
{
   public:
    static OrientationHandler & get_instance();

    // Initialization and task management
    esp_err_t init();
    void start_task();

    // Calibration routines
    esp_err_t load_calibration();
    esp_err_t calibrate_mag(uint32_t timeout_s = 10);
    esp_err_t calibrate_accel(uint32_t timeout_s = 10);
    esp_err_t calibrate_gyro(uint32_t timeout_s = 10);

    // Thread-safe accessors
    double get_yaw() const;
    double get_pitch() const;
    double get_roll() const;

    void get_acceleration(double * ax, double * ay, double * az) const;
    void get_gyro(double * gx, double * gy, double * gz) const;
    void get_magnetometer(double * mx, double * my, double * mz) const;

    // AHRS configuration
    void set_fusion_gain(float gain);
    void set_acceleration_rejection(float degrees);
    void set_magnetic_rejection(float degrees);

    // Calibration state queries
    bool is_accel_calibrated() const;
    bool is_gyro_calibrated() const;
    bool is_mag_calibrated() const;

   private:
    OrientationHandler();
    ~OrientationHandler() = default;
    OrientationHandler(const OrientationHandler &) = delete;
    OrientationHandler & operator=(const OrientationHandler &) = delete;

    // Task and AHRS update
    static void task_wrapper(void * arg);
    void task_loop();
    void update_ahrs();

    // Calibration helpers
    esp_err_t save_accel_calibration_to_nvs();
    esp_err_t save_gyro_calibration_to_nvs();
    esp_err_t save_mag_calibration_to_nvs();
    esp_err_t load_accel_calibration_from_nvs();
    esp_err_t load_gyro_calibration_from_nvs();
    esp_err_t load_mag_calibration_from_nvs();

    IMUGY85 & m_driver;

    // AHRS states
    FusionAhrs m_ahrs = {};
    FusionOffset m_offset = {};
    FusionAhrsSettings m_settings = {};

    // FreeRTOS
    TaskHandle_t m_task_handle = nullptr;
    SemaphoreHandle_t m_mutex = nullptr;

    // Cached data (updated by task, read by accessors)
    double m_yaw = 0.0;
    double m_pitch = 0.0;
    double m_roll = 0.0;
    double m_ax = 0.0;
    double m_ay = 0.0;
    double m_az = 0.0;
    double m_gx = 0.0;
    double m_gy = 0.0;
    double m_gz = 0.0;
    double m_mx = 0.0;
    double m_my = 0.0;
    double m_mz = 0.0;

    // Calibration state flags
    bool m_accel_calibrated = false;
    bool m_gyro_calibrated = false;
    bool m_mag_calibrated = false;

    // Constants
    static constexpr uint32_t TASK_RATE_HZ = 100;
    static constexpr uint32_t MUTEX_WAIT_MS = 10;
    static constexpr uint32_t TASK_STACK_BYTES = 4096;
    static constexpr uint32_t TASK_PRIORITY = 5;

    // NVS namespaces and keys
    static constexpr const char * NVS_NS_ACCEL = "accel_calib";
    static constexpr const char * NVS_NS_GYRO = "gyro_calib";
    static constexpr const char * NVS_NS_MAG = "mag_calib";
    static constexpr const char * NVS_KEY_BLOB = "calib_blob";
};

#endif  // ORIENTATION_HANDLER_H
