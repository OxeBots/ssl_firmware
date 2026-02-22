#include <driver/ledc.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <sdkconfig.h>
#include <stdio.h>

#include <numeric>
#include <vector>

#include "BL48250.h"
#include "IMUGY85.h"
#include "NVSManager.h"
#include "mirf.h"
#include "omni_robot.h"
#include "wheel_state_estimator.h"

static const char * TAG = "MAIN";

i2c_master_bus_handle_t bus_handle;

// Tasks Configuration
#define IMU_TASK_RATE_HZ 100
#define SERIAL_PRINT_RATE_HZ 20

IMUGY85 imu;
SemaphoreHandle_t data_mutex;

// Shared Data Container
struct SharedData
{
    double ax, ay, az;
    double gx, gy, gz;
    double mx, my, mz;
    double roll, pitch, yaw;
    double azimuth;
    char mag_dir[4];
} imu_data;

void setup_i2c()
{
    i2c_master_bus_config_t i2c_mst_config = {.i2c_port = (i2c_port_t)CONFIG_I2C_PORT_NUM,
                                              .sda_io_num = (gpio_num_t)CONFIG_SDA_GPIO,
                                              .scl_io_num = (gpio_num_t)CONFIG_SCL_GPIO,
                                              .clk_source = I2C_CLK_SRC_DEFAULT,
                                              .glitch_ignore_cnt = 7,
                                              .intr_priority = 0,
                                              .trans_queue_depth = 0,
                                              .flags = {.enable_internal_pullup = 1, .allow_pd = 0}};

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    I2Cdev::init(bus_handle);
    ESP_LOGI(TAG, "I2C Initialized");
}

void imu_update_task(void * pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / IMU_TASK_RATE_HZ);
    xLastWakeTime = xTaskGetTickCount();

    while (true)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        imu.update();

        if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            imu.get_acceleration(&imu_data.ax, &imu_data.ay, &imu_data.az);
            imu.get_gyro(&imu_data.gx, &imu_data.gy, &imu_data.gz);
            imu.get_magnetometer(&imu_data.mx, &imu_data.my, &imu_data.mz);

            imu_data.roll = imu.get_roll();
            imu_data.pitch = imu.get_pitch();
            imu_data.yaw = imu.get_yaw();

            imu_data.azimuth = imu.mag.get_azimuth();
            imu.mag.get_direction(imu_data.mag_dir, imu_data.azimuth);

            xSemaphoreGive(data_mutex);
        }
    }
}

void heartbeat_task(void * pvParam)
{
    ledc_timer_config_t timer_config = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                        .duty_resolution = LEDC_TIMER_10_BIT,
                                        .timer_num = LEDC_TIMER_0,
                                        .freq_hz = 1,
                                        .clk_cfg = LEDC_AUTO_CLK,
                                        .deconfigure = false};

    ledc_timer_config(&timer_config);

    ledc_channel_config_t channel_config = {.gpio_num = (gpio_num_t)CONFIG_BLINK_GPIO,
                                            .speed_mode = LEDC_LOW_SPEED_MODE,
                                            .channel = LEDC_CHANNEL_0,
                                            .intr_type = LEDC_INTR_DISABLE,
                                            .timer_sel = LEDC_TIMER_0,
                                            .duty = 1UL << (timer_config.duty_resolution - 1),
                                            .hpoint = 0,
                                            .sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE,
                                            .flags = {.output_invert = 0}};

    ledc_channel_config(&channel_config);
    vTaskDelete(nullptr);
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting Oxebots SSL Firmware...");

    ESP_ERROR_CHECK(NVSManager::init());

    setup_i2c();

    data_mutex = xSemaphoreCreateMutex();

    const std::array<adc_channel_t, 4> wheel_adc_channels = {
      static_cast<adc_channel_t>(CONFIG_MOTOR_FL_ENC_CHANNEL), static_cast<adc_channel_t>(CONFIG_MOTOR_BL_ENC_CHANNEL),
      static_cast<adc_channel_t>(CONFIG_MOTOR_BR_ENC_CHANNEL), static_cast<adc_channel_t>(CONFIG_MOTOR_FR_ENC_CHANNEL)};

    WheelStateEstimator & w_state_estimator = WheelStateEstimator::get_instance();

    // Tie the central NVSManager locks to the wheel estimator's task to safely halt the ADC during flash commits
    NVSManager::set_adc_callbacks([&w_state_estimator]() { w_state_estimator.suspend(); },
                                  [&w_state_estimator]() { w_state_estimator.resume(); });

    ESP_LOGI(TAG, "Initializing Wheel State Estimator...");
    ESP_ERROR_CHECK(w_state_estimator.init(wheel_adc_channels, ADC_ATTEN_DB_12));

    ESP_LOGI(TAG, "Initializing IMU...");
    imu.init();
    struct SharedData local_data;

    xTaskCreate(heartbeat_task, "LED Blink", configMINIMAL_STACK_SIZE * 2, nullptr, 5, nullptr);
    xTaskCreate(imu_update_task, "IMU Update", configMINIMAL_STACK_SIZE * 2, nullptr, 5, nullptr);

    ESP_LOGI(TAG, "Starting sensor calibration check...");

    // Wheel Calibration
    // w_state_estimator.configure_encoder_i2c(0);  // one-time config
    ESP_LOGI(TAG, "Checking wheel encoders...");
    w_state_estimator.load_or_calibrate(5000);

    // Magnetometer Calibration
    ESP_LOGI(TAG, "Checking magnetometer calibration...");

    if (imu.load_or_calibrate_mag(10) != ESP_OK)
        ESP_LOGE(TAG, "Magnetometer calibration failed or timed out!");
    else
        ESP_LOGI(TAG, "Magnetometer is ready.");

    // Main log loop
    while (true)
    {
        const std::array<float, NUM_ENC_CHANNELS> angles_rad = w_state_estimator.get_filtered_angle_rad();
        const std::array<float, NUM_ENC_CHANNELS> angles_deg = w_state_estimator.get_filtered_angle_deg();
        const std::array<float, NUM_ENC_CHANNELS> rpms = w_state_estimator.get_filtered_rpm();
        const std::array<float, NUM_ENC_CHANNELS> accels = w_state_estimator.get_filtered_acceleration_rps2();

        for (int i = 0; i < NUM_ENC_CHANNELS; ++i)
        {
            printf(">w_%d_rad:%f\n", i + 1, angles_rad[i]);
            printf(">w_%d_deg:%f\n", i + 1, angles_deg[i]);
            printf(">w_%d_rpm:%f\n", i + 1, rpms[i]);
            printf(">w_%d_acc:%f\n", i + 1, accels[i]);
        }

        if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            local_data = imu_data;
            xSemaphoreGive(data_mutex);
        }

        printf(">a.x:%.2f\n", local_data.ay);
        printf(">a.y:%.2f\n", -local_data.ax);
        printf(">a.z:%.2f\n", local_data.az);
        printf(">g.x:%.2f\n", local_data.gx);
        printf(">g.y:%.2f\n", local_data.gy);
        printf(">g.z:%.2f\n", local_data.gz);
        printf(">m.x:%.2f\n", local_data.mx);
        printf(">m.y:%.2f\n", local_data.my);
        printf(">m.z:%.2f\n", local_data.mz);
        printf(">m.a:%.2f\n", local_data.azimuth);
        printf("Mag dir: %s\n", local_data.mag_dir);

        printf(">pose.roll:%.2f\n", local_data.roll);
        printf(">pose.pitch:%.2f\n", local_data.pitch);
        printf(">pose.yaw:%.2f\n", local_data.yaw);

        float rad_roll = local_data.roll * M_PI / 180.0f;
        float rad_pitch = local_data.pitch * M_PI / 180.0f;
        float rad_yaw = local_data.yaw * M_PI / 180.0f;

        printf(">3D|IMU:R:%.4f:%.4f:%.4f:S:cube:W:3:H:1.5:D:4:C:grey|g\n",
               rad_roll,   // X Rotation
               rad_pitch,  // Y Rotation
               rad_yaw    // Z Rotation
        );

        vTaskDelay(pdMS_TO_TICKS(1000 / SERIAL_PRINT_RATE_HZ));
    }
}
