#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <sdkconfig.h>
#include <stdio.h>

#include <numeric>
#include <vector>

#include "BL48250.h"
#include "IMUGY85.h"
#include "NRF24L01.h"
#include "NVSManager.h"
#include "mirf.h"
#include "omni_robot.h"
#include "ssl_robot_protocol_bp.h"
#include "wheel_state_estimator.h"

static const char * TAG = "MAIN";

// NVS namespace / key for robot configuration
static constexpr const char * ROBOT_CFG_NVS_NS = "robot_cfg";
static constexpr const char * ROBOT_CFG_KEY_ID = "robot_id";

// Runtime robot ID — loaded from NVS at startup, updated via config messages
static uint8_t g_robot_id = 0;

// ---------------------------------------------------------------------------
// Deferred NVS write queue — radio callback posts here, a dedicated NVS task
// commits immediately.  Using a separate task avoids the mutex priority
// inheritance crash (xTaskPriorityDisinherit).
// ---------------------------------------------------------------------------
struct NvsWriteRequest
{
    char ns[16];
    char key[16];
    int32_t value;
};
static QueueHandle_t g_nvs_write_queue = nullptr;

static void nvs_writer_task(void * /*arg*/)
{
    NvsWriteRequest req;
    while (true)
    {
        if (xQueueReceive(g_nvs_write_queue, &req, portMAX_DELAY) == pdTRUE)
        {
            // Use save_i32_direct to bypass the ADC suspend/resume callbacks.
            // Those callbacks use ESP-IDF IPC calls that are unsafe from this
            // task context and cause xTaskPriorityDisinherit crashes.
            NVSManager::save_i32_direct(req.ns, req.key, req.value);
        }
    }
}

i2c_master_bus_handle_t bus_handle;

#define IMU_TASK_RATE_HZ 100
#define SERIAL_PRINT_RATE_HZ 20

IMUGY85 imu;
NRF24L01 radio(static_cast<gpio_num_t>(CONFIG_IRQ_GPIO));
SemaphoreHandle_t data_mutex;

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

static void load_robot_id_from_nvs()
{
    int32_t stored_id = 0;
    esp_err_t err = NVSManager::load_i32(ROBOT_CFG_NVS_NS, ROBOT_CFG_KEY_ID, &stored_id);

    if (err == ESP_OK)
    {
        g_robot_id = static_cast<uint8_t>(stored_id);
        ESP_LOGI(TAG, "Loaded robot ID from NVS: %d", g_robot_id);
    }
    else
    {
        g_robot_id = 0;
        ESP_LOGW(TAG, "No robot ID in NVS — defaulting to ID 0.");
    }
}

static void save_robot_id_to_nvs(uint8_t id)
{
    // Post to the deferred queue so the main loop does the actual NVS write.
    // Direct NVS calls from the radio receiver task cause mutex priority
    // inheritance crashes (xTaskPriorityDisinherit assert in tasks.c).
    NvsWriteRequest req{};
    strncpy(req.ns, ROBOT_CFG_NVS_NS, sizeof(req.ns) - 1);
    strncpy(req.key, ROBOT_CFG_KEY_ID, sizeof(req.key) - 1);
    req.value = static_cast<int32_t>(id);

    if (xQueueSend(g_nvs_write_queue, &req, pdMS_TO_TICKS(100)) == pdTRUE)
        ESP_LOGI(TAG, "Queued NVS write: robot_id=%d", id);
    else
        ESP_LOGE(TAG, "NVS write queue full — robot_id=%d not persisted!", id);
}

static void send_telemetry_response(uint32_t echo_timestamp)
{
    RobotTelemetry tel;
    memset(&tel, 0, sizeof(RobotTelemetry));

    tel.header.msg_type = MSG_TYPE_TELEMETRY;
    tel.header.robot_id = g_robot_id;
    tel.header.timestamp = echo_timestamp;

    tel.robot_pose.x = 0;
    tel.robot_pose.y = 0;
    tel.robot_pose.angle = static_cast<int16_t>(imu_data.yaw * 100);

    tel.battery_percentage = 95;
    tel.kicker_voltage = 1650;
    tel.error_flags = 0;

    uint8_t buf[BYTES_LENGTH_ROBOT_TELEMETRY] = {0};
    EncodeRobotTelemetry(&tel, buf);
    radio.send_raw(buf, BYTES_LENGTH_ROBOT_TELEMETRY);
}

static void handle_config_message(const RobotConfig * cfg)
{
    const uint8_t target = cfg->header.robot_id;

    if (target != g_robot_id && target != ROBOT_ID_BROADCAST)
    {
        ESP_LOGD(TAG, "Config ignored — addressed to robot %d, we are %d.", target, g_robot_id);
        return;
    }

    ESP_LOGI(TAG, "Handling config flags=0x%02X param=%d", cfg->config_flags, cfg->param);

    if (cfg->config_flags & CONFIG_FLAG_SET_ID)
    {
        uint8_t new_id = static_cast<uint8_t>(cfg->param & 0xFF);
        if (new_id > ROBOT_ID_MAX)
        {
            ESP_LOGE(TAG, "CONFIG_FLAG_SET_ID: requested ID %d exceeds ROBOT_ID_MAX (%d). Rejected.", new_id,
                     ROBOT_ID_MAX);
        }
        else
        {
            ESP_LOGI(TAG, "Setting robot ID: %d → %d", g_robot_id, new_id);
            g_robot_id = new_id;
            save_robot_id_to_nvs(g_robot_id);
            ESP_LOGW(TAG, "Robot ID changed.");
        }
    }

    if (cfg->config_flags & CONFIG_FLAG_RUN_CALIBRATION)
    {
        ESP_LOGI(TAG, "CONFIG_FLAG_RUN_CALIBRATION: triggering wheel encoder calibration.");
        WheelStateEstimator::get_instance().load_or_calibrate(5000);
        // TODO: calibrate all subsystems, not just encoders
    }

    if (cfg->config_flags & CONFIG_FLAG_RESET_NVS)
    {
        ESP_LOGW(TAG, "CONFIG_FLAG_RESET_NVS: resetting robot ID to 0.");
        g_robot_id = 0;
        save_robot_id_to_nvs(g_robot_id);
        // TODO: erase all NVS keys, not just robot ID
    }
}

static void handle_radio_packet(const uint8_t * payload, uint8_t len)
{
    // Every valid packet must start with a Header (5 bytes).
    if (len < BYTES_LENGTH_HEADER)
    {
        ESP_LOGW(TAG, "Packet too short for header (%d bytes). Discarding.", len);
        return;
    }

    // IMPORTANT: bitproto C decoders use bitwise OR to write decoded bits into
    // struct fields — they never zero the destination first.  Every Decode*
    // call must therefore target a zero-initialised struct, otherwise garbage
    // stack bits bleed into fields whose encoded value contains zeros.
    Header hdr;
    memset(&hdr, 0, sizeof(Header));
    DecodeHeader(&hdr, const_cast<uint8_t *>(payload));

    const uint8_t msg_type = hdr.msg_type;
    const uint8_t robot_id = hdr.robot_id;

    switch (msg_type)
    {
        case MSG_TYPE_COMMAND:
            if (len < BYTES_LENGTH_ROBOT_COMMAND)
            {
                ESP_LOGW(TAG, "MSG_TYPE_COMMAND: packet too short (%d / %d bytes).", len, BYTES_LENGTH_ROBOT_COMMAND);
                return;
            }

            if (robot_id != g_robot_id && robot_id != ROBOT_ID_BROADCAST)
            {
                ESP_LOGD(TAG, "Command ignored — addressed to robot %d, we are %d.", robot_id, g_robot_id);
                return;
            }

            RobotCommand cmd;
            memset(&cmd, 0, sizeof(RobotCommand));
            DecodeRobotCommand(&cmd, const_cast<uint8_t *>(payload));

            ESP_LOGI(TAG, "CMD robot=%d ts=%lu x=%d y=%d kick=%d", cmd.header.robot_id,
                     (unsigned long)cmd.header.timestamp, cmd.target_pose.x, cmd.target_pose.y, cmd.kick_velocity);

            // TODO: feed cmd into motion controller here

            if (robot_id == ROBOT_ID_BROADCAST)
                return;  // Broadcast: execute without reply

            send_telemetry_response(cmd.header.timestamp);
            break;

        case MSG_TYPE_CONFIG:
            if (len < BYTES_LENGTH_ROBOT_CONFIG)
            {
                ESP_LOGW(TAG, "MSG_TYPE_CONFIG: packet too short (%d / %d bytes).", len, BYTES_LENGTH_ROBOT_CONFIG);
                return;
            }

            // Config messages do not generate a reply
            RobotConfig cfg;
            memset(&cfg, 0, sizeof(RobotConfig));
            DecodeRobotConfig(&cfg, const_cast<uint8_t *>(payload));
            handle_config_message(&cfg);
            break;

        case MSG_TYPE_TELEMETRY:
            // Another robot's telemetry reply received on the shared channel — ignore.
            ESP_LOGD(TAG, "Received telemetry from robot %d — not our concern, ignoring.", robot_id);
            break;

        default:
            ESP_LOGW(TAG, "Unknown msg_type=0x%02X — discarding.", msg_type);
            break;
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting Oxebots SSL Firmware...");

    ESP_ERROR_CHECK(NVSManager::init());

    load_robot_id_from_nvs();
    ESP_LOGI(TAG, "Robot ID: %d", g_robot_id);

    // Mandatory ISR setup required by FreeRTOS GPIO Interrupts
    gpio_install_isr_service(0);

    setup_i2c();

    data_mutex = xSemaphoreCreateMutex();

    const std::array<adc_channel_t, 4> wheel_adc_channels = {
      static_cast<adc_channel_t>(CONFIG_MOTOR_FL_ENC_CHANNEL), static_cast<adc_channel_t>(CONFIG_MOTOR_BL_ENC_CHANNEL),
      static_cast<adc_channel_t>(CONFIG_MOTOR_BR_ENC_CHANNEL), static_cast<adc_channel_t>(CONFIG_MOTOR_FR_ENC_CHANNEL)};

    WheelStateEstimator & w_state_estimator = WheelStateEstimator::get_instance();
    NVSManager::set_adc_callbacks([&w_state_estimator]() { w_state_estimator.suspend(); },
                                  [&w_state_estimator]() { w_state_estimator.resume(); });

    g_nvs_write_queue = xQueueCreate(8, sizeof(NvsWriteRequest));
    configASSERT(g_nvs_write_queue);

    ESP_LOGI(TAG, "Initializing Radio Communication...");
    if (radio.init(CONFIG_RADIO_CHANNEL, 32, "ADMIN", "ESP32") == ESP_OK)
        radio.start(handle_radio_packet);
    else
        ESP_LOGE(TAG, "Radio initialization failed! Proceeding without radio link.");

    ESP_LOGI(TAG, "Initializing Wheel State Estimator...");
    ESP_ERROR_CHECK(w_state_estimator.init(wheel_adc_channels, ADC_ATTEN_DB_12));

    ESP_LOGI(TAG, "Initializing IMU...");
    imu.init();

    ESP_LOGI(TAG, "Checking wheel encoders...");
    w_state_estimator.load_or_calibrate(5000);

    if (imu.load_or_calibrate_mag(10) != ESP_OK)
        ESP_LOGE(TAG, "Magnetometer calibration failed or timed out!");
    else
        ESP_LOGI(TAG, "Magnetometer is ready.");

    struct SharedData local_data;

    // Tasks
    xTaskCreate(nvs_writer_task, "nvs_task", 4096, nullptr, 4, nullptr);
    xTaskCreate(imu_update_task, "imu_task", configMINIMAL_STACK_SIZE * 2, nullptr, 5, nullptr);

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
        printf(">m.dir: %s\n", local_data.mag_dir);

        printf(">pose.roll:%.2f\n", local_data.roll);
        printf(">pose.pitch:%.2f\n", local_data.pitch);
        printf(">pose.yaw:%.2f\n", local_data.yaw);

        float rad_roll = local_data.roll * M_PI / 180.0f;
        float rad_pitch = local_data.pitch * M_PI / 180.0f;
        float rad_yaw = local_data.yaw * M_PI / 180.0f;

        printf(">3D|IMU:R:%.4f:%.4f:%.4f:S:cube:W:3:H:1.5:D:4:C:grey|g\n",
               rad_roll,   // X Rotation
               rad_pitch,  // Y Rotation
               rad_yaw     // Z Rotation
        );

        vTaskDelay(pdMS_TO_TICKS(1000 / SERIAL_PRINT_RATE_HZ));
    }
}
