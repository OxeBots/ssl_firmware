/**
 * @file SerialDebug.cpp
 * @brief Serial debug dashboard service: aggregates firmware state and
 *        streams it as binary frames over UART0 while a dashboard client
 *        keeps the connection alive.
 *
 * Stream contents (rates chosen for a 115200 8N1 console):
 *   - DebugState  at 25 Hz: four wheel PWM/velocity/target + attitude.
 *   - DebugImu    at 10 Hz: calibrated accelerometer/gyro/magnetometer.
 *   - DebugTof    per new frame: one 8x8 VL53L5CX range frame.
 *   - DebugHealth at 2 Hz: safety lifecycle, faults, supply, heap.
 *   - DebugText   per log line: ESP_LOG output captured by a vprintf hook.
 *
 * The dashboard drives everything with DebugControl frames; without a
 * keepalive every 1.5 s the stream stops, so unplugging or closing the port
 * always ends the debug dump.
 */

// Includes stay outside the conditional so the PlatformIO library finder
// sees the component dependencies even when the dashboard is disabled.
#include "SerialDebug.h"

#include <driver/uart.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <sdkconfig.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "OrientationHandler.h"
#include "ProtocolHandler.h"
#include "RobotState.h"
#include "WheelController.h"
#include "constants.h"
#include "serial_debug_protocol_bp.h"
#include "wheel_state_estimator.h"

#if CONFIG_SERIAL_DEBUG

static const char * TAG = "SerialDebug";

// Original console vprintf, restored output when not streaming.
static vprintf_like_t s_original_vprintf = nullptr;

SerialDebug & SerialDebug::get_instance()
{
    static SerialDebug instance;
    return instance;
}

esp_err_t SerialDebug::init(ToFPerceptionService * tof_service)
{
    if (m_tx_mutex != nullptr)
        return ESP_ERR_INVALID_STATE;
    m_tof_service = tof_service;

    // Recursive: the log hook holds it across line assembly and then sends
    // DebugText frames through the same critical section.
    m_tx_mutex = xSemaphoreCreateRecursiveMutex();
    if (m_tx_mutex == nullptr)
        return ESP_ERR_NO_MEM;

    // UART0 is the console: the driver adds buffered RX (control frames)
    // and buffered TX (stream frames) on top of it.
    if (uart_driver_install(UART_NUM_0, 1024, 1024, 0, nullptr, 0) != ESP_OK)
        return ESP_FAIL;

    // VMON supply sensing for the health stream.
    adc_oneshot_unit_init_cfg_t unit_config = {
      .unit_id = ADC_UNIT_1,
      .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    if (adc_oneshot_new_unit(&unit_config, &m_adc_handle) == ESP_OK)
    {
        adc_oneshot_chan_cfg_t channel_config = {
          .atten = ADC_ATTEN_DB_12,
          .bitwidth = ADC_BITWIDTH_12,
        };
        const adc_channel_t channel = static_cast<adc_channel_t>(CONFIG_VMON_ADC_CHANNEL);
        if (adc_oneshot_config_channel(m_adc_handle, channel, &channel_config) == ESP_OK)
        {
            adc_cali_line_fitting_config_t cali_config = {
              .unit_id = ADC_UNIT_1,
              .atten = ADC_ATTEN_DB_12,
              .bitwidth = ADC_BITWIDTH_12,
              .default_vref = ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF,
            };
            m_adc_ready = adc_cali_create_scheme_line_fitting(&cali_config, &m_adc_cali) == ESP_OK;
        }
    }
    if (!m_adc_ready)
        ESP_LOGW(TAG, "VMON ADC unavailable, supply voltage will read 0 V.");

    PowerMonitorConfig power_config;
    power_config.adc_to_supply_ratio = static_cast<float>(CONFIG_VMON_SUPPLY_RATIO_MILLI) / 1000.0f;
    power_config.undervoltage_v = static_cast<float>(CONFIG_VMON_UNDERVOLTAGE_MV) / 1000.0f;
    power_config.overvoltage_v = static_cast<float>(CONFIG_VMON_OVERVOLTAGE_MV) / 1000.0f;
    m_power_monitor = PowerMonitor(power_config);

    // Capture ESP_LOG output; forwarded to the console when not streaming.
    s_original_vprintf = esp_log_set_vprintf(&SerialDebug::log_hook);

    ESP_LOGI(TAG, "Serial debug service ready; streams when a dashboard subscribes.");

    return ESP_OK;
}

void SerialDebug::start_task()
{
    if (xTaskCreate(&SerialDebug::task_wrapper, "serial_debug", 4096, this, 3, &m_task_handle) !=
        pdPASS)
        ESP_LOGE(TAG, "Failed to create the serial debug task.");
}

void SerialDebug::task_wrapper(void * arg)
{
    static_cast<SerialDebug *>(arg)->task_loop();
}

void SerialDebug::task_loop()
{
    uint8_t rx[128];
    uint32_t rx_bytes_total = 0;
    uint32_t rx_bytes_interval = 0;
    uint64_t last_rx_report_us = 0;
    while (true)
    {
        const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
        const uint32_t now_ms = static_cast<uint32_t>(now_us / 1000ULL);

        // Drain received control frames.
        int n = uart_read_bytes(UART_NUM_0, rx, sizeof(rx), 0);
        if (n > 0)
        {
            rx_bytes_total += static_cast<uint32_t>(n);
            rx_bytes_interval += static_cast<uint32_t>(n);
        }
        // Periodic RX diagnostic while idle: proves whether host bytes reach
        // the firmware at all (a serial monitor attached on the host side
        // still lets RX through - only the device TX is split between them).
        if (!m_streaming.load() && now_us - last_rx_report_us >= RX_REPORT_INTERVAL_US)
        {
            last_rx_report_us = now_us;
            ESP_LOGI(TAG,
                     "RX diagnostic: %lu total, %lu in the last interval.",
                     (unsigned long)rx_bytes_total,
                     (unsigned long)rx_bytes_interval);
            rx_bytes_interval = 0;
        }
        for (int i = 0; i < n; ++i)
        {
            m_decoder.feed(rx[i]);
            if (m_decoder.has_frame())
            {
                handle_control_frame();
                m_decoder.pop();
            }
        }

        if (m_streaming.load())
        {
            // Signed delta: a control frame handled this iteration stamps
            // m_last_keepalive_us with a reading newer than now_us (taken at
            // loop top), which would wrap an unsigned subtraction and look
            // like a timeout.
            const int64_t since_keepalive_us =
              static_cast<int64_t>(now_us) - static_cast<int64_t>(m_last_keepalive_us);
            if (since_keepalive_us > static_cast<int64_t>(KEEPALIVE_TIMEOUT_US))
            {
                m_streaming.store(false);
                ESP_LOGW(TAG, "Dashboard keepalive timed out, streaming stopped.");
            }
            else
            {
                if (now_ms - m_last_state_ms >= STATE_PERIOD_MS)
                {
                    m_last_state_ms = now_ms;
                    send_state(now_ms);
                }
                if (now_ms - m_last_imu_ms >= IMU_PERIOD_MS)
                {
                    m_last_imu_ms = now_ms;
                    send_imu();
                }
                if (now_ms - m_last_health_ms >= HEALTH_PERIOD_MS)
                {
                    m_last_health_ms = now_ms;
                    send_health();
                }
                send_tof();
            }
        }

        // CONFIG_FREERTOS_HZ=100 makes pdMS_TO_TICKS(4) round to zero,
        // which would spin the task and starve the CPU1 idle task (task
        // watchdog): always delay at least one tick.
        TickType_t delay_ticks = pdMS_TO_TICKS(4);
        if (delay_ticks == 0)
            delay_ticks = 1;
        vTaskDelay(delay_ticks);
    }
}

void SerialDebug::handle_control_frame()
{
    if (m_decoder.message_type() != DBG_MSG_CONTROL)
        return;
    if (m_decoder.message_length() != BYTES_LENGTH_DEBUG_CONTROL)
        return;

    // Decoders use bitwise OR: always decode into a zero-initialized struct.
    DebugControl control;
    memset(&control, 0, sizeof(control));
    DecodeDebugControl(&control, const_cast<uint8_t *>(m_decoder.message()));

    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    m_last_keepalive_us = now_us;

    // While streaming this becomes a DebugText frame on the dashboard
    // console; while idle it shows on the serial monitor, which proves the
    // control path works. Keepalives arrive at ~1 Hz and only say "still
    // alive", so logging them would flood the dashboard console.
    if (control.op != DBG_CONTROL_KEEPALIVE)
        ESP_LOGI(TAG, "Debug control frame received (op=%d).", control.op);

    switch (control.op)
    {
        case DBG_CONTROL_START_STREAM:
            m_last_state_ms = 0;
            m_last_imu_ms = 0;
            m_last_health_ms = 0;
            m_state_sequence = 0;
            m_imu_sequence = 0;
            m_health_sequence = 0;
            m_tof_frame_sent = false;
            m_streaming.store(true);
            ESP_LOGI(TAG, "Dashboard connected, streaming started.");
            break;

        case DBG_CONTROL_STOP_STREAM:
            m_streaming.store(false);
            ESP_LOGI(TAG, "Dashboard disconnected, streaming stopped.");
            break;

        case DBG_CONTROL_KEEPALIVE:
            break;

        case DBG_CONTROL_SET_TARGETS:
        {
            std::array<float, 4> targets = {0.0f, 0.0f, 0.0f, 0.0f};
            for (size_t i = 0; i < 4; ++i)
            {
                const float rpm = static_cast<float>(control.wheel_targets[i]) / 10.0f;
                targets[i] = rpm * RPM_TO_RAD_S;
            }
            WheelController::get_instance().set_target_velocities(targets);
            ESP_LOGI(TAG,
                     "Wheel targets: %.1f %.1f %.1f %.1f RPM",
                     static_cast<double>(targets[0] * RAD_S_TO_RPM),
                     static_cast<double>(targets[1] * RAD_S_TO_RPM),
                     static_cast<double>(targets[2] * RAD_S_TO_RPM),
                     static_cast<double>(targets[3] * RAD_S_TO_RPM));
            break;
        }

        case DBG_CONTROL_STOP_MOTORS:
            WheelController::get_instance().set_target_velocities({0.0f, 0.0f, 0.0f, 0.0f});
            ESP_LOGI(TAG, "Wheel targets zeroed by dashboard.");
            break;

        default:
            ESP_LOGW(TAG, "Unknown debug control op %d.", control.op);
            break;
    }
}

bool SerialDebug::send_frame(uint8_t msg_type, const void * message, size_t message_len)
{
    if (m_tx_mutex == nullptr)
        return false;

    uint8_t frame[serial_debug::MAX_FRAME_BYTES];
    const size_t frame_len = serial_debug::encode_frame(
      msg_type, static_cast<const uint8_t *>(message), message_len, frame, sizeof(frame));
    if (frame_len == 0)
        return false;

    if (xSemaphoreTakeRecursive(m_tx_mutex, pdMS_TO_TICKS(20)) != pdTRUE)
        return false;
    uart_write_bytes(UART_NUM_0, frame, frame_len);
    xSemaphoreGiveRecursive(m_tx_mutex);
    return true;
}

bool SerialDebug::send_text_line(const char * line, size_t length)
{
    if (m_tx_mutex == nullptr)
        return false;
    if (xSemaphoreTakeRecursive(m_tx_mutex, pdMS_TO_TICKS(20)) != pdTRUE)
        return false;

    // Zero-init before encode (bitproto uses bitwise OR).
    DebugText text;
    memset(&text, 0, sizeof(text));
    const size_t n = std::min(length, TEXT_MAX);
    memcpy(text.text, line, n);
    text.length = static_cast<uint8_t>(n);

    // Serialize with the generated bitproto encoder: the C struct carries
    // compiler padding, so its raw memory is not the wire format.
    uint8_t payload[BYTES_LENGTH_DEBUG_TEXT];
    EncodeDebugText(&text, payload);

    uint8_t frame[serial_debug::MAX_FRAME_BYTES];
    const size_t frame_len = serial_debug::encode_frame(
      DBG_MSG_TEXT, payload, BYTES_LENGTH_DEBUG_TEXT, frame, sizeof(frame));
    if (frame_len != 0)
        uart_write_bytes(UART_NUM_0, frame, frame_len);
    xSemaphoreGiveRecursive(m_tx_mutex);
    return frame_len != 0;
}

int SerialDebug::log_hook(const char * format, va_list args)
{
    SerialDebug & self = get_instance();

    // Format once; the copy keeps va_list reusable for the console path.
    char raw[LOG_LINE_CAP];
    va_list args_copy;
    va_copy(args_copy, args);
    int written = vsnprintf(raw, sizeof(raw), format, args_copy);
    va_end(args_copy);
    if (written < 0)
        written = 0;
    if (written >= static_cast<int>(sizeof(raw)))
        written = static_cast<int>(sizeof(raw)) - 1;

    if (!self.m_streaming.load())
    {
        // Console path: still emit through the UART driver instead of the
        // raw FIFO, so log text can never splice into a frame that the
        // driver is transmitting. The trailing delimiter closes the text
        // as its own decoder chunk, so the next frame starts clean.
        if (xSemaphoreTakeRecursive(self.m_tx_mutex, pdMS_TO_TICKS(20)) == pdTRUE)
        {
            uart_write_bytes(UART_NUM_0, raw, written);
            uart_write_bytes(UART_NUM_0, "\x00", 1);
            xSemaphoreGiveRecursive(self.m_tx_mutex);
        }
        return written;
    }

    // Append to the pending line, stripping ANSI colors, and emit a frame
    // for every completed line.
    if (xSemaphoreTakeRecursive(self.m_tx_mutex, pdMS_TO_TICKS(20)) != pdTRUE)
        return written;
    for (int i = 0; i < written; ++i)
    {
        const char c = raw[i];
        if (self.m_log_escape_left > 0)
        {
            // Inside an ANSI escape sequence such as ESC [ 0 ; 32 m.
            --self.m_log_escape_left;
            if (c == 'm')
                self.m_log_escape_left = 0;
            continue;
        }
        if (c == 0x1B)
        {
            self.m_log_escape_left = 8;  // Bounded: sequences are far shorter
            continue;
        }
        if (c == '\n' || c == '\r')
        {
            if (self.m_log_line_len > 0)
            {
                self.send_text_line(self.m_log_line, self.m_log_line_len);
                self.m_log_line_len = 0;
            }
            continue;
        }
        if (self.m_log_line_len >= TEXT_MAX)
        {
            // Line longer than one frame: flush it and keep the tail.
            self.send_text_line(self.m_log_line, self.m_log_line_len);
            self.m_log_line_len = 0;
        }
        self.m_log_line[self.m_log_line_len++] = c;
    }
    xSemaphoreGiveRecursive(self.m_tx_mutex);
    return written;
}

void SerialDebug::send_state(uint32_t now_ms)
{
    // Zero-init before encode (bitproto uses bitwise OR).
    DebugState state;
    memset(&state, 0, sizeof(state));

    auto & wheel_ctrl = WheelController::get_instance();
    const auto duty = wheel_ctrl.get_pid_outputs();
    const auto velocity = wheel_ctrl.get_current_velocities();
    const auto target = wheel_ctrl.get_target_velocities();
    auto & estimator = WheelStateEstimator::get_instance();

    for (size_t i = 0; i < 4; ++i)
    {
        state.wheels[i].pwm = static_cast<int16_t>(std::lround(duty[i]));
        state.wheels[i].velocity =
          static_cast<int16_t>(std::lround(velocity[i] * RAD_S_TO_RPM * 10.0f));
        state.wheels[i].target =
          static_cast<int16_t>(std::lround(target[i] * RAD_S_TO_RPM * 10.0f));
        if (estimator.is_channel_calibrated(i))
            state.encoder_valid_mask |= static_cast<uint8_t>(1u << i);
    }

    auto & orientation = OrientationHandler::get_instance();
    state.yaw = static_cast<int16_t>(std::lround(orientation.get_yaw() * 100.0));
    state.pitch = static_cast<int16_t>(std::lround(orientation.get_pitch() * 100.0));
    state.roll = static_cast<int16_t>(std::lround(orientation.get_roll() * 100.0));

    state.uptime_ms = now_ms;
    state.sequence = ++m_state_sequence;

    uint8_t payload[BYTES_LENGTH_DEBUG_STATE];
    EncodeDebugState(&state, payload);
    send_frame(DBG_MSG_STATE, payload, BYTES_LENGTH_DEBUG_STATE);
}

void SerialDebug::send_imu()
{
    DebugImu imu;
    memset(&imu, 0, sizeof(imu));

    // Calibrated outputs: acceleration in g, rotation in deg/s, field in mG.
    double ax = 0.0, ay = 0.0, az = 0.0;
    double gx = 0.0, gy = 0.0, gz = 0.0;
    double mx = 0.0, my = 0.0, mz = 0.0;
    auto & orientation = OrientationHandler::get_instance();
    orientation.get_acceleration(&ax, &ay, &az);
    orientation.get_gyro(&gx, &gy, &gz);
    orientation.get_magnetometer(&mx, &my, &mz);

    imu.accel_x = static_cast<int16_t>(std::lround(ax * 1000.0));
    imu.accel_y = static_cast<int16_t>(std::lround(ay * 1000.0));
    imu.accel_z = static_cast<int16_t>(std::lround(az * 1000.0));
    imu.gyro_x = static_cast<int16_t>(std::lround(gx * 10.0));
    imu.gyro_y = static_cast<int16_t>(std::lround(gy * 10.0));
    imu.gyro_z = static_cast<int16_t>(std::lround(gz * 10.0));
    imu.mag_x = static_cast<int16_t>(std::lround(mx));
    imu.mag_y = static_cast<int16_t>(std::lround(my));
    imu.mag_z = static_cast<int16_t>(std::lround(mz));
    imu.sequence = ++m_imu_sequence;

    uint8_t payload[BYTES_LENGTH_DEBUG_IMU];
    EncodeDebugImu(&imu, payload);
    send_frame(DBG_MSG_IMU, payload, BYTES_LENGTH_DEBUG_IMU);
}

void SerialDebug::send_tof()
{
    if (m_tof_service == nullptr)
        return;

    ToFPerceptionFrame frame;
    if (!m_tof_service->get_latest_frame(&frame) || !frame.valid)
        return;
    if (m_tof_frame_sent && frame.sequence == m_last_tof_sequence)
        return;  // No new measurement since the last frame we sent.
    m_last_tof_sequence = frame.sequence;
    m_tof_frame_sent = true;

    DebugTof tof;
    memset(&tof, 0, sizeof(tof));
    tof.sequence = frame.sequence;
    for (size_t zone = 0; zone < 64; ++zone)
    {
        const size_t index = zone * VL53L5CX_NB_TARGET_PER_ZONE;
        const bool has_target = frame.target_status[index] != 0;
        tof.distance_mm[zone] = has_target ? frame.distance_mm[index] : 0;
    }
    uint8_t payload[BYTES_LENGTH_DEBUG_TOF];
    EncodeDebugTof(&tof, payload);
    send_frame(DBG_MSG_TOF, payload, BYTES_LENGTH_DEBUG_TOF);
}

bool SerialDebug::read_supply_voltage(uint64_t now_us)
{
    if (!m_adc_ready)
        return false;

    int raw = 0;
    const adc_channel_t channel = static_cast<adc_channel_t>(CONFIG_VMON_ADC_CHANNEL);
    if (adc_oneshot_read(m_adc_handle, channel, &raw) != ESP_OK)
        return false;

    int millivolt = 0;
    if (m_adc_cali != nullptr && adc_cali_raw_to_voltage(m_adc_cali, raw, &millivolt) != ESP_OK)
        millivolt = raw * 3300 / 4095;  // Uncalibrated fallback
    return m_power_monitor.update_adc_voltage(static_cast<float>(millivolt) / 1000.0f, now_us);
}

void SerialDebug::send_health()
{
    DebugHealth health;
    memset(&health, 0, sizeof(health));

    auto & robot_state = RobotState::get_instance();
    health.robot_id = robot_state.get_id();
    health.error_flags = robot_state.get_error_flags();

    auto & protocol = ProtocolHandler::get_instance();
    health.lifecycle = static_cast<uint8_t>(protocol.get_lifecycle());
    const HealthState & safety = protocol.get_health_state();
    health.active_faults = safety.active_faults;
    health.latched_faults = safety.latched_faults;
    health.motion_inhibited = safety.motion_inhibited ? 1 : 0;
    health.kick_inhibited = safety.kick_inhibited ? 1 : 0;

    auto & orientation = OrientationHandler::get_instance();
    uint8_t cal_flags = 0;
    if (orientation.is_accel_calibrated())
        cal_flags |= 1;
    if (orientation.is_gyro_calibrated())
        cal_flags |= 2;
    if (orientation.is_mag_calibrated())
        cal_flags |= 4;
    health.imu_cal_flags = cal_flags;

    health.tof_available = (m_tof_service != nullptr && m_tof_service->is_available()) ? 1 : 0;

    read_supply_voltage(static_cast<uint64_t>(esp_timer_get_time()));
    health.supply_voltage =
      static_cast<uint16_t>(std::lroundf(m_power_monitor.get_supply_voltage_v() * 100.0f));
    uint8_t supply_flags = 0;
    if (m_power_monitor.is_valid())
        supply_flags |= 1;
    if (m_power_monitor.is_undervoltage())
        supply_flags |= 2;
    if (m_power_monitor.is_overvoltage())
        supply_flags |= 4;
    health.supply_flags = supply_flags;

    health.free_heap = esp_get_free_heap_size();
    health.min_free_heap = esp_get_minimum_free_heap_size();
    health.sequence = ++m_health_sequence;

    uint8_t payload[BYTES_LENGTH_DEBUG_HEALTH];
    EncodeDebugHealth(&health, payload);
    send_frame(DBG_MSG_HEALTH, payload, BYTES_LENGTH_DEBUG_HEALTH);
}

#endif  // CONFIG_SERIAL_DEBUG
