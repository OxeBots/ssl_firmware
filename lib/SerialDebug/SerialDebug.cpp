/**
 * @file SerialDebug.cpp
 * @brief Serial debug dashboard service: aggregates firmware state and
 *        streams it as binary frames over UART0 while a dashboard client
 *        keeps the connection alive.
 *
 * Stream contents (rates chosen for a 115200 8N1 console):
 *   - DebugState  at 25 Hz: four wheel PWM/velocity/target/angle + attitude.
 *   - DebugImu    at 10 Hz: calibrated accelerometer/gyro/magnetometer.
 *   - DebugTof    per new frame: one 8x8 VL53L5CX range frame.
 *   - DebugHealth at 2 Hz: safety lifecycle, faults, radio link, supply, heap.
 *   - DebugCalibration at 5 Hz: motor/encoder wiring calibration progress.
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
#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <sdkconfig.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "MotorCalibration.h"
#include "OrientationHandler.h"
#include "PowerMonitor.h"
#include "ProtocolHandler.h"
#include "RobotState.h"
#include "WheelController.h"
#include "WheelTuning.h"
#include "constants.h"
#include "serial_debug_protocol_bp.h"
#include "wheel_state_estimator.h"

#if CONFIG_SERIAL_DEBUG

static const char * TAG = "SerialDebug";

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

    // Application logs use the same live session as sensor diagnostics.
    esp_log_set_vprintf(&SerialDebug::log_hook);

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

/**
 * @brief Check that a dashboard subscribed and its last heartbeat is recent.
 */
bool SerialDebug::session_active() const
{
    if (!m_streaming.load())
        return false;
    const uint64_t last_keepalive_us = m_last_keepalive_us.load();
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    return now_us - last_keepalive_us <= KEEPALIVE_TIMEOUT_US;
}

/**
 * @brief Drop the bench motion override and stop the wheels it was driving.
 *
 * The override only lifts the health gate for the dashboard target path; the
 * health state reported to the rest of the firmware is unchanged.
 */
void SerialDebug::clear_motion_override()
{
    if (!m_motion_override)
        return;
    m_motion_override = false;
    WheelController::get_instance().set_target_velocities({0.0f, 0.0f, 0.0f, 0.0f});
    ESP_LOGW(TAG, "Bench motion override cleared; wheel targets zeroed.");
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
        // Report received bytes only to a subscribed dashboard.
        if (session_active() && now_us - last_rx_report_us >= RX_REPORT_INTERVAL_US)
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
            if (!m_decoder.has_frame())
                continue;
            if (m_decoder.message_type() == DBG_MSG_POSITION_CONTROL)
                handle_position_frame();
            else
                handle_control_frame();
            m_decoder.pop();
        }

        if (m_streaming.load())
        {
            if (!session_active())
            {
                m_streaming.store(false);
                clear_motion_override();
                MotorCalibration::get_instance().abort();
                WheelTuning::get_instance().abort();
                WheelController::get_instance().stop_position_moves();
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
                if (now_ms - m_last_calibration_ms >= CALIBRATION_PERIOD_MS)
                {
                    m_last_calibration_ms = now_ms;
                    send_calibration();
                }
                if (now_ms - m_last_tuning_ms >= TUNING_PERIOD_MS)
                {
                    m_last_tuning_ms = now_ms;
                    send_tuning();
                }
                if (now_ms - m_last_position_ms >= POSITION_PERIOD_MS)
                {
                    m_last_position_ms = now_ms;
                    send_position();
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
    // Heartbeats do not need a console message for every received packet.
    if (control.op != DBG_CONTROL_KEEPALIVE && session_active())
        ESP_LOGI(TAG, "Debug control frame received (op=%d).", control.op);

    switch (control.op)
    {
        case DBG_CONTROL_START_STREAM:
            if (xSemaphoreTakeRecursive(m_tx_mutex, pdMS_TO_TICKS(20)) != pdTRUE)
                break;
            m_last_state_ms = 0;
            m_last_imu_ms = 0;
            m_last_health_ms = 0;
            m_last_calibration_ms = 0;
            m_last_tuning_ms = 0;
            m_last_position_ms = 0;
            m_state_sequence = 0;
            m_imu_sequence = 0;
            m_health_sequence = 0;
            m_calibration_sequence = 0;
            m_tuning_sequence = 0;
            m_position_sequence = 0;
            m_tof_frame_sent = false;
            m_log_line_len = 0;
            m_log_escape_left = 0;
            m_last_keepalive_us.store(now_us);
            m_streaming.store(true);
            xSemaphoreGiveRecursive(m_tx_mutex);
            clear_motion_override();
            ESP_LOGI(TAG, "Dashboard connected, streaming started.");
            break;

        case DBG_CONTROL_STOP_STREAM:
            m_streaming.store(false);
            clear_motion_override();
            MotorCalibration::get_instance().abort();
            WheelTuning::get_instance().abort();
            WheelController::get_instance().stop_position_moves();
            ESP_LOGI(TAG, "Dashboard disconnected, streaming stopped.");
            break;

        case DBG_CONTROL_START_CALIBRATION:
        {
            if (!session_active())
                break;
            MotorCalibration & calibration = MotorCalibration::get_instance();
            if (calibration.is_running())
            {
                ESP_LOGW(TAG, "Motor calibration already running; start ignored.");
                break;
            }
            if (WheelTuning::get_instance().is_running())
            {
                ESP_LOGW(TAG, "Wheel control calibration running; wiring calibration ignored.");
                break;
            }
            if (calibration.start() != ESP_OK)
                ESP_LOGE(TAG, "Failed to start the motor calibration.");
            else
                ESP_LOGW(TAG, "Motor calibration started: the wheels spin one at a time.");
            break;
        }

        case DBG_CONTROL_STOP_CALIBRATION:
            MotorCalibration::get_instance().abort();
            ESP_LOGW(TAG, "Motor calibration abort requested.");
            break;

        case DBG_CONTROL_START_TUNING:
        {
            if (!session_active())
                break;
            if (MotorCalibration::get_instance().is_running())
            {
                ESP_LOGW(TAG, "Wiring calibration running; control calibration ignored.");
                break;
            }
            WheelTuning & tuning = WheelTuning::get_instance();
            if (tuning.is_running())
            {
                ESP_LOGW(TAG, "Control calibration already running; start ignored.");
                break;
            }
            if (tuning.start() != ESP_OK)
                ESP_LOGE(TAG, "Failed to start the control calibration.");
            else
                ESP_LOGW(TAG, "Control calibration started: the wheels spin one at a time.");
            break;
        }

        case DBG_CONTROL_STOP_TUNING:
            WheelTuning::get_instance().abort();
            WheelController::get_instance().stop_position_moves();
            ESP_LOGW(TAG, "Control calibration abort requested.");
            break;

        case DBG_CONTROL_KEEPALIVE:
            if (session_active())
                m_last_keepalive_us.store(now_us);
            break;

        case DBG_CONTROL_SET_TARGETS:
        {
            if (MotorCalibration::get_instance().is_running())
            {
                ESP_LOGW(TAG, "Wheel targets rejected: motor calibration is running.");
                break;
            }
            if (WheelTuning::get_instance().is_running())
            {
                ESP_LOGW(TAG, "Wheel targets rejected: control calibration is running.");
                break;
            }
            if (!session_active())
                break;
            // The dashboard writes wheel targets directly, bypassing the
            // safety supervisor: apply the same motion inhibition the health
            // monitor classified from the live observations, unless the bench
            // motion override was enabled explicitly.
            if (ProtocolHandler::get_instance().get_health_state().motion_inhibited &&
                !m_motion_override)
            {
                ESP_LOGW(TAG, "Wheel targets rejected: motion inhibited by the safety monitor.");
                WheelController::get_instance().set_target_velocities({0.0f, 0.0f, 0.0f, 0.0f});
                break;
            }
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
            MotorCalibration::get_instance().abort();
            WheelTuning::get_instance().abort();
            WheelController::get_instance().stop_position_moves();
            WheelController::get_instance().set_target_velocities({0.0f, 0.0f, 0.0f, 0.0f});
            ESP_LOGI(TAG, "Wheel targets zeroed by dashboard.");
            break;

        case DBG_CONTROL_ENABLE_MOTION_OVERRIDE:
            if (!session_active())
                break;
            m_motion_override = true;
            ESP_LOGW(TAG,
                     "Bench motion override enabled: wheel targets ignore the health inhibit.");
            break;

        case DBG_CONTROL_DISABLE_MOTION_OVERRIDE:
            clear_motion_override();
            break;

        default:
            ESP_LOGW(TAG, "Unknown debug control op %d.", control.op);
            break;
    }
}

/**
 * @brief Applies one dashboard position-control frame.
 *
 * Position moves follow the same motion gate as wheel targets: a health
 * inhibit blocks them unless the bench motion override is active, and a
 * running calibration blocks them unconditionally.
 */
void SerialDebug::handle_position_frame()
{
    if (m_decoder.message_length() != BYTES_LENGTH_DEBUG_POSITION_CONTROL)
        return;

    // Decoders use bitwise OR: always decode into a zero-initialized struct.
    DebugPositionControl control;
    memset(&control, 0, sizeof(control));
    DecodeDebugPositionControl(&control, const_cast<uint8_t *>(m_decoder.message()));

    if (!session_active())
        return;

    WheelController & wheels = WheelController::get_instance();
    switch (control.op)
    {
        case DBG_POSITION_OP_MOVE:
        {
            if (MotorCalibration::get_instance().is_running() ||
                WheelTuning::get_instance().is_running())
            {
                ESP_LOGW(TAG, "Position move rejected: a calibration is running.");
                break;
            }
            if (ProtocolHandler::get_instance().get_health_state().motion_inhibited &&
                !m_motion_override)
            {
                ESP_LOGW(TAG, "Position move rejected: motion inhibited by the safety monitor.");
                break;
            }
            std::array<float, 4> angles = {0.0f, 0.0f, 0.0f, 0.0f};
            for (size_t i = 0; i < 4; ++i)
                angles[i] =
                  static_cast<float>(control.angle_mdeg[i]) * static_cast<float>(M_PI) / 180000.0f;
            const float max_velocity =
              (control.max_rpm > 0) ? static_cast<float>(control.max_rpm) * RPM_TO_RAD_S : 0.0f;
            if (wheels.start_position_move(control.wheel_mask,
                                           angles,
                                           control.mode == DBG_POSITION_MODE_RELATIVE,
                                           max_velocity) != ESP_OK)
            {
                ESP_LOGW(TAG,
                         "Position move rejected (mask=0x%X).",
                         static_cast<unsigned>(control.wheel_mask));
            }
            break;
        }

        case DBG_POSITION_OP_STOP:
            wheels.stop_position_moves();
            ESP_LOGI(TAG, "Position moves stopped by dashboard.");
            break;

        default:
            ESP_LOGW(TAG, "Unknown position control op %d.", control.op);
            break;
    }
}

/**
 * @brief Streams the control-calibration status with the parameters in effect.
 *
 * The gains/feedforward columns always report WheelController's live values,
 * so the dashboard shows the stored parameters at idle and the candidate under
 * validation while the routine runs.
 */
void SerialDebug::send_tuning()
{
    // Zero-init before encode (bitproto uses bitwise OR).
    DebugTuning tuning;
    memset(&tuning, 0, sizeof(tuning));

    const WheelTuning::Status status = WheelTuning::get_instance().get_status();
    const auto parameters = WheelController::get_instance().get_parameters();

    tuning.state = static_cast<uint8_t>(status.state);
    tuning.wheel = status.wheel;
    tuning.phase = static_cast<uint8_t>(status.phase);
    tuning.progress = status.progress;
    tuning.result_mask = status.result_mask;
    tuning.error_code = static_cast<uint8_t>(status.error);

    for (size_t i = 0; i < 4; ++i)
    {
        tuning.kp[i] = static_cast<int16_t>(std::clamp<long>(
          std::lround(static_cast<double>(parameters[i].kp) * 100.0), -32768L, 32767L));
        tuning.ki[i] = static_cast<int16_t>(std::clamp<long>(
          std::lround(static_cast<double>(parameters[i].ki) * 100.0), -32768L, 32767L));
        tuning.kd[i] = static_cast<int16_t>(std::clamp<long>(
          std::lround(static_cast<double>(parameters[i].kd) * 10000.0), -32768L, 32767L));
        tuning.feedforward[i] = static_cast<int16_t>(std::clamp<long>(
          std::lround(static_cast<double>(parameters[i].feedforward)), -32768L, 32767L));
        tuning.velocity_ff[i] = static_cast<int16_t>(std::clamp<long>(
          std::lround(static_cast<double>(parameters[i].velocity_ff) * 100.0), -32768L, 32767L));
        tuning.rise_ms[i] = status.rise_ms[i];
        tuning.overshoot_pct[i] = status.overshoot_pct[i];
        tuning.ss_error_pct[i] = status.ss_error_pct[i];
    }
    tuning.sequence = ++m_tuning_sequence;

    uint8_t payload[BYTES_LENGTH_DEBUG_TUNING];
    EncodeDebugTuning(&tuning, payload);
    send_frame(DBG_MSG_TUNING, payload, BYTES_LENGTH_DEBUG_TUNING);
}

/**
 * @brief Streams the position-control masks, continuous angles and targets.
 */
void SerialDebug::send_position()
{
    // Zero-init before encode (bitproto uses bitwise OR).
    DebugPosition position;
    memset(&position, 0, sizeof(position));

    WheelController & wheels = WheelController::get_instance();
    position.active_mask = wheels.get_position_active_mask();
    position.done_mask = wheels.get_position_done_mask();
    position.error_code = static_cast<uint8_t>(wheels.get_position_error());

    const auto angles = wheels.get_position_angles_rad();
    const auto targets = wheels.get_position_targets_rad();
    // Radians to thousandths of a degree.
    constexpr double MILLI_DEG_PER_RAD = 180000.0 / 3.14159265358979323846;
    for (size_t i = 0; i < 4; ++i)
    {
        position.angle_mdeg[i] = static_cast<int32_t>(
          std::clamp<long>(std::lround(static_cast<double>(angles[i]) * MILLI_DEG_PER_RAD),
                           -2147483647L,
                           2147483647L));
        position.target_mdeg[i] = static_cast<int32_t>(
          std::clamp<long>(std::lround(static_cast<double>(targets[i]) * MILLI_DEG_PER_RAD),
                           -2147483647L,
                           2147483647L));
    }
    position.sequence = ++m_position_sequence;

    uint8_t payload[BYTES_LENGTH_DEBUG_POSITION];
    EncodeDebugPosition(&position, payload);
    send_frame(DBG_MSG_POSITION, payload, BYTES_LENGTH_DEBUG_POSITION);
}

bool SerialDebug::send_frame(uint8_t msg_type, const void * message, size_t message_len)
{
    if (m_tx_mutex == nullptr || !session_active())
        return false;

    uint8_t frame[serial_debug::MAX_FRAME_BYTES];
    const size_t frame_len = serial_debug::encode_frame(
      msg_type, static_cast<const uint8_t *>(message), message_len, frame, sizeof(frame));
    if (frame_len == 0)
        return false;

    if (xSemaphoreTakeRecursive(m_tx_mutex, pdMS_TO_TICKS(20)) != pdTRUE)
        return false;
    if (!session_active())
    {
        xSemaphoreGiveRecursive(m_tx_mutex);
        return false;
    }
    uart_write_bytes(UART_NUM_0, frame, frame_len);
    xSemaphoreGiveRecursive(m_tx_mutex);
    return true;
}

bool SerialDebug::send_text_line(const char * line, size_t length)
{
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

    return send_frame(DBG_MSG_TEXT, payload, sizeof(payload));
}

int SerialDebug::log_hook(const char * format, va_list args)
{
    SerialDebug & self = get_instance();
    if (!self.session_active())
        return 0;

    // Format only when a dashboard is receiving diagnostics.
    char raw[LOG_LINE_CAP];
    va_list args_copy;
    va_copy(args_copy, args);
    int written = vsnprintf(raw, sizeof(raw), format, args_copy);
    va_end(args_copy);
    if (written < 0)
        written = 0;
    if (written >= static_cast<int>(sizeof(raw)))
        written = static_cast<int>(sizeof(raw)) - 1;

    // Append to the pending line, stripping ANSI colors, and emit a frame
    // for every completed line.
    if (xSemaphoreTakeRecursive(self.m_tx_mutex, pdMS_TO_TICKS(20)) != pdTRUE)
        return written;
    if (!self.session_active())
    {
        xSemaphoreGiveRecursive(self.m_tx_mutex);
        return 0;
    }
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
    const auto target = wheel_ctrl.get_target_velocities();
    auto & estimator = WheelStateEstimator::get_instance();
    // The estimator is the source of truth: the control loop may be paused
    // (open-loop calibration), so its cached velocity would be stale.
    const auto encoder_velocity = estimator.get_smoothed_velocity_rad_s();
    const auto angle = estimator.get_measured_angle_rad();

    for (size_t i = 0; i < 4; ++i)
    {
        state.wheels[i].pwm = static_cast<int16_t>(std::lround(duty[i]));
        // Negate into the motor frame, matching the wheel targets.
        state.wheels[i].velocity =
          static_cast<int16_t>(std::lround(-encoder_velocity[i] * RAD_S_TO_RPM * 10.0f));
        state.wheels[i].target =
          static_cast<int16_t>(std::lround(target[i] * RAD_S_TO_RPM * 10.0f));
        // Wrap the encoder-shaft angle into one turn and scale to hundredths
        // of a degree, the DebugWheelAngle wire unit.
        double angle_deg = std::fmod(static_cast<double>(angle[i]) * (180.0 / M_PI), 360.0);
        if (angle_deg < 0.0)
            angle_deg += 360.0;
        state.wheels[i].angle = static_cast<uint16_t>(std::lround(angle_deg * 100.0)) % 36000;
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
        tof.distance_mm[zone] = frame.distance_mm[index];
        tof.target_status[zone] = frame.target_status[index];
    }
    uint8_t payload[BYTES_LENGTH_DEBUG_TOF];
    EncodeDebugTof(&tof, payload);
    send_frame(DBG_MSG_TOF, payload, BYTES_LENGTH_DEBUG_TOF);
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
    const HealthState safety = protocol.get_health_state();
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

    health.radio_ready = protocol.is_radio_ready() ? 1 : 0;
    health.base_station_connected = protocol.is_base_station_connected() ? 1 : 0;

    // The WheelStateEstimator continuous stream keeps the power monitor
    // updated; the health frame only publishes its latest sample.
    const PowerMonitor & power = PowerMonitor::get_instance();
    health.supply_voltage =
      static_cast<uint16_t>(std::lroundf(power.get_supply_voltage_v() * 100.0f));
    uint8_t supply_flags = 0;
    if (power.is_valid())
        supply_flags |= 1;
    if (power.is_undervoltage())
        supply_flags |= 2;
    if (power.is_overvoltage())
        supply_flags |= 4;
    health.supply_flags = supply_flags;

    health.free_heap = esp_get_free_heap_size();
    health.min_free_heap = esp_get_minimum_free_heap_size();
    health.sequence = ++m_health_sequence;

    uint8_t payload[BYTES_LENGTH_DEBUG_HEALTH];
    EncodeDebugHealth(&health, payload);
    send_frame(DBG_MSG_HEALTH, payload, BYTES_LENGTH_DEBUG_HEALTH);
}

void SerialDebug::send_calibration()
{
    // Zero-init before encode (bitproto uses bitwise OR).
    DebugCalibration calibration;
    memset(&calibration, 0, sizeof(calibration));

    const MotorCalibration::Status status = MotorCalibration::get_instance().get_status();
    auto & estimator = WheelStateEstimator::get_instance();

    calibration.state = static_cast<uint8_t>(status.state);
    calibration.wheel = status.wheel;
    calibration.progress = status.progress;
    calibration.result_mask = status.result_mask;
    calibration.i2c_ready = status.i2c_ready;
    calibration.mapping_valid = estimator.has_calibrated_mapping() ? 1 : 0;

    const auto analog_map = estimator.get_analog_mapping();
    const auto mux_map = estimator.get_mux_mapping();
    uint8_t range_mask = 0;
    for (size_t i = 0; i < 4; ++i)
    {
        calibration.analog_map[i] = analog_map[i];
        calibration.mux_map[i] = mux_map[i];
        calibration.analog_level[i] = status.analog_level[i];
        calibration.i2c_level[i] = status.i2c_level[i];
        if (estimator.is_channel_calibrated(i))
            range_mask |= static_cast<uint8_t>(1U << i);
    }
    calibration.range_mask = range_mask;
    calibration.error_code = static_cast<uint8_t>(status.error);
    calibration.sequence = ++m_calibration_sequence;

    uint8_t payload[BYTES_LENGTH_DEBUG_CALIBRATION];
    EncodeDebugCalibration(&calibration, payload);
    send_frame(DBG_MSG_CALIBRATION, payload, BYTES_LENGTH_DEBUG_CALIBRATION);
}

#endif  // CONFIG_SERIAL_DEBUG
