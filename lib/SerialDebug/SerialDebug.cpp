/**
 * @file SerialDebug.cpp
 * @brief Serial debug dashboard service: aggregates firmware state and
 *        streams it as binary frames over UART0 while a dashboard client
 *        keeps the connection alive.
 *
 * Subscribed stream contents (115200 8N1):
 *   - DebugState  at ~83 Hz: four wheel PWM/velocity/target/angle and the
 *     digital BL48250 pulse speed/count.
 *   - DebugImu    at 10 Hz: attitude and calibrated inertial readings.
 *   - DebugTof    per new frame: one 8x8 VL53L5CX range frame.
 *   - DebugHealth at 2 Hz: safety lifecycle, faults, radio link, supply, heap.
 *   - DebugCalibration at 5 Hz: motor/encoder wiring calibration progress.
 *   - DebugTuning at 5 Hz: control calibration status, per-direction
 *     parameters/metrics and the calibrated maximum speed limit.
 *   - DebugRaw    at 10 Hz: raw ADC values, the focused wheel's raw I2C angle,
 *     FG pulse deltas, EKF wheel speed and the applied PWM/direction.
 *   - DebugText   per log line: ESP_LOG output captured by a vprintf hook.
 *
 * DebugSubscribe selects the streams and renews the session. The link
 * heartbeat is sent even when no data stream is requested.
 */

// Includes stay outside the conditional so the PlatformIO library finder
// sees the component dependencies even when the dashboard is disabled.
#include "SerialDebug.h"

#include <driver/uart.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <sdkconfig.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "AS5600.h"
#include "BL48250.h"
#include "CD4052B.h"
#include "OrientationHandler.h"
#include "PowerMonitor.h"
#include "ProtocolHandler.h"
#include "RobotState.h"
#include "WheelController.h"
#include "constants.h"
#include "serial_debug_protocol_bp.h"
#include "wheel_state_estimator.h"

#if CONFIG_SERIAL_DEBUG

#include "MotorCalibration.h"

static const char * TAG = "SerialDebug";

// The calibration phase enum is streamed as DBG_TUNING_PHASE_*: keep the two in
// sync, including the maximum-speed phase added for the calibrated limit.
static_assert(static_cast<uint8_t>(MotorCalibration::ControllerPhase::MAX_SPEED) ==
                DBG_TUNING_PHASE_MAX_SPEED,
              "ControllerPhase values must match the DBG_TUNING_PHASE_* constants.");

SerialDebug & SerialDebug::get_instance()
{
    static SerialDebug instance;
    return instance;
}

esp_err_t SerialDebug::init(ToFPerceptionService * tof_service, CD4052B * cd4052b)
{
    if (m_tx_mutex != nullptr)
        return ESP_ERR_INVALID_STATE;
    m_tof_service = tof_service;
    m_cd4052b = cd4052b;

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

void SerialDebug::stop_bench_targets()
{
    if (m_has_bench_targets)
        ProtocolHandler::get_instance().set_bench_targets({});
    m_has_bench_targets = false;
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
        if ((m_streams.load() & DBG_STREAM_TEXT) && session_active() &&
            now_us - last_rx_report_us >= RX_REPORT_INTERVAL_US)
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
            if (m_decoder.message_type() == DBG_MSG_SUBSCRIBE)
                handle_subscription_frame();
            else if (m_decoder.message_type() == DBG_MSG_POSITION_CONTROL)
                handle_position_frame();
            else if (m_decoder.message_type() == DBG_MSG_WHEEL_TUNING)
                handle_wheel_tuning_frame();
            else
                handle_control_frame();
            m_decoder.pop();
        }

        if (m_streaming.load())
        {
            if (!session_active())
            {
                m_streaming.store(false);
                stop_bench_targets();
                m_raw_encoder_wheel = -1;
                MotorCalibration::get_instance().abort();
                WheelController::get_instance().stop_position_moves();
            }
            else
            {
                const uint16_t streams = m_streams.load();
                if (now_ms - m_last_link_ms >= LINK_PERIOD_MS)
                {
                    m_last_link_ms = now_ms;
                    send_link(now_ms);
                    if (streams & DBG_STREAM_TEXT)
                        WheelController::get_instance().log_diagnostics();
                }
                const uint32_t state_elapsed_ms = now_ms - m_last_state_ms;
                if ((streams & DBG_STREAM_STATE) && state_elapsed_ms >= STATE_PERIOD_MS)
                {
                    m_last_state_ms = now_ms - state_elapsed_ms % STATE_PERIOD_MS;
                    send_state(now_ms);
                }
                if ((streams & DBG_STREAM_IMU) && now_ms - m_last_imu_ms >= IMU_PERIOD_MS)
                {
                    m_last_imu_ms = now_ms;
                    send_imu();
                }
                if ((streams & DBG_STREAM_HEALTH) && now_ms - m_last_health_ms >= HEALTH_PERIOD_MS)
                {
                    m_last_health_ms = now_ms;
                    send_health();
                }
                if ((streams & DBG_STREAM_CALIBRATION) &&
                    now_ms - m_last_calibration_ms >= CALIBRATION_PERIOD_MS)
                {
                    m_last_calibration_ms = now_ms;
                    send_calibration();
                }
                if ((streams & DBG_STREAM_TUNING) && now_ms - m_last_tuning_ms >= TUNING_PERIOD_MS)
                {
                    m_last_tuning_ms = now_ms;
                    send_tuning();
                }
                if ((streams & DBG_STREAM_POSITION) &&
                    now_ms - m_last_position_ms >= POSITION_PERIOD_MS)
                {
                    m_last_position_ms = now_ms;
                    send_position();
                }
                if ((streams & DBG_STREAM_RAW) && now_ms - m_last_raw_ms >= RAW_PERIOD_MS)
                {
                    m_last_raw_ms = now_ms;
                    send_raw();
                }
                if (streams & DBG_STREAM_TOF)
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

void SerialDebug::handle_subscription_frame()
{
    if (m_decoder.message_length() != BYTES_LENGTH_DEBUG_SUBSCRIBE)
        return;
    // Decoders use bitwise OR: always decode into a zero-initialized struct.
    DebugSubscribe subscription;
    memset(&subscription, 0, sizeof(subscription));
    DecodeDebugSubscribe(&subscription, const_cast<uint8_t *>(m_decoder.message()));

    const bool new_session = !session_active();
    if (new_session)
    {
        if (xSemaphoreTakeRecursive(m_tx_mutex, 0) != pdTRUE)
            return;
        m_last_link_ms = 0;
        m_last_state_ms = 0;
        m_last_pulse_velocity_ms = 0;
        m_state_sequence = 0;
        m_raw_encoder_wheel = -1;
        m_log_line_len = 0;
        m_log_escape_left = 0;
        xSemaphoreGiveRecursive(m_tx_mutex);
        stop_bench_targets();
    }

    const uint16_t old_streams = m_streams.exchange(subscription.streams);
    if (new_session || ((old_streams ^ subscription.streams) & DBG_STREAM_RAW))
        m_raw_pulse_seeded = false;
    if (new_session || ((old_streams ^ subscription.streams) & DBG_STREAM_TOF))
        m_tof_frame_sent = false;
    if ((subscription.streams & DBG_STREAM_ENCODER_STAGES) &&
        (new_session || !(old_streams & DBG_STREAM_ENCODER_STAGES)))
        m_encoder_stage_scan_pending = true;
    m_last_keepalive_us.store(static_cast<uint64_t>(esp_timer_get_time()));
    m_streaming.store(true);
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

    if (session_active())
        ESP_LOGI(TAG, "Debug control frame received (op=%d).", control.op);

    auto & calibration = MotorCalibration::get_instance();
    switch (control.op)
    {
        case DBG_CONTROL_STOP_STREAM:
            m_streaming.store(false);
            stop_bench_targets();
            m_raw_encoder_wheel = -1;
            calibration.abort();
            WheelController::get_instance().stop_position_moves();
            ESP_LOGI(TAG, "Dashboard disconnected, streaming stopped.");
            break;

        case DBG_CONTROL_START_CALIBRATION:
        case DBG_CONTROL_START_TUNING:
        {
            if (!session_active())
                break;
            const bool align_encoders = control.op == DBG_CONTROL_START_CALIBRATION;
            const char * routine = align_encoders ? "Encoder alignment" : "Controller tuning";
            const esp_err_t result = align_encoders ? calibration.start_encoder_alignment()
                                                    : calibration.start_controller_tuning();
            if (result == ESP_ERR_INVALID_STATE)
                ESP_LOGW(TAG, "%s start ignored: a motor calibration is running.", routine);
            else if (result != ESP_OK)
                ESP_LOGE(TAG, "%s start failed: %s.", routine, esp_err_to_name(result));
            else
            {
                if (align_encoders)
                    m_encoder_stage_scan_pending = true;
                ESP_LOGW(TAG, "%s started: the wheels spin one at a time.", routine);
            }
            break;
        }

        case DBG_CONTROL_STOP_CALIBRATION:
            calibration.abort(MotorCalibration::Routine::ENCODER_ALIGNMENT);
            ESP_LOGW(TAG, "Encoder alignment abort requested.");
            break;

        case DBG_CONTROL_STOP_TUNING:
            calibration.abort(MotorCalibration::Routine::CONTROLLER_TUNING);
            WheelController::get_instance().stop_position_moves();
            ESP_LOGW(TAG, "Control calibration abort requested.");
            break;

        case DBG_CONTROL_SET_TARGETS:
        {
            if (!session_active())
                break;
            std::array<float, 4> targets = {0.0f, 0.0f, 0.0f, 0.0f};
            for (size_t i = 0; i < 4; ++i)
            {
                const float rpm = static_cast<float>(control.wheel_targets[i]) / 10.0f;
                targets[i] = rpm * RPM_TO_RAD_S;
            }
            if (!ProtocolHandler::get_instance().set_bench_targets(targets))
            {
                ESP_LOGW(TAG, "Wheel targets rejected: radio or motor calibration has control.");
                break;
            }
            m_has_bench_targets = targets != std::array<float, 4>{};
            ESP_LOGI(TAG,
                     "Wheel targets: %.1f %.1f %.1f %.1f RPM",
                     static_cast<double>(targets[0] * RAD_S_TO_RPM),
                     static_cast<double>(targets[1] * RAD_S_TO_RPM),
                     static_cast<double>(targets[2] * RAD_S_TO_RPM),
                     static_cast<double>(targets[3] * RAD_S_TO_RPM));
            break;
        }

        case DBG_CONTROL_STOP_MOTORS:
            calibration.abort();
            WheelController::get_instance().stop_position_moves();
            WheelController::get_instance().set_target_velocities({0.0f, 0.0f, 0.0f, 0.0f});
            m_has_bench_targets = false;
            ESP_LOGI(TAG, "Wheel targets zeroed by dashboard.");
            break;

        case DBG_CONTROL_SELECT_RAW_ENCODER:
        {
            if (!session_active())
                break;
            // The selector reuses wheel_targets[0]: 0..3 focuses that wheel's
            // encoder on the I2C mux, anything else releases the focus.
            const int wheel = static_cast<int>(control.wheel_targets[0]);
            if (wheel >= 0 && wheel < 4)
            {
                m_raw_encoder_wheel = static_cast<int8_t>(wheel);
                ESP_LOGI(TAG, "Raw encoder focus: M%d.", wheel + 1);
            }
            else
            {
                m_raw_encoder_wheel = -1;
                ESP_LOGI(TAG, "Raw encoder focus cleared.");
            }
            break;
        }

        default:
            ESP_LOGW(TAG, "Unknown debug control op %d.", control.op);
            break;
    }
}

/**
 * @brief Applies one dashboard position-control frame.
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
            std::array<float, 4> angles = {0.0f, 0.0f, 0.0f, 0.0f};
            for (size_t i = 0; i < 4; ++i)
                angles[i] =
                  static_cast<float>(control.angle_mdeg[i]) * static_cast<float>(M_PI) / 180000.0f;
            const float max_velocity =
              (control.max_rpm > 0) ? static_cast<float>(control.max_rpm) * RPM_TO_RAD_S : 0.0f;
            const bool relative = control.mode == DBG_POSITION_MODE_RELATIVE;
            if (ProtocolHandler::get_instance().start_bench_position_move(
                  control.wheel_mask, angles, relative, max_velocity) != ESP_OK)
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
 * @brief Applies a live tuning edit and replies with the values in effect.
 */
void SerialDebug::handle_wheel_tuning_frame()
{
    if (!session_active() || m_decoder.message_length() != BYTES_LENGTH_DEBUG_WHEEL_TUNING)
        return;

    DebugWheelTuning message;
    memset(&message, 0, sizeof(message));
    DecodeDebugWheelTuning(&message, const_cast<uint8_t *>(m_decoder.message()));
    if (message.status != DBG_WHEEL_TUNING_REQUEST)
        return;

    WheelController::TuningValues values = {};
    static_assert(sizeof(message.values) == sizeof(values));
    esp_err_t result = ESP_OK;
    if (message.wheel >= 4)
        result = ESP_ERR_INVALID_ARG;
    else if (message.fields != 0)
    {
        std::memcpy(values.data(), message.values, sizeof(message.values));
        result = ProtocolHandler::get_instance().apply_bench_wheel_tuning(
          message.wheel, message.fields, values);
    }

    values.fill(0.0f);
    message.mode =
      static_cast<uint8_t>(WheelController::get_instance().get_live_tuning(message.wheel, values));
    std::memcpy(message.values, values.data(), sizeof(message.values));
    message.status = result == ESP_OK                  ? DBG_WHEEL_TUNING_OK
                     : result == ESP_ERR_INVALID_STATE ? DBG_WHEEL_TUNING_BUSY
                                                       : DBG_WHEEL_TUNING_INVALID;
    uint8_t payload[BYTES_LENGTH_DEBUG_WHEEL_TUNING];
    EncodeDebugWheelTuning(&message, payload);
    send_frame(DBG_MSG_WHEEL_TUNING, payload, sizeof(payload));
}

/**
 * @brief Streams the control-calibration status with the parameters in effect.
 *
 * The gains/feedforward columns always report WheelController's live values,
 * so the dashboard shows the stored parameters at idle and the candidate under
 * validation while the routine runs. Each direction has its own feedforward
 * branch and validated step metrics: feedforward/velocity_ff and rise/overshoot/
 * steady-state error describe the forward (positive target) step, the *_rev
 * fields the reverse one. max_rpm carries the calibrated common speed limit
 * currently in effect (also while idle) and wheel_max_rpm the maxima measured
 * by the latest maximum-speed calibration.
 */
void SerialDebug::send_tuning()
{
    // Zero-init before encode (bitproto uses bitwise OR).
    DebugTuning tuning;
    memset(&tuning, 0, sizeof(tuning));

    const auto status = MotorCalibration::get_instance().get_controller_status();
    const auto parameters = WheelController::get_instance().get_parameters();
    // The limit in effect comes from the controller so the dashboard also sees
    // the persisted value at idle, before any run in this session.
    const float max_velocity_rad_s = WheelController::get_instance().get_max_velocity_rad_s();
    tuning.max_rpm = static_cast<uint16_t>(std::clamp<long>(
      std::lround(static_cast<double>(max_velocity_rad_s * RAD_S_TO_RPM * 10.0f)), 0L, 65535L));

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
        tuning.feedforward_rev[i] = static_cast<int16_t>(std::clamp<long>(
          std::lround(static_cast<double>(parameters[i].feedforward_rev)), -32768L, 32767L));
        tuning.velocity_ff_rev[i] = static_cast<int16_t>(
          std::clamp<long>(std::lround(static_cast<double>(parameters[i].velocity_ff_rev) * 100.0),
                           -32768L,
                           32767L));
        tuning.rise_ms[i] = status.rise_ms[i];
        tuning.overshoot_pct[i] = status.overshoot_pct[i];
        tuning.ss_error_pct[i] = status.ss_error_pct[i];
        tuning.rise_ms_rev[i] = status.rise_ms_rev[i];
        tuning.overshoot_pct_rev[i] = status.overshoot_pct_rev[i];
        tuning.ss_error_pct_rev[i] = status.ss_error_pct_rev[i];
        tuning.wheel_max_rpm[i] = status.wheel_max_rpm[i];
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

    if (xSemaphoreTakeRecursive(m_tx_mutex, 0) != pdTRUE)
        return false;
    if (!session_active())
    {
        xSemaphoreGiveRecursive(m_tx_mutex);
        return false;
    }
    size_t tx_free = 0;
    // Leave room for the driver's ring-buffer metadata. A complete frame is
    // either queued immediately or dropped, even when a control task logs.
    constexpr size_t TX_QUEUE_HEADROOM = 64;
    const bool queued =
      uart_get_tx_buffer_free_size(UART_NUM_0, &tx_free) == ESP_OK &&
      tx_free >= frame_len + TX_QUEUE_HEADROOM &&
      uart_write_bytes(UART_NUM_0, frame, frame_len) == static_cast<int>(frame_len);
    xSemaphoreGiveRecursive(m_tx_mutex);
    return queued;
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
    if (!(self.m_streams.load() & DBG_STREAM_TEXT) || !self.session_active())
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
    if (xSemaphoreTakeRecursive(self.m_tx_mutex, 0) != pdTRUE)
        return written;
    if (!(self.m_streams.load() & DBG_STREAM_TEXT) || !self.session_active())
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

void SerialDebug::send_link(uint32_t now_ms)
{
    DebugLink link = {};
    link.uptime_ms = now_ms;
    uint8_t payload[BYTES_LENGTH_DEBUG_LINK];
    EncodeDebugLink(&link, payload);
    send_frame(DBG_MSG_LINK, payload, sizeof(payload));
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
    WheelStateEstimator::ControlSnapshot encoder;
    ++m_state_sequence;
    if (!estimator.get_control_snapshot(&encoder))
        return;
    // In the low-speed pulse/chopper mode the smoothed estimator is biased by
    // the stick-slip bursts. Report the closed-loop window mean that the law
    // actually controls, so the dashboard shows the speed the controller is
    // regulating.
    std::array<float, 4> low_speed_velocity = {};
    const uint8_t low_speed_mask = wheel_ctrl.get_low_speed_velocities_rad_s(low_speed_velocity);
    // Digital feedback: the FG pulse window and the cumulative edge count.
    auto & motors = BL48250::get_instance();
    const auto pulse_counts = motors.pulse_counts();
    if (now_ms - m_last_pulse_velocity_ms >= PULSE_VELOCITY_PERIOD_MS)
    {
        m_last_pulse_velocity_ms = now_ms;
        m_pulse_valid_mask = 0;
        for (size_t i = 0; i < 4; ++i)
        {
            float velocity = 0.0f;
            if (motors.pulse_velocity_rad_s(i, &velocity))
                m_pulse_valid_mask |= static_cast<uint8_t>(1U << i);
            m_pulse_velocity_rpm[i] =
              static_cast<int16_t>(std::clamp(std::lround(velocity * RAD_S_TO_RPM), -2048L, 2047L));
        }
    }

    for (size_t i = 0; i < 4; ++i)
    {
        state.wheels[i].pwm = static_cast<int16_t>(std::lround(duty[i]));
        // Negate into the motor frame, matching the wheel targets.
        const float velocity = (low_speed_mask & (1U << i)) ? low_speed_velocity[i]
                                                            : -encoder.smoothed_velocity_rad_s[i];
        state.wheels[i].velocity =
          static_cast<int16_t>(std::lround(velocity * RAD_S_TO_RPM * 10.0f));
        state.wheels[i].target =
          static_cast<int16_t>(std::lround(target[i] * RAD_S_TO_RPM * 10.0f));
        // The encoder already reports one turn. Round to tenths of a degree.
        state.wheels[i].angle = static_cast<uint16_t>(
          std::lround(encoder.measured_angle_rad[i] * (1800.0f / static_cast<float>(M_PI))) % 3600);
        if (estimator.is_channel_calibrated(i))
            state.encoder_valid_mask |= static_cast<uint8_t>(1u << i);
        state.wheels[i].pulse_velocity = m_pulse_velocity_rpm[i];
        state.wheels[i].pulse_count = static_cast<uint32_t>(pulse_counts[i]);
    }

    state.pulse_valid_mask = m_pulse_valid_mask;
    state.uptime_ms = now_ms;
    state.sequence = m_state_sequence;

    // Packed fields share bytes and the encoder ORs partial writes.
    uint8_t payload[BYTES_LENGTH_DEBUG_STATE] = {};
    EncodeDebugState(&state, payload);
    send_frame(DBG_MSG_STATE, payload, BYTES_LENGTH_DEBUG_STATE);
}

void SerialDebug::send_raw()
{
    // Zero-init before encode (bitproto uses bitwise OR).
    DebugRaw raw;
    memset(&raw, 0, sizeof(raw));

    auto & estimator = WheelStateEstimator::get_instance();
    auto & motors = BL48250::get_instance();

    // The CD4052B mux reaches one encoder at a time: read only the focused
    // wheel's raw angle and report the unreadable sentinel for the others. The
    // encoder-alignment and control calibrations own the mux while they run.
    std::array<uint16_t, NUM_ENC_CHANNELS> i2c_angles = {};
    i2c_angles.fill(0xFFFF);
    const bool mux_free = !MotorCalibration::get_instance().is_running();
    if (mux_free && m_raw_encoder_wheel >= 0)
    {
        const uint8_t wheel = static_cast<uint8_t>(m_raw_encoder_wheel);
        uint16_t angle = 0;
        if (estimator.read_wheel_i2c_raw_angle(wheel, &angle) == ESP_OK)
            i2c_angles[wheel] = angle;
    }

    // Sample the ADC block means immediately after the I2C read: the ADC stream
    // publishes a fresh block roughly every millisecond, so fetching the
    // snapshot right after the mux read keeps both paths close in time and
    // removes most of the phase offset visible while the wheel spins.
    const auto analog_values = estimator.get_raw_values();
    const auto analog_map = estimator.get_analog_mapping();

    const auto pulse_counts = motors.pulse_counts();
    const auto ekf_velocity = estimator.get_filtered_velocity_rad_s();
    const auto duty_cycles = motors.get_duty_cycles();
    const auto directions = motors.get_directions();

    for (size_t i = 0; i < 4; ++i)
    {
        raw.analog_raw[i] = analog_values[analog_map[i]];
        raw.i2c_raw[i] = i2c_angles[i];
        // FG edges since the previous raw frame; the first frame only seeds.
        if (m_raw_pulse_seeded)
        {
            const int32_t delta = pulse_counts[i] - m_last_pulse_counts[i];
            raw.pulse_delta[i] = static_cast<uint16_t>(std::clamp<int32_t>(delta, 0, 65535));
        }
        m_last_pulse_counts[i] = pulse_counts[i];
        // Same motor frame and sign as DebugState.velocity, but before the extra
        // smoothing of the control loop.
        raw.ekf_velocity[i] = static_cast<int16_t>(
          std::clamp<long>(std::lround(-static_cast<double>(ekf_velocity[i]) * RAD_S_TO_RPM * 10.0),
                           -32768L,
                           32767L));
        // The driver stores the duty as a magnitude: apply the direction sign
        // so a positive value means MOTOR_CW.
        const int32_t signed_duty = (directions[i] == config::driver::MOTOR_CW)
                                      ? static_cast<int32_t>(duty_cycles[i])
                                      : -static_cast<int32_t>(duty_cycles[i]);
        raw.applied_pwm[i] = static_cast<int16_t>(std::clamp<int32_t>(signed_duty, -1023, 1023));
    }
    m_raw_pulse_seeded = true;
    raw.sequence = ++m_raw_sequence;

    uint8_t payload[BYTES_LENGTH_DEBUG_RAW];
    EncodeDebugRaw(&raw, payload);
    send_frame(DBG_MSG_RAW, payload, BYTES_LENGTH_DEBUG_RAW);
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

    imu.yaw = static_cast<int16_t>(std::lround(orientation.get_yaw() * 100.0));
    imu.pitch = static_cast<int16_t>(std::lround(orientation.get_pitch() * 100.0));
    imu.roll = static_cast<int16_t>(std::lround(orientation.get_roll() * 100.0));
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

    const auto status = MotorCalibration::get_instance().get_encoder_status();
    auto & estimator = WheelStateEstimator::get_instance();

    calibration.state = static_cast<uint8_t>(status.state);
    calibration.wheel = status.wheel;
    calibration.progress = status.progress;
    calibration.result_mask = status.result_mask;
    calibration.i2c_ready = status.i2c_ready;
    calibration.analog_ready = status.analog_ready;
    bool mapping_valid = false;
    const auto analog_map = estimator.get_analog_mapping();
    const auto mux_map = estimator.get_mux_mapping(&mapping_valid);
    calibration.mapping_valid = mapping_valid ? 1 : 0;
    uint8_t range_mask = 0;
    for (size_t i = 0; i < 4; ++i)
    {
        calibration.analog_map[i] = analog_map[i];
        calibration.mux_map[i] = mux_map[i];
        calibration.detected_mux_map[i] = status.mux_map[i];
        calibration.analog_level[i] = status.analog_level[i];
        calibration.i2c_level[i] = status.i2c_level[i];
        calibration.wheel_error[i] = static_cast<uint8_t>(status.wheel_error[i]);
        calibration.magnet_warning[i] = static_cast<uint8_t>(status.magnet_warning[i]);
        if (estimator.is_channel_calibrated(i))
            range_mask |= static_cast<uint8_t>(1U << i);
    }
    calibration.range_mask = range_mask;
    calibration.error_code = static_cast<uint8_t>(status.error);

    // Ordinary calibration telemetry reports the last verified stages.
    // Probe the hardware when opening the encoder screen or after alignment.
    auto & wheels = WheelController::get_instance();
    if (m_cd4052b != nullptr && m_encoder_stage_scan_pending &&
        (m_streams.load() & DBG_STREAM_ENCODER_STAGES) &&
        !MotorCalibration::get_instance().is_running() && wheels.get_position_active_mask() == 0 &&
        wheels.get_target_velocities() == std::array<float, 4>{})
    {
        m_encoder_stage_scan_pending = false;
        m_encoder_stage_mask = 0;
        for (size_t i = 0; i < 4; ++i)
        {
            if (m_cd4052b->select(mux_map[i]) != ESP_OK)
                continue;
            // Analog switch settling before the I2C transfer.
            esp_rom_delay_us(50);
            AS5600::OutputStage stage = AS5600::OutputStage::ANALOG_FULL;
            if (AS5600::read_output_stage(&stage) != ESP_OK)
                continue;
            m_encoder_stages[i] = static_cast<uint8_t>(stage);
            m_encoder_stage_mask |= static_cast<uint8_t>(1U << i);
        }
        m_cd4052b->disable();
    }
    for (size_t i = 0; i < 4; ++i) calibration.encoder_stage[i] = m_encoder_stages[i];
    calibration.stage_mask = m_encoder_stage_mask;

    // Raw ADC endpoints recorded by the encoder range calibration: min maps to
    // 0 rad and max to 2*pi rad. Zero when the wheel has no valid stored range.
    const auto adc_min_raw = estimator.get_range_raw_min();
    const auto adc_max_raw = estimator.get_range_raw_max();
    for (size_t i = 0; i < 4; ++i)
    {
        calibration.adc_min_raw[i] = adc_min_raw[i];
        calibration.adc_max_raw[i] = adc_max_raw[i];
    }

    calibration.sequence = ++m_calibration_sequence;

    uint8_t payload[BYTES_LENGTH_DEBUG_CALIBRATION];
    EncodeDebugCalibration(&calibration, payload);
    send_frame(DBG_MSG_CALIBRATION, payload, BYTES_LENGTH_DEBUG_CALIBRATION);
}

#endif  // CONFIG_SERIAL_DEBUG
