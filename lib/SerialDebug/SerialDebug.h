#ifndef _SERIAL_DEBUG_H_
#define _SERIAL_DEBUG_H_

#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <atomic>
#include <cstdint>

#include "SerialDebugCodec.h"
#include "ToFPerceptionService.h"

// Streams binary debug frames over UART0 for the robot_bench GUI. Compiled in
// only when CONFIG_SERIAL_DEBUG is set. The service stays silent until the
// dashboard sends a START_STREAM control frame and keeps streaming only while
// keepalives arrive, so a closed serial port means no debug traffic at all.
class SerialDebug
{
   public:
    static SerialDebug & get_instance();

    // Install the UART driver and the log capture hook.
    esp_err_t init(ToFPerceptionService * tof_service);

    // Create the streaming task.
    void start_task();

   private:
    SerialDebug() = default;
    SerialDebug(const SerialDebug &) = delete;
    SerialDebug & operator=(const SerialDebug &) = delete;

    static void task_wrapper(void * arg);
    void task_loop();

    void handle_control_frame();
    void handle_position_frame();
    // Drop the bench override and stop the wheels it was driving.
    void clear_motion_override();
    void send_state(uint32_t now_ms);
    void send_imu();
    void send_tof();
    void send_health();
    void send_calibration();
    void send_tuning();
    void send_position();

    // Serialize, frame and transmit one message; takes the TX mutex.
    bool send_frame(uint8_t msg_type, const void * message, size_t message_len);

    // Queue one ASCII log line as a DebugText frame; takes the TX mutex.
    bool send_text_line(const char * line, size_t length);

    // Log capture is active only while the dashboard keeps its session alive.
    static int log_hook(const char * format, va_list args);
    bool session_active() const;

    // Max chars per DebugText frame (protocol limit)
    static constexpr size_t TEXT_MAX = 96;
    // Line assembly buffer: TEXT_MAX plus ANSI/level prefix headroom
    static constexpr size_t LOG_LINE_CAP = 128;
    static constexpr uint32_t STATE_PERIOD_MS = 40;         // 25 Hz
    static constexpr uint32_t IMU_PERIOD_MS = 100;          // 10 Hz
    static constexpr uint32_t HEALTH_PERIOD_MS = 500;       // 2 Hz
    static constexpr uint32_t CALIBRATION_PERIOD_MS = 200;  // 5 Hz
    static constexpr uint32_t TUNING_PERIOD_MS = 200;       // 5 Hz
    static constexpr uint32_t POSITION_PERIOD_MS = 100;     // 10 Hz
    static constexpr uint64_t KEEPALIVE_TIMEOUT_US = 1500000;
    // Received-byte report cadence while connected.
    static constexpr uint64_t RX_REPORT_INTERVAL_US = 10000000;

    TaskHandle_t m_task_handle = nullptr;
    SemaphoreHandle_t m_tx_mutex = nullptr;
    serial_debug::Decoder m_decoder;

    ToFPerceptionService * m_tof_service = nullptr;
    std::atomic_bool m_streaming{false};
    std::atomic<uint64_t> m_last_keepalive_us{0};
    // Bench override: accept dashboard wheel targets while the health monitor
    // inhibits motion. Cleared when the streaming session ends.
    bool m_motion_override = false;

    // Stream period bookkeeping
    uint32_t m_last_state_ms = 0;
    uint32_t m_last_imu_ms = 0;
    uint32_t m_last_health_ms = 0;
    uint32_t m_last_calibration_ms = 0;
    uint32_t m_last_tuning_ms = 0;
    uint32_t m_last_position_ms = 0;
    uint32_t m_state_sequence = 0;
    uint32_t m_imu_sequence = 0;
    uint32_t m_health_sequence = 0;
    uint32_t m_calibration_sequence = 0;
    uint32_t m_tuning_sequence = 0;
    uint32_t m_position_sequence = 0;
    uint32_t m_last_tof_sequence = 0;
    bool m_tof_frame_sent = false;

    // Log capture line assembly (any task may log; guarded by m_tx_mutex)
    char m_log_line[LOG_LINE_CAP] = {};
    size_t m_log_line_len = 0;
    size_t m_log_escape_left = 0;
};

#endif  // _SERIAL_DEBUG_H_
