#ifndef _KICKER_CONTROLLER_H_
#define _KICKER_CONTROLLER_H_

#include <driver/i2c_master.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <stdint.h>

class KickerController
{
   public:
    static KickerController & get_instance();

    esp_err_t init(i2c_master_bus_handle_t i2c_bus);
    esp_err_t start();

    // Queue a kick command for the kicker task (non-blocking).
    void request_kick(uint8_t kick_velocity, uint32_t sequence);

    // Queue a safe-stop transition to disable or re-enable capacitor charging.
    void set_safe_stop(bool active);

    // Cached capacitor voltage in volts (0 when communication is unavailable).
    uint8_t get_voltage() const;

    // True when the kicker board is reachable, charged, idle, and fault-free.
    bool is_ready() const;

    // True when the kicker board is unreachable or reports a hardware fault.
    bool has_fault() const;

   private:
    enum class CommandType : uint8_t
    {
        KICK = 0,
        SAFE_STOP = 1,
    };

    struct KickerCommand
    {
        CommandType type = CommandType::KICK;
        uint8_t kick_velocity = 0;
        uint32_t sequence = 0;
        bool safe_stop_active = false;
    };

    KickerController() = default;
    ~KickerController() = default;
    KickerController(const KickerController &) = delete;
    KickerController & operator=(const KickerController &) = delete;

    static void task_wrapper(void * arg);
    void task_loop();
    void handle_command(const KickerCommand & cmd);
    void poll_board();

    static constexpr uint32_t TASK_STACK_BYTES = 4096;
    static constexpr UBaseType_t TASK_PRIORITY = 4;
    static constexpr UBaseType_t COMMAND_QUEUE_DEPTH = 4;
    static constexpr uint32_t POLL_PERIOD_MS = 50;
    static constexpr uint32_t MUTEX_WAIT_MS = 10;

    bool m_initialized = false;
    TaskHandle_t m_task_handle = nullptr;
    SemaphoreHandle_t m_mutex = nullptr;
    QueueHandle_t m_cmd_queue = nullptr;

    // Cached board state protected by m_mutex.
    bool m_comm_ok = false;
    uint8_t m_status = 0;
    uint8_t m_voltage = 0;
    bool m_safe_stop_queued = false;

    // Task-private state accessed only from task_loop().
    bool m_in_safe_stop = false;
    uint32_t m_last_kick_seq = 0;
};

#endif  // _KICKER_CONTROLLER_H_
