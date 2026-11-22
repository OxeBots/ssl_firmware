/**
 * @file kicker_controller.cpp
 * @brief Runtime service managing the kicker board over the shared I2C bus.
 */

#include "kicker_controller.h"

#include <esp_log.h>

#include "i2c_kicker.h"

static const char * TAG = "KickerController";

static constexpr uint8_t DEFAULT_SETPOINT_V = 150;
static constexpr uint8_t DEFAULT_KICK_DURATION = 50;

/**
 * @brief Convert radio kick velocity (1 to 255) into board kick duty (1 to KICKER_DUTY_MAX).
 */
static uint8_t velocity_to_duty(uint8_t kick_velocity)
{
    const uint16_t scaled =
      (static_cast<uint16_t>(kick_velocity) * KICKER_DUTY_MAX + 127U) / 255U;
    return scaled == 0U ? 1U : static_cast<uint8_t>(scaled);
}

/**
 * @brief Return the single KickerController instance.
 */
KickerController & KickerController::get_instance()
{
    static KickerController instance;
    return instance;
}

/**
 * @brief Bind the kicker driver to the I2C bus, verify the board, and allocate RTOS objects.
 *
 * @param i2c_bus Shared I2C master bus handle cre
 ated before task startup.
 * @return ESP_OK on success, or an error code when the board is absent or setup fails.
 */
esp_err_t KickerController::init(i2c_master_bus_handle_t i2c_bus)
{
    if (m_initialized)
        return ESP_ERR_INVALID_STATE;
    if (i2c_bus == nullptr)
        return ESP_ERR_INVALID_ARG;

    if (!kicker_init(i2c_bus))
        return ESP_FAIL;

    uint8_t version = 0;
    esp_err_t err = kicker_get_version(&version);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Kicker did not respond on I2C: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Kicker v%u detected.", version);

    err = kicker_set_setpoint_voltage(DEFAULT_SETPOINT_V);
    if (err != ESP_OK)
        return err;

    err = kicker_set_kick_duration(DEFAULT_KICK_DURATION);
    if (err != ESP_OK)
        return err;

    err = kicker_enable_charge(true);
    if (err != ESP_OK)
        return err;

    m_mutex = xSemaphoreCreateMutex();
    if (m_mutex == nullptr)
        return ESP_ERR_NO_MEM;

    m_cmd_queue = xQueueCreate(COMMAND_QUEUE_DEPTH, sizeof(KickerCommand));
    if (m_cmd_queue == nullptr)
    {
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
        return ESP_ERR_NO_MEM;
    }

    poll_board();
    m_initialized = true;
    return ESP_OK;
}

/**
 * @brief Launch the kicker FreeRTOS task.
 *
 * Must be called after init().
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized or already running,
 *         or ESP_ERR_NO_MEM if task creation fails.
 */
esp_err_t KickerController::start()
{
    if (!m_initialized || m_task_handle != nullptr)
        return ESP_ERR_INVALID_STATE;

    BaseType_t result = xTaskCreate(
      task_wrapper, "kicker_task", TASK_STACK_BYTES, this, TASK_PRIORITY, &m_task_handle);
    if (result != pdPASS)
        return ESP_ERR_NO_MEM;

    return ESP_OK;
}

/**
 * @brief Enqueue a kick request without blocking the caller.
 *
 * @param kick_velocity Requested kick strength from the radio command (1 to 255).
 * @param sequence Monotonic command sequence used to avoid firing twice for one command.
 */
void KickerController::request_kick(uint8_t kick_velocity, uint32_t sequence)
{
    if (m_cmd_queue == nullptr || kick_velocity == 0U)
        return;

    KickerCommand cmd = {};
    cmd.type = CommandType::KICK;
    cmd.kick_velocity = kick_velocity;
    cmd.sequence = sequence;
    xQueueSend(m_cmd_queue, &cmd, 0);
}

/**
 * @brief Enqueue a safe-stop state change to disable or re-enable charging.
 *
 * Only sends a queue item when the requested state changes.
 *
 * @param active True to enter safe stop and disable charging; false to re-enable charging.
 */
void KickerController::set_safe_stop(bool active)
{
    if (m_cmd_queue == nullptr || m_mutex == nullptr)
        return;

    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) != pdTRUE)
        return;

    const bool changed = (active != m_safe_stop_queued);
    if (changed)
    {
        KickerCommand cmd = {};
        cmd.type = CommandType::SAFE_STOP;
        cmd.safe_stop_active = active;
        if (xQueueSend(m_cmd_queue, &cmd, 0) == pdPASS)
            m_safe_stop_queued = active;
    }

    xSemaphoreGive(m_mutex);
}

/**
 * @brief Return the latest cached capacitor voltage in volts.
 *
 * @return Capacitor voltage in volts, or 0 when unavailable.
 */
uint8_t KickerController::get_voltage() const
{
    if (m_mutex == nullptr)
        return 0;

    uint8_t voltage = 0;
    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE)
    {
        voltage = m_comm_ok ? m_voltage : 0;
        xSemaphoreGive(m_mutex);
    }
    return voltage;
}

/**
 * @brief Report whether the kicker is ready to fire a new kick.
 *
 * @return True when communication is healthy, charge is complete, and no fault or kick is active.
 */
bool KickerController::is_ready() const
{
    if (m_mutex == nullptr)
        return false;

    bool ready = false;
    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE)
    {
        ready = m_comm_ok && kicker_is_done(m_status) && !kicker_is_kicking(m_status) &&
                !kicker_has_fault(m_status) && !kicker_has_ov_trip(m_status);
        xSemaphoreGive(m_mutex);
    }
    return ready;
}

/**
 * @brief Report whether the kicker has a communication or board fault.
 *
 * @return True when uninitialized, unreachable, or reporting fault or overvoltage.
 */
bool KickerController::has_fault() const
{
    if (m_mutex == nullptr)
        return true;

    bool fault = true;
    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE)
    {
        fault = !m_comm_ok || kicker_has_fault(m_status) || kicker_has_ov_trip(m_status);
        xSemaphoreGive(m_mutex);
    }
    return fault;
}

/**
 * @brief FreeRTOS entry point forwarding execution to task_loop().
 */
void KickerController::task_wrapper(void * arg)
{
    static_cast<KickerController *>(arg)->task_loop();
}

/**
 * @brief Wait for queued kick or safety commands and poll board status periodically.
 */
void KickerController::task_loop()
{
    while (true)
    {
        KickerCommand cmd = {};
        if (xQueueReceive(m_cmd_queue, &cmd, pdMS_TO_TICKS(POLL_PERIOD_MS)) == pdTRUE)
            handle_command(cmd);

        poll_board();
    }
}

/**
 * @brief Execute a queued safe-stop or kick command over I2C.
 */
void KickerController::handle_command(const KickerCommand & cmd)
{
    if (cmd.type == CommandType::SAFE_STOP)
    {
        m_in_safe_stop = cmd.safe_stop_active;
        esp_err_t err = kicker_enable_charge(!m_in_safe_stop);
        if (err != ESP_OK)
            ESP_LOGW(TAG, "Failed to update charge enable: %s", esp_err_to_name(err));
        return;
    }

    if (m_in_safe_stop || cmd.kick_velocity == 0U || cmd.sequence == m_last_kick_seq)
        return;

    uint8_t status = 0;
    if (kicker_get_status(&status) != ESP_OK)
        return;

    const bool can_kick = kicker_is_done(status) && !kicker_is_kicking(status) &&
                          !kicker_has_fault(status) && !kicker_has_ov_trip(status);
    if (!can_kick)
        return;

    const uint8_t duty = velocity_to_duty(cmd.kick_velocity);
    if (kicker_set_kick_duty(duty) != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to set kick duty (%u).", duty);
        return;
    }

    if (kicker_start_kick() != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to trigger kick.");
        return;
    }

    m_last_kick_seq = cmd.sequence;
}

/**
 * @brief Read board status and capacitor voltage over I2C and publish the cached snapshot.
 */
void KickerController::poll_board()
{
    uint8_t status = 0;
    uint8_t voltage = 0;
    const bool ok =
      (kicker_get_status(&status) == ESP_OK) && (kicker_get_voltage(&voltage) == ESP_OK);

    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE)
    {
        m_comm_ok = ok;
        if (ok)
        {
            m_status = status;
            m_voltage = voltage;
        }
        xSemaphoreGive(m_mutex);
    }
}
