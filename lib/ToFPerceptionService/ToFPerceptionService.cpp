/**
 * @file ToFPerceptionService.cpp
 * @brief Runtime acquisition of VL53L5CX range frames.
 */

#include "ToFPerceptionService.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <string.h>

static const char * TAG = "ToFPerceptionService";

ToFPerceptionService::~ToFPerceptionService()
{
    stop();
    if (m_mutex != nullptr)
        vSemaphoreDelete(m_mutex);
}

esp_err_t ToFPerceptionService::init(i2c_master_bus_handle_t bus_handle,
                                     gpio_num_t interrupt_gpio,
                                     uint8_t frequency_hz)
{
    if (m_initialized)
        return ESP_ERR_INVALID_STATE;
    if (bus_handle == nullptr || frequency_hz == 0 || frequency_hz > 60)
        return ESP_ERR_INVALID_ARG;

    esp_err_t err = m_sensor.init(bus_handle);
    if (err != ESP_OK)
        return err;
    if (!m_sensor.test_connection())
        return ESP_ERR_NOT_FOUND;

    if (m_sensor.set_resolution(VL53L5CX_RESOLUTION_4X4) != VL53L5CX_STATUS_OK ||
        m_sensor.set_ranging_frequency(frequency_hz) != VL53L5CX_STATUS_OK)
        return ESP_ERR_INVALID_RESPONSE;

    bool configure_interrupt = (interrupt_gpio >= 0 && interrupt_gpio < GPIO_NUM_MAX);
#if defined(CONFIG_ESP_CONSOLE_UART_NUM) && CONFIG_ESP_CONSOLE_UART_NUM == 0
    if (interrupt_gpio == GPIO_NUM_1)
    {
        // GPIO1 is UART0 TX on the ESP32. Reconfiguring it would disconnect the
        // serial console used for firmware diagnostics and test results.
        ESP_LOGW(TAG, "GPIO1 is UART0 TX; using polling instead of the ToF interrupt");
        configure_interrupt = false;
    }
#endif

    if (configure_interrupt)
    {
        gpio_config_t config = {};
        config.intr_type = GPIO_INTR_DISABLE;
        config.pin_bit_mask = 1ULL << interrupt_gpio;
        config.mode = GPIO_MODE_INPUT;
        config.pull_up_en = GPIO_PULLUP_DISABLE;
        config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        err = gpio_config(&config);
        if (err != ESP_OK)
            return err;
        m_interrupt_gpio = interrupt_gpio;
    }

    m_mutex = xSemaphoreCreateMutex();
    if (m_mutex == nullptr)
        return ESP_ERR_NO_MEM;

    m_frequency_hz = frequency_hz;
    m_initialized = true;
    m_available = true;
    return ESP_OK;
}

esp_err_t ToFPerceptionService::start()
{
    if (!m_initialized)
        return ESP_ERR_INVALID_STATE;
    if (m_task_handle != nullptr)
        return ESP_ERR_INVALID_STATE;

    if (m_sensor.start_ranging() != VL53L5CX_STATUS_OK)
    {
        m_available = false;
        return ESP_FAIL;
    }

    BaseType_t result = xTaskCreate(task_wrapper, "tof_perception", 6144, this, 4, &m_task_handle);
    if (result != pdPASS)
    {
        m_sensor.stop_ranging();
        m_available = false;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void ToFPerceptionService::stop()
{
    if (m_task_handle != nullptr)
    {
        vTaskDelete(m_task_handle);
        m_task_handle = nullptr;
    }
    if (m_initialized && m_sensor.stop_ranging() == VL53L5CX_STATUS_OK)
        m_available = true;
}

bool ToFPerceptionService::get_latest_frame(ToFPerceptionFrame * frame) const
{
    if (frame == nullptr || m_mutex == nullptr)
        return false;
    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(10)) != pdTRUE)
        return false;
    *frame = m_frame;
    xSemaphoreGive(m_mutex);
    return frame->valid;
}

bool ToFPerceptionService::get_latest_state(LocalPerceptionState * state,
                                            int16_t obstacle_stop_distance_mm) const
{
    if (state == nullptr)
        return false;

    ToFPerceptionFrame frame = {};
    if (!get_latest_frame(&frame))
        return false;

    *state = project_frame(frame, obstacle_stop_distance_mm);
    return state->valid;
}

/**
 * @brief Project a validated range frame into the shared local perception state.
 *
 * This keeps sensor-specific storage out of the state and exposes only the
 * nearest valid target and the conservative obstacle stop condition.
 */
LocalPerceptionState ToFPerceptionService::project_frame(const ToFPerceptionFrame & frame,
                                                         int16_t obstacle_stop_distance_mm)
{
    LocalPerceptionState state = {};
    state.resolution = frame.resolution;
    state.timestamp_us = frame.timestamp_us;
    state.valid = frame.valid;

    int16_t nearest_distance_mm = -1;
    for (size_t index = 0; index < ToFPerceptionFrame::MAX_TARGETS; ++index)
    {
        state.range_mm[index] = frame.distance_mm[index];
        state.target_status[index] = frame.target_status[index];

        const int16_t distance_mm = frame.distance_mm[index];
        if (frame.target_status[index] != 0 && distance_mm > 0 &&
            (nearest_distance_mm < 0 || distance_mm < nearest_distance_mm))
        {
            nearest_distance_mm = distance_mm;
            state.nearest_zone = static_cast<uint8_t>(index);
        }
    }

    state.nearest_distance_mm = nearest_distance_mm;
    state.obstacle_close =
      nearest_distance_mm >= 0 && nearest_distance_mm <= obstacle_stop_distance_mm;
    return state;
}

bool ToFPerceptionService::is_available() const
{
    return m_available;
}

void ToFPerceptionService::task_wrapper(void * arg)
{
    static_cast<ToFPerceptionService *>(arg)->task_loop();
}

void ToFPerceptionService::task_loop()
{
    const TickType_t period = pdMS_TO_TICKS(1000 / m_frequency_hz);
    TickType_t last_wake = xTaskGetTickCount();
    VL53L5CX_ResultsData results = {};

    while (true)
    {
        vTaskDelayUntil(&last_wake, period);

        uint8_t ready = 0;
        uint8_t status = m_sensor.check_data_ready(&ready);
        if (status != VL53L5CX_STATUS_OK)
        {
            m_available = false;
            continue;
        }
        if (ready == 0)
            continue;

        status = m_sensor.get_ranging_data(&results);
        if (status != VL53L5CX_STATUS_OK)
        {
            m_available = false;
            continue;
        }
        publish_result(results);
    }
}

void ToFPerceptionService::publish_result(const VL53L5CX_ResultsData & results)
{
    if (m_mutex == nullptr || xSemaphoreTake(m_mutex, pdMS_TO_TICKS(10)) != pdTRUE)
        return;

    constexpr size_t target_count = ToFPerceptionFrame::MAX_TARGETS;
    memcpy(m_frame.distance_mm.data(),
           results.distance_mm,
           target_count * sizeof(results.distance_mm[0]));
    memcpy(m_frame.target_status.data(),
           results.target_status,
           target_count * sizeof(results.target_status[0]));
    m_frame.resolution = VL53L5CX_RESOLUTION_4X4;
    ++m_frame.sequence;
    m_frame.timestamp_us = esp_timer_get_time();
    m_frame.valid = true;
    m_available = true;
    xSemaphoreGive(m_mutex);
}
