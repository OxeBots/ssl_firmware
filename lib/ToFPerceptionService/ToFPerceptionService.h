#ifndef _TOF_PERCEPTION_SERVICE_H_
#define _TOF_PERCEPTION_SERVICE_H_

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <stdint.h>

#include <array>
#include <atomic>

#include "RuntimeTypes.h"
#include "VL53L5CX.h"

struct ToFPerceptionFrame
{
    static constexpr size_t MAX_ZONES = VL53L5CX_RESOLUTION_8X8;
    static constexpr size_t MAX_TARGETS = VL53L5CX_RESOLUTION_8X8 * VL53L5CX_NB_TARGET_PER_ZONE;

    uint8_t resolution = VL53L5CX_RESOLUTION_8X8;
    uint32_t sequence = 0;
    int64_t timestamp_us = 0;
    bool valid = false;
    std::array<int16_t, MAX_TARGETS> distance_mm = {};
    std::array<uint8_t, MAX_TARGETS> target_status = {};

    bool has_valid_target(size_t index) const;
};

class ToFPerceptionService
{
   public:
    ToFPerceptionService() = default;
    ~ToFPerceptionService();

    esp_err_t init(i2c_master_bus_handle_t i2c_bus,
                   gpio_num_t interrupt_gpio,
                   uint8_t frequency_hz = 15);
    esp_err_t start();
    void stop();

    bool get_latest_frame(ToFPerceptionFrame * frame) const;
    bool get_latest_state(LocalPerceptionState * state,
                          int16_t obstacle_stop_distance_mm = 150) const;
    static LocalPerceptionState project_frame(const ToFPerceptionFrame & frame,
                                              int16_t obstacle_stop_distance_mm = 150);
    bool is_available() const;

   private:
    static void task_wrapper(void * arg);
    void task_loop();
    void publish_result(const VL53L5CX_ResultsData & results);

    VL53L5CX m_sensor;
    gpio_num_t m_interrupt_gpio = GPIO_NUM_NC;
    TaskHandle_t m_task_handle = nullptr;
    SemaphoreHandle_t m_mutex = nullptr;
    ToFPerceptionFrame m_frame;
    uint8_t m_frequency_hz = 15;
    bool m_initialized = false;
    std::atomic_bool m_available = false;
    // Timestamp of the last published valid frame (us); availability means
    // frames are still arriving, not merely that init once passed.
    std::atomic<uint64_t> m_last_frame_us = 0;
};

#endif  // _TOF_PERCEPTION_SERVICE_H_
