#include <esp_err.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdint>
#include <cstdio>

#include "../../shared_test_bus.h"
#include "I2Cdev.h"
#include "IMUGY85.h"

extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(200));

    const i2c_master_bus_handle_t i2c_bus = shared_test_i2c_bus();
    I2Cdev::init(i2c_bus);

    IMUGY85 & imu = IMUGY85::get_instance();
    if (imu.init(i2c_bus) != ESP_OK)
    {
        printf("ERR,imu_init_failed\n");
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Aguarda o PLL do ITG-3200 estabilizar (~50-70 ms)
    vTaskDelay(pdMS_TO_TICKS(100));

    printf("HEADER,timestamp_us,temp_raw,gx_raw,gy_raw,gz_raw\n");

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);  // 100 Hz (1 tick @ 100 Hz)

    while (true)
    {
        vTaskDelayUntil(&last_wake, period);

        int16_t gx = 0, gy = 0, gz = 0, temp = 0;

        imu.read_gyro(&gx, &gy, &gz, &temp);

        const int64_t ts_us = esp_timer_get_time();
        printf("GYRO,%lld,%d,%d,%d,%d\n", static_cast<long long>(ts_us), temp, gx, gy, gz);
    }
}
