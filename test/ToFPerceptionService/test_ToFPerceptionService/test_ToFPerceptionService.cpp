#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdkconfig.h>
#include <unity.h>

#include "../shared_test_bus.h"
#include "I2Cdev.h"
#include "SPIdev.h"
#include "ToFPerceptionService.h"

static i2c_master_bus_handle_t s_bus_handle = nullptr;
static ToFPerceptionService s_service;
static bool s_service_initialized = false;

esp_err_t setup_service()
{
    s_bus_handle = shared_test_i2c_bus();
    I2Cdev::init(s_bus_handle);
    return ESP_OK;
}

void test_invalid_lifecycle_calls()
{
    static ToFPerceptionService uninitialized;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      uninitialized.init(shared_test_i2c_bus(), GPIO_NUM_NC, 0));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      uninitialized.init(shared_test_i2c_bus(), GPIO_NUM_NC, 16));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, uninitialized.start());
}

void test_service_publishes_range_frame()
{
    TEST_ASSERT_TRUE(s_service_initialized);
    TEST_ASSERT_EQUAL(ESP_OK, s_service.start());

    ToFPerceptionFrame frame;
    bool received = false;
    for (int attempt = 0; attempt < 30 && !received; ++attempt)
    {
        vTaskDelay(pdMS_TO_TICKS(50));
        received = s_service.get_latest_frame(&frame);
    }

    TEST_ASSERT_TRUE_MESSAGE(received, "Timed out waiting for a ToF frame");
    TEST_ASSERT_EQUAL_UINT8(VL53L5CX_RESOLUTION_8X8, frame.resolution);
    TEST_ASSERT_TRUE(frame.sequence > 0);
    TEST_ASSERT_TRUE(frame.timestamp_us > 0);
    TEST_ASSERT_TRUE(s_service.is_available());
    s_service.stop();
}

void test_short_range_profile_is_applied_to_sensor()
{
    TEST_ASSERT_TRUE(s_service_initialized);
    static VL53L5CX_Configuration device = {};
    device.platform.handle = I2Cdev::get_device_handle(VL53L5CX::DEFAULT_ADDRESS >> 1, 1000000);
    TEST_ASSERT_NOT_NULL(device.platform.handle);

    uint8_t value = 0;
    TEST_ASSERT_EQUAL(VL53L5CX_STATUS_OK, vl53l5cx_get_resolution(&device, &value));
    TEST_ASSERT_EQUAL_UINT8(VL53L5CX_RESOLUTION_8X8, value);
    TEST_ASSERT_EQUAL(VL53L5CX_STATUS_OK, vl53l5cx_get_ranging_frequency_hz(&device, &value));
    TEST_ASSERT_EQUAL_UINT8(15, value);
    TEST_ASSERT_EQUAL(VL53L5CX_STATUS_OK, vl53l5cx_get_ranging_mode(&device, &value));
    TEST_ASSERT_EQUAL_UINT8(VL53L5CX_RANGING_MODE_CONTINUOUS, value);
    TEST_ASSERT_EQUAL(VL53L5CX_STATUS_OK, vl53l5cx_get_target_order(&device, &value));
    TEST_ASSERT_EQUAL_UINT8(VL53L5CX_TARGET_ORDER_CLOSEST, value);
    TEST_ASSERT_EQUAL(VL53L5CX_STATUS_OK, vl53l5cx_get_sharpener_percent(&device, &value));
    TEST_ASSERT_EQUAL_UINT8(0, value);
}

void test_service_initialization()
{
    TEST_ASSERT_EQUAL(ESP_OK, setup_service());
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK,
                              s_service.init(s_bus_handle, (gpio_num_t)CONFIG_TOF_INT_GPIO, 15),
                              "VL53L5CX was not detected at I2C address 0x29");
    s_service_initialized = true;
}

extern "C" void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_invalid_lifecycle_calls);
    RUN_TEST(test_service_initialization);
    RUN_TEST(test_short_range_profile_is_applied_to_sensor);
    RUN_TEST(test_service_publishes_range_frame);
    UNITY_END();
}
