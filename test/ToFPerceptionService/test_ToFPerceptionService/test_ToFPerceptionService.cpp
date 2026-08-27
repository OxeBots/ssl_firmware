#include <ToFPerceptionService.h>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdkconfig.h>
#include <unity.h>

static i2c_master_bus_handle_t s_bus_handle = nullptr;
static ToFPerceptionService s_service;
static bool s_service_initialized = false;

esp_err_t setup_service()
{
    i2c_master_bus_config_t bus_config = {
      .i2c_port = (i2c_port_t)CONFIG_I2C_PORT_NUM,
      .sda_io_num = (gpio_num_t)CONFIG_SDA_GPIO,
      .scl_io_num = (gpio_num_t)CONFIG_SCL_GPIO,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .intr_priority = 0,
      .trans_queue_depth = 0,
      .flags = {.enable_internal_pullup = true, .allow_pd = false}};
    return i2c_new_master_bus(&bus_config, &s_bus_handle);
}

void test_invalid_lifecycle_calls()
{
    ToFPerceptionService uninitialized;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, uninitialized.init(nullptr, GPIO_NUM_NC));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, uninitialized.start());
}

void test_service_publishes_range_frame()
{
    TEST_ASSERT_TRUE(s_service_initialized);
    TEST_ASSERT_TRUE(s_service.is_available());
    TEST_ASSERT_EQUAL(ESP_OK, s_service.start());

    ToFPerceptionFrame frame;
    bool received = false;
    for (int attempt = 0; attempt < 30 && !received; ++attempt)
    {
        vTaskDelay(pdMS_TO_TICKS(50));
        received = s_service.get_latest_frame(&frame);
    }

    TEST_ASSERT_TRUE_MESSAGE(received, "Timed out waiting for a ToF frame");
    TEST_ASSERT_EQUAL_UINT8(VL53L5CX_RESOLUTION_4X4, frame.resolution);
    TEST_ASSERT_TRUE(frame.sequence > 0);
    TEST_ASSERT_TRUE(frame.timestamp_us > 0);
    s_service.stop();
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
    RUN_TEST(test_service_publishes_range_frame);
    UNITY_END();
}
