#include "AS5600.h"
#include "I2Cdev.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "unity.h"

const char * TAG = "ASTEST";

i2c_master_bus_handle_t i2c_bus_handle;

// i2c pins
#define I2C_MASTER_SDA_IO GPIO_NUM_21
#define I2C_MASTER_SCL_IO GPIO_NUM_22
#define MUX_A GPIO_NUM_27
#define MUX_B GPIO_NUM_14
// declaring encoder pointer
AS5600 * encoder = nullptr;

uint16_t raw_angle = 0;

void gpio_setup(void)
{
    gpio_set_direction(GPIO_NUM_27, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_14, GPIO_MODE_OUTPUT);
}
void i2c_setup(void)
{
    i2c_master_bus_config_t i2c_mst_config = {};
    i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_mst_config.i2c_port = -1;  // Deixa o ESP escolher a porta livre
    i2c_mst_config.scl_io_num = I2C_MASTER_SCL_IO;
    i2c_mst_config.sda_io_num = I2C_MASTER_SDA_IO;
    i2c_mst_config.glitch_ignore_cnt = 7;
    i2c_mst_config.flags.enable_internal_pullup = true;
    esp_err_t bus_err = i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle);
    if (bus_err != ESP_OK)
    {
        ESP_LOGE(TAG, "Fail to Create I2C Channel");
    }
    else
    {
        ESP_LOGE(TAG, "I2C Channel Created");
        I2Cdev::init(i2c_bus_handle);
    }
}
void MUX_READ_RAW_ANGLE(int MUX_CHANNEL_1, int MUX_CHANNEL_2)
{
    gpio_set_level(MUX_A, MUX_CHANNEL_1);
    gpio_set_level(MUX_B, MUX_CHANNEL_2);
    ESP_LOGI(TAG, "CHANNEL %i %i SELECTED", MUX_CHANNEL_1, MUX_CHANNEL_2);
    vTaskDelay(pdMS_TO_TICKS(1000));
    encoder = new AS5600(ADC_CHANNEL_0, nullptr, false, ADC_UNIT_1, ADC_BITWIDTH_12);
    esp_err_t err = encoder->init_i2c();
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Encoder I2C bus initialized");
        encoder->get_i2c_raw_angle(&raw_angle);
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP_LOGI(TAG, "RAW_ANGLE: %u", raw_angle);
    }
    else
    {
        ESP_LOGE(TAG, "No response from i2c encoder");
    }
}
extern "C" void app_main()
{
    // I2C CONFIG INIT AND VERIFY
    i2c_setup();
    while (1)
    {
        MUX_READ_RAW_ANGLE(0, 0);
        MUX_READ_RAW_ANGLE(0, 1);
        MUX_READ_RAW_ANGLE(1, 0);
        MUX_READ_RAW_ANGLE(1, 1);
    }
}