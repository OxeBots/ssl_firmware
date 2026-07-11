#include "AS5600.h"
#include "I2Cdev.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "unity.h"

const char * TAG = "AS5600_MUX_TEST";

i2c_master_bus_handle_t i2c_bus_handle;

// Pinos I2C
#define I2C_MASTER_SDA_IO GPIO_NUM_21
#define I2C_MASTER_SCL_IO GPIO_NUM_22

// Pinos do MUX
#define MUX_A GPIO_NUM_32
#define MUX_B GPIO_NUM_33

// Ponteiro do encoder
AS5600 * encoder = nullptr;
uint16_t raw_angle = 0;

void gpio_setup()
{
    gpio_reset_pin(MUX_A);
    gpio_reset_pin(MUX_B);
    
    gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << MUX_A) | (1ULL << MUX_B),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

void i2c_setup(void)
{
    i2c_master_bus_config_t i2c_mst_config = {};
    i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_mst_config.i2c_port = I2C_NUM_0;
    i2c_mst_config.scl_io_num = I2C_MASTER_SCL_IO;
    i2c_mst_config.sda_io_num = I2C_MASTER_SDA_IO;
    i2c_mst_config.glitch_ignore_cnt = 7;
    i2c_mst_config.flags.enable_internal_pullup = true;
    
    esp_err_t bus_err = i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle);
    if (bus_err != ESP_OK) {
        ESP_LOGE(TAG, "Fail to Create I2C Channel");
    } else {
        ESP_LOGI(TAG, "I2C Channel Created");
        I2Cdev::init(i2c_bus_handle);
    }
}

void MUX_READ_RAW_ANGLE(bool MUX_CHANNEL_1, bool MUX_CHANNEL_2)
{
    //seta as portas do mux
    gpio_set_level(MUX_A, MUX_CHANNEL_1);
    gpio_set_level(MUX_B, MUX_CHANNEL_2);
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_err_t err = encoder->get_i2c_raw_angle(&raw_angle);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "CH [%i, %i] -> RAW_ANGLE: %u", MUX_CHANNEL_1, MUX_CHANNEL_2, raw_angle);
    } else {
        ESP_LOGE(TAG, "CH [%i, %i] -> Falha de comunicacao com sensor", MUX_CHANNEL_1, MUX_CHANNEL_2);
    }
}

extern "C" void app_main()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Configurações de gpio e i2c
    gpio_setup();
    i2c_setup();
    
    //Instanciamento do encoder
    encoder = new AS5600(ADC_CHANNEL_0, nullptr, false, ADC_UNIT_1, ADC_BITWIDTH_12);
    if (encoder->init_i2c() == ESP_OK) {
        ESP_LOGI(TAG, "AS5600 I2C inicializado com sucesso.");
    } else {
        ESP_LOGE(TAG, "Erro crítico ao inicializar AS5600.");
    }

    // Loop principal
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        MUX_READ_RAW_ANGLE(0, 0);
        MUX_READ_RAW_ANGLE(0, 1);
        MUX_READ_RAW_ANGLE(1, 0);
        MUX_READ_RAW_ANGLE(1, 1);
        ESP_LOGI(TAG, "==============================================");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}