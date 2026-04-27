#include <driver/i2c_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unity.h>
#include "vl53l5cx_api.h"
// #include "ADXL345.h"  // Desabilitado — sensor não utilizado neste projeto
#include "I2Cdev.h"

static VL53L5CX Ball;
static VL53L5CX_ResultsData Results;  

static const char *TAG = "VL53L5CX_TEST";
static i2c_master_bus_handle_t bus_handle;


static void setup_i2c(void)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port          = CONFIG_I2C_PORT_NUM,
        .sda_io_num        = (gpio_num_t)CONFIG_SDA_GPIO,
        .scl_io_num        = (gpio_num_t)CONFIG_SCL_GPIO,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority     = 0,
        .trans_queue_depth = 0,
        .flags             = {.enable_internal_pullup = true, .allow_pd = false},
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    I2Cdev::init(bus_handle);
}


void setUp(void)
{
    // Nenhuma inicialização adicional necessária por enquanto
}

void tearDown(void)
{
    // Nenhuma limpeza adicional necessária por enquanto
}

/* -----------------------------------------------------------------------
 * Testes
 * --------------------------------------------------------------------- */

void test_connection_and_device_id(void)
{
    ESP_LOGI(TAG, "Testando conexão...");

    bool connected = Ball.test_connection();
    TEST_ASSERT_TRUE_MESSAGE(connected, "Falha na conexão com VL53L5CX");

    uint8_t devId = Ball.get_device_id();
    ESP_LOGI(TAG, "Device ID lido: 0x%02X", devId);
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(0xF0, devId, "Device ID incorreto");
    // NOTA: confirme o valor esperado (0xF0) no datasheet do VL53L5CX.
}

void test_range_basics(void)
{
    uint8_t status = 0;
    int     loop   = 0;

    status = Ball.start_ranging();
    TEST_ASSERT_EQUAL_MESSAGE(VL53L5CX_STATUS_OK, status, "Falha ao iniciar ranging");

    while (loop < 10)
    {
        // Aguarda dados disponíveis antes de ler
        uint8_t isReady = 0;
        status = Ball.check_data_ready(&isReady);

        if (status == VL53L5CX_STATUS_OK && isReady)
        {
            status = Ball.get_ranging_data(&Results);
            TEST_ASSERT_EQUAL_MESSAGE(VL53L5CX_STATUS_OK, status, "Falha ao obter dados de ranging");

            for (int i = 0; i < 16; i++)
            {
                printf("Zone: %3d  |  Status: %3u  |  Distance: %4d mm\n",
                       i,
                       Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE * i],
                       Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE * i]);
            }
            printf("\n");
            loop++;  

        else
        {
            // Aguarda antes de verificar novamente
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }

    Ball.stop_ranging();
}  


extern "C" void app_main(void)
{
    setup_i2c();

    
    uint8_t initStatus = Ball.init();
    if (initStatus != VL53L5CX_STATUS_OK)
    {
        ESP_LOGE(TAG, "Falha ao inicializar VL53L5CX (status: %u)", initStatus);
        return;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);

    UNITY_BEGIN();
    RUN_TEST(test_connection_and_device_id);  // CORRIGIDO: nome correto da função
    RUN_TEST(test_range_basics);
    UNITY_END();
}