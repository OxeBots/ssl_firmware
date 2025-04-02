#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// you can enable debug logging to Serial at 115200
// #define REMOTEXY__DEBUGLOG

// RemoteXY select connection mode and include library
#define REMOTEXY_MODE__ESP32CORE_BLE
#include <BLEDevice.h>

// RemoteXY connection settings
#define REMOTEXY_BLUETOOTH_NAME "OxebotsPrototype"
#include <RemoteXY.h>

// RemoteXY GUI configuration
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =  // 80 bytes
  {255, 5,   0,   0,  0,   73, 0,  19,  0,   0,  0,  0,  31,  1,   200, 84,
   1,   1,   5,   0,  2,   86, 5,  25,  13,  0,  64, 26, 31,  31,  79,  78,
   0,   79,  70,  70, 0,   5,  5,  22,  58,  58, 32, 64, 26,  31,  5,   136,
   20,  60,  60,  4,  64,  26, 31, 129, 158, 13, 17, 6,  64,  17,  71,  117,
   105, 100, 101, 0,  129, 26, 15, 16,  6,   64, 17, 77, 111, 118, 101, 0};

// this structure defines all the variables and events of the control interface
struct
{
    // input variables
    uint8_t switch_01;     // =1 if switch ON and =0 if OFF
    int8_t joystick_01_x;  // from -100 to 100
    int8_t joystick_01_y;  // from -100 to 100
    int8_t joystick_02_x;  // from -100 to 100
    int8_t joystick_02_y;  // from -100 to 100

    // other variable
    uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

// LED Pins
#define BLINK_GPIO (gpio_num_t) CONFIG_BLINK_GPIO

void heartbeat_task(void * pvParam)
{
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while (true)
    {
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void Task_RemoteXY(void * pvParameters)
{
    RemoteXY_Init();
    while (true)
    {
        RemoteXY_Handler();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void setup()
{
    Serial.begin(BAUD_RATE);

    // Blinking LED task
    xTaskCreate(heartbeat_task, "LED Blink", configMINIMAL_STACK_SIZE, nullptr,
                5, nullptr);

    // RemoteXY task
    xTaskCreate(Task_RemoteXY, "RemoteXY", 20240, NULL, 2, NULL);
}

void loop()
{
    vTaskDelete(NULL);  // FreeRTOS takes over
}
