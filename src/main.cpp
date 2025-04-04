#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "driver/ledc.h"
#include "kinematics/omnidirectional_robot.h"

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

struct RobotCommand
{
    float angle;
    float x_velocity;
    float y_velocity;
};

// LED Pins
#define BLINK_GPIO (gpio_num_t) CONFIG_BLINK_GPIO

void heartbeat_task(void * pvParam)
{
    // Configure LEDC timer for 1 Hz (1-second period)
    ledc_timer_config_t timer_config = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                        .duty_resolution = LEDC_TIMER_10_BIT,
                                        .timer_num = LEDC_TIMER_0,
                                        .freq_hz = 1,
                                        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&timer_config);

    // Configure LEDC channel with 50% duty cycle
    ledc_channel_config_t channel_config = {
      .gpio_num = BLINK_GPIO,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LEDC_CHANNEL_0,
      .timer_sel = LEDC_TIMER_0,
      .duty = (1 << 10) / 2,  // 50% of 1024 = 512
      .hpoint = 0};
    ledc_channel_config(&channel_config);

    // Task no longer needed, delete itself to free CPU resources
    vTaskDelete(NULL);
}

void remoteXY_task(void * pvParameters)
{
    RemoteXY_Init();
    while (true)
    {
        // If there is no command, the handler will exit quickly and not block
        // the CPU, but it needs to be called periodically to check if it
        // receives data from the phone
        RemoteXY_Handler();

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// Function to convert the joystick values to velocity and angle of the chassis

RobotCommand convertJoystickToRobotCommand()
{
    constexpr float X_VEL_LIMIT = 1.0;       // Maximum X velocity
    constexpr float Y_VEL_LIMIT = 1.0;       // Maximum Y velocity
    constexpr float ANGLE_LIMIT = 2 * M_PI;  // Maximum angle

    RobotCommand command;
    // Clamp joystick values to the limits
    command.x_velocity =
      constrain(RemoteXY.joystick_01_x, -X_VEL_LIMIT, X_VEL_LIMIT);
    command.y_velocity =
      constrain(RemoteXY.joystick_01_y, -Y_VEL_LIMIT, Y_VEL_LIMIT);
    // Convert the angle between joystick_02_x and joystick_02_y in rad angle
    // using atan2
    command.angle = atan2(RemoteXY.joystick_02_y, RemoteXY.joystick_02_x);
    // Normalize the angle to be between -ANGLE_LIMIT / 2 and ANGLE_LIMIT / 2
    // in one line
    command.angle = fmod(command.angle + M_PI, 2 * M_PI) - M_PI;
    command.angle =
      constrain(command.angle, -ANGLE_LIMIT / 2, ANGLE_LIMIT / 2);

    return command;
}

void setup()
{
    Serial.begin(BAUD_RATE);

    // Blinking LED task
    xTaskCreate(heartbeat_task, "LED Blink", configMINIMAL_STACK_SIZE, nullptr,
                5, nullptr);

    // RemoteXY task
    xTaskCreate(remoteXY_task, "RemoteXY", 20240, NULL, 2, NULL);

    // Create the robot object
    OmnidirectionalRobot robot(0.1, 0.2);
}

void loop()
{
    vTaskDelete(NULL);  // FreeRTOS takes over
}
