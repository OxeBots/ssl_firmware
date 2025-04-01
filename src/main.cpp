#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

AsyncWebServer server(80);

// LED Pins
const int LED_UP = 26;
const int LED_DOWN = 27;
const int LED_LEFT = 14;
const int LED_RIGHT = 12;
const int LED_CENTER = 13;
#define BLINK_GPIO (gpio_num_t) CONFIG_BLINK_GPIO

// Synchronization primitives
SemaphoreHandle_t ledMutex;

// LED states
volatile bool ledUpState = false;
volatile bool ledDownState = false;
volatile bool ledLeftState = false;
volatile bool ledRightState = false;
volatile bool ledCenterState = false;

// WiFi credentials
const char* ssid = "Wokwi-GUEST";
const char* password = "";
const int WIFI_CHANNEL = 6;


void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}

String createHtml() {
    // Local copies of LED states for thread-safe access
    bool up {false}, down {false}, left {false}, right {false}, center {false};

    if (xSemaphoreTake(ledMutex, portMAX_DELAY) == pdTRUE) {
        up = ledUpState;
        down = ledDownState;
        left = ledLeftState;
        right = ledRightState;
        center = ledCenterState;
        xSemaphoreGive(ledMutex);
    }

    String response = R"(
      <!DOCTYPE html><html>
        <head>
          <title>ESP32 Joystick Controller</title>
          <meta name="viewport" content="width=device-width, initial-scale=1">
          <style>
            html { font-family: sans-serif; text-align: center; }
            body { display: inline-flex; flex-direction: column; }
            h1 { margin-bottom: 1.2em; }
            .joystick {
              display: grid;
              grid-template-columns: repeat(3, 1fr);
              grid-template-rows: repeat(3, 1fr);
              gap: 1em;
              width: 300px;
              height: 300px;
              margin: 0 auto;
            }
            .btn {
              background-color: #5B5;
              border: none;
              color: #fff;
              padding: 0.5em 1em;
              font-size: 2em;
              text-decoration: none;
              border-radius: 50%;
            }
            .btn.OFF { background-color: #333; }
            .btn.up { grid-column: 2; grid-row: 1; }
            .btn.down { grid-column: 2; grid-row: 3; }
            .btn.left { grid-column: 1; grid-row: 2; }
            .btn.right { grid-column: 3; grid-row: 2; }
            .btn.center { grid-column: 2; grid-row: 2; }
          </style>
        </head>
        <body>
          <h1>ESP32 Joystick Controller</h1>
          <div class="joystick">
            <a href="?direction=up" class="btn up UP_STATE">UP_STATE</a>
            <a href="?direction=left" class="btn left LEFT_STATE">LEFT_STATE</a>
            <a href="?direction=center" class="btn center CENTER_STATE">CENTER_STATE</a>
            <a href="?direction=right" class="btn right RIGHT_STATE">RIGHT_STATE</a>
            <a href="?direction=down" class="btn down DOWN_STATE">DOWN_STATE</a>
          </div>
        </body>
      </html>
    )";

    response.replace("UP_STATE", up ? "ON" : "OFF");
    response.replace("DOWN_STATE", down ? "ON" : "OFF");
    response.replace("LEFT_STATE", left ? "ON" : "OFF");
    response.replace("RIGHT_STATE", right ? "ON" : "OFF");
    response.replace("CENTER_STATE", center ? "ON" : "OFF");
    return response;
}

void wifiTask(void *pvParameters) {
    Serial.print("Connecting to WiFi... ");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password, WIFI_CHANNEL);
    
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("Failed! Retrying...");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    Serial.println(" Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    
    server.begin();
    vTaskDelete(NULL);
}

void heartbeat_task(void * pvParam)
{
  gpio_pad_select_gpio(BLINK_GPIO);
  gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
  while (true) {
    gpio_set_level(BLINK_GPIO, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(BLINK_GPIO, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup() {
    Serial.begin(BAUD_RATE);
    
    // Initialize LEDs
    pinMode(LED_UP, OUTPUT);
    pinMode(LED_DOWN, OUTPUT);
    pinMode(LED_LEFT, OUTPUT);
    pinMode(LED_RIGHT, OUTPUT);
    pinMode(LED_CENTER, OUTPUT);
    
    // Create mutex for LED state protection
    ledMutex = xSemaphoreCreateMutex();

    // Configure server routes
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        if (request->hasParam("direction")) {
            String dir = request->getParam("direction")->value();
            
            if (xSemaphoreTake(ledMutex, portMAX_DELAY) == pdTRUE) {
                // Update LED state immediately
                if (dir == "up") {
                    ledUpState = !ledUpState;
                    digitalWrite(LED_UP, ledUpState);
                } else if (dir == "down") {
                    ledDownState = !ledDownState;
                    digitalWrite(LED_DOWN, ledDownState);
                } else if (dir == "left") {
                    ledLeftState = !ledLeftState;
                    digitalWrite(LED_LEFT, ledLeftState);
                } else if (dir == "right") {
                    ledRightState = !ledRightState;
                    digitalWrite(LED_RIGHT, ledRightState);
                } else if (dir == "center") {
                    ledCenterState = !ledCenterState;
                    digitalWrite(LED_CENTER, ledCenterState);
                }
                xSemaphoreGive(ledMutex);
            }
        }
        request->send(200, "text/html", createHtml());
    });

    server.onNotFound(notFound);

    // Create WiFi task with lower priority
    xTaskCreate(wifiTask, "WiFiTask", 4096, NULL, 1, NULL);
    xTaskCreate(heartbeat_task, "LED Blink", configMINIMAL_STACK_SIZE, nullptr, 5, nullptr);

}

void loop() {
    vTaskDelete(NULL); // FreeRTOS takes over
}