/**
 * @brief Application entry point — pure initialization sequencer.
 *
 * Responsibilities of this file:
 *   1. Initialize each subsystem in dependency order.
 *   2. Launch FreeRTOS tasks via each subsystem's start method.
 *   3. Enter the idle/debugging teleplot output loop.
 *
 * This file contains NO global variables and NO free-standing runtime
 * functions. All runtime logic lives in the appropriate component under lib/.
 *
 * Boot sequence:
 *   1.: NVS (no dependencies)
 *   2.: Robot state (depends on NVS)
 *   3.: GPIO ISR service (must precede any GPIO interrupt registration)
 *   4.: IMU hardware driver init (IMUGY85)
 *   5.: Wheel encoders (ADC) + NVS ADC callbacks
 *   6.: Motor driver + WheelController
 *   7.: Radio + ProtocolHandler + Telemetry
 *   8.: Calibration (blocking; loads NVS or runs interactive routine)
 *   9.: Start orientation task (OrientationHandler @ 100 Hz)
 *   Idle: debugging teleplot serial output
 */

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <sdkconfig.h>

#include "BL48250.h"
#include "NRF24L01.h"
#include "NVSManager.h"
#include "OrientationHandler.h"
#include "ProtocolHandler.h"
#include "RobotState.h"
#include "Telemetry.h"
#include "WheelController.h"
#include "omni_robot.h"
#include "vl53l5cx_api.h"
#include "wheel_state_estimator.h"

static const char * TAG = "MAIN";

// Motor LEDC configuration
static constexpr ledc_timer_t MOTOR_LEDC_TIMER = LEDC_TIMER_0;
static constexpr ledc_mode_t MOTOR_LEDC_SPEED_MODE = LEDC_LOW_SPEED_MODE;
static constexpr ledc_timer_bit_t MOTOR_DUTY_RESOLUTION = LEDC_TIMER_10_BIT;
static constexpr uint32_t MOTOR_PWM_FREQ_HZ = 20000;

// Debugging teleplot output rate
static constexpr uint32_t SERIAL_PRINT_RATE_HZ = 20;

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "--- Oxebots SSL Firmware starting ---");

    // 1: NVS
    ESP_ERROR_CHECK(NVSManager::init());
    NVSManager::start();

    // 2: Robot state (depends on NVS)
    RobotState::get_instance().init();
    ESP_LOGI(TAG, "Robot ID: %d", RobotState::get_instance().get_id());

    // 3: GPIO ISR service
    // Must be installed before NRF24L01 init and any GPIO interrupt handlers.
    gpio_install_isr_service(0);

    // 4: IMU hardware driver init (IMUGY85)
    ESP_ERROR_CHECK(OrientationHandler::get_instance().init());

    // 4: IMU calibration load from NVS (must precede ADC task init) NVS flash
    // operations disable the cache; the ADC task would preempt the NVS writer
    // and crash on flash code.
    if (OrientationHandler::get_instance().load_calibration() != ESP_OK)
    {
        ESP_LOGW(TAG, "Some IMU calibrations missing. Robot will operate in fallback mode.");
        ESP_LOGW(TAG, "Send CONFIG_FLAG_RUN_CALIBRATION to calibrate sensors.");
    }

    // 5: Wheel encoders (ADC)
    const std::array<adc_channel_t, 4> enc_channels = {
      static_cast<adc_channel_t>(CONFIG_MOTOR_FL_ENC_CHANNEL),
      static_cast<adc_channel_t>(CONFIG_MOTOR_BL_ENC_CHANNEL),
      static_cast<adc_channel_t>(CONFIG_MOTOR_BR_ENC_CHANNEL),
      static_cast<adc_channel_t>(CONFIG_MOTOR_FR_ENC_CHANNEL),
    };

    WheelStateEstimator & wse = WheelStateEstimator::get_instance();
    ESP_ERROR_CHECK(wse.init(enc_channels, ADC_ATTEN_DB_12));

    // 6: Motor driver + WheelController
    config::driver::MotorDriverConfig motor_config;
    motor_config.timer = MOTOR_LEDC_TIMER;
    motor_config.speed_mode = MOTOR_LEDC_SPEED_MODE;
    motor_config.duty_resolution = MOTOR_DUTY_RESOLUTION;
    motor_config.pwm_freq = MOTOR_PWM_FREQ_HZ;
    motor_config.motor_pwm_pins = {
      static_cast<gpio_num_t>(CONFIG_MOTOR_FL_PWM_GPIO),
      static_cast<gpio_num_t>(CONFIG_MOTOR_BL_PWM_GPIO),
      static_cast<gpio_num_t>(CONFIG_MOTOR_BR_PWM_GPIO),
      static_cast<gpio_num_t>(CONFIG_MOTOR_FR_PWM_GPIO),
    };
    motor_config.motor_dir_pins = {
      static_cast<gpio_num_t>(CONFIG_MOTOR_FL_DIR_GPIO),
      static_cast<gpio_num_t>(CONFIG_MOTOR_BL_DIR_GPIO),
      static_cast<gpio_num_t>(CONFIG_MOTOR_BR_DIR_GPIO),
      static_cast<gpio_num_t>(CONFIG_MOTOR_FR_DIR_GPIO),
    };
    motor_config.motor_channels = {
      LEDC_CHANNEL_0,
      LEDC_CHANNEL_1,
      LEDC_CHANNEL_2,
      LEDC_CHANNEL_3,
    };

    ESP_ERROR_CHECK(BL48250::get_instance().configure(motor_config));
    ESP_ERROR_CHECK(WheelController::get_instance().init());

    // 7: Radio + ProtocolHandler + Telemetry
    static NRF24L01 radio(static_cast<gpio_num_t>(CONFIG_IRQ_GPIO));

    // Kinematics model: wheel radius and robot centre-to-wheel distance from
    // the omni_robot config namespace.
    static OmnidirectionalRobot robot(config::kinematic::OMNI_WHEEL_RADIUS,
                                      config::kinematic::OMNI_WHEEL_DISTANCE);

    if (radio.init(CONFIG_RADIO_CHANNEL, 32, "ADMIN", "ESP32") == ESP_OK)
    {
        ProtocolHandler::get_instance().init(&radio, &WheelController::get_instance(), &robot);
        Telemetry::get_instance().init(&radio);

        radio.start([](const uint8_t * payload, uint8_t len) {
            ProtocolHandler::get_instance().on_packet(payload, len);
        });
        ESP_LOGI(TAG, "Radio initialized on channel %d.", CONFIG_RADIO_CHANNEL);
    }
    else
    {
        ESP_LOGE(TAG, "Radio initialization failed! Proceeding without radio link.");
    }

    // 8: Calibration (blocking; loads NVS or runs interactive routine)
    ESP_LOGI(TAG, "Checking wheel encoder calibration...");
    if (wse.load_calibration() != ESP_OK)
    {
        ESP_LOGW(TAG, "Wheel encoder calibration missing. Robot will operate in fallback mode.");
        ESP_LOGW(TAG, "Send CONFIG_FLAG_RUN_CALIBRATION to calibrate encoders.");
    }

    // 9: Start orientation task (100 Hz)
    OrientationHandler::get_instance().start_task();

    ESP_LOGI(TAG, "--- All subsystems up. Entering idle loop. ---");

    // Idle loop — debugging teleplot serial output
    // Uncomment the blocks you need; nothing else lives here.
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000 / SERIAL_PRINT_RATE_HZ));

        // --Wheel velocity telemetry(Teleplot)-- -- -- -- -- -- -- -- -- -- -- -
        // {
        //     const auto current = WheelController::get_instance().get_current_velocities();
        //     const auto target = WheelController::get_instance().get_target_velocities();
        //     static const char * labels[4] = {"FL", "BL", "BR", "FR"};
        //     for (int i = 0; i < 4; ++i)
        //     {
        //         printf(">wheel_%s_target:%.2f\n", labels[i], target[i]);
        //         printf(">wheel_%s_current:%.2f\n", labels[i], current[i]);
        //     }
        // }

        // -- Wheel encoder RPM (Teleplot) --------------------------------
        // const auto rpms = wse.get_filtered_rpm();
        // for (int i = 0; i < 4; ++i) printf(">w_%d_rpm:%f\n", i + 1, rpms[i]);

        // -- IMU telemetry (Teleplot) ------------------------------------
        // printf(">pose.yaw:%.2f\n",   OrientationHandler::get_instance().get_yaw());
        // printf(">pose.pitch:%.2f\n", OrientationHandler::get_instance().get_pitch());
        // printf(">pose.roll:%.2f\n",  OrientationHandler::get_instance().get_roll());

        // -- 3D orientation (Teleplot 3D view) ---------------------------
        // double yaw   = OrientationHandler::get_instance().get_yaw()   * M_PI / 180.0;
        // double pitch = OrientationHandler::get_instance().get_pitch() * M_PI / 180.0;
        // double roll  = OrientationHandler::get_instance().get_roll()  * M_PI / 180.0;
        // printf(">3D|IMU:R:%.4f:%.4f:%.4f:S:cube:W:3:H:1.5:D:4:C:grey|g\n",
        //        roll, pitch, yaw);
    }
}
