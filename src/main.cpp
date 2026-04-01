/**
 * @file main.cpp
 * @brief Application entry point — pure initialization sequencer.
 *
 * Responsibilities of this file:
 *   1. Initialize each subsystem in dependency order.
 *   2. Launch FreeRTOS tasks via each subsystem's start method.
 *   3. Enter the idle/Teleplot output loop.
 *
 * This file contains NO global variables and NO free-standing runtime
 * functions. All runtime logic lives in the appropriate component under lib/.
 *
 * Boot sequence (see FSD §2.3.2):
 *   Phase 1: NVS (no dependencies)
 *   Phase 2: Robot identity (depends on NVS)
 *   Phase 3: GPIO ISR service (must precede any GPIO interrupt registration)
 *   Phase 4: IMU hardware driver init (IMUGY85)
 *   Phase 5: Wheel encoders (ADC) + NVS ADC callbacks
 *   Phase 6: Motor driver + WheelController
 *   Phase 7: Radio + ProtocolHandler + TelemetryService
 *   Phase 8: Calibration (blocking; loads NVS or runs interactive routine)
 *   Phase 9: Start orientation task (OrientationHandler @ 100 Hz)
 *   Idle:    Teleplot serial output
 *
 * -------------------------------------------------------------------------
 * Motor pin configuration
 * -------------------------------------------------------------------------
 * The BL48250 GPIO pins must be defined in sdkconfig (via menuconfig) or in
 * a board-specific header. The macros below default to safe fallback values
 * if not yet set, but MUST be configured for the physical hardware.
 *
 * Required sdkconfig keys (add to sdkconfig.defaults or Kconfig):
 *   CONFIG_MOTOR_FL_PWM_GPIO   CONFIG_MOTOR_FL_DIR_GPIO
 *   CONFIG_MOTOR_BL_PWM_GPIO   CONFIG_MOTOR_BL_DIR_GPIO
 *   CONFIG_MOTOR_BR_PWM_GPIO   CONFIG_MOTOR_BR_DIR_GPIO
 *   CONFIG_MOTOR_FR_PWM_GPIO   CONFIG_MOTOR_FR_DIR_GPIO
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
#include "TelemetryService.h"
#include "WheelController.h"
#include "omni_robot.h"
#include "wheel_state_estimator.h"

static const char * TAG = "MAIN";

// -----------------------------------------------------------------------
// Motor LEDC configuration
// -----------------------------------------------------------------------
static constexpr ledc_timer_t MOTOR_LEDC_TIMER = LEDC_TIMER_0;
static constexpr ledc_mode_t MOTOR_LEDC_SPEED_MODE = LEDC_LOW_SPEED_MODE;
static constexpr ledc_timer_bit_t j = LEDC_TIMER_10_BIT;
static constexpr uint32_t MOTOR_PWM_FREQ_HZ = 20000;

// -----------------------------------------------------------------------
// Teleplot output rate
// -----------------------------------------------------------------------
static constexpr uint32_t SERIAL_PRINT_RATE_HZ = 20;

// -----------------------------------------------------------------------
// app_main
// -----------------------------------------------------------------------

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "--- Oxebots SSL Firmware starting ---");

    // ====================================================================
    // Phase 1: NVS
    // ====================================================================
    ESP_ERROR_CHECK(NVSManager::init());
    NVSManager::start();

    // ====================================================================
    // Phase 2: Robot identity
    // ====================================================================
    RobotState::get_instance().init();
    ESP_LOGI(TAG, "Robot ID: %d", RobotState::get_instance().get_id());

    // ====================================================================
    // Phase 3: GPIO ISR service
    // Must be installed before NRF24L01 init and any GPIO interrupt handlers.
    // ====================================================================
    gpio_install_isr_service(0);

    // ====================================================================
    // Phase 4: IMU hardware driver init (IMUGY85)
    // ====================================================================
    ESP_ERROR_CHECK(OrientationHandler::get_instance().init());

    // ====================================================================
    // Phase 5: Wheel encoders (ADC)
    // ====================================================================
    const std::array<adc_channel_t, 4> enc_channels = {
      static_cast<adc_channel_t>(CONFIG_MOTOR_FL_ENC_CHANNEL),
      static_cast<adc_channel_t>(CONFIG_MOTOR_BL_ENC_CHANNEL),
      static_cast<adc_channel_t>(CONFIG_MOTOR_BR_ENC_CHANNEL),
      static_cast<adc_channel_t>(CONFIG_MOTOR_FR_ENC_CHANNEL),
    };

    WheelStateEstimator & wse = WheelStateEstimator::get_instance();
    ESP_ERROR_CHECK(wse.init(enc_channels, ADC_ATTEN_DB_12));

    // ====================================================================
    // Phase 6: Motor driver + WheelController
    // ====================================================================
    static BL48250 motor_driver(MOTOR_LEDC_TIMER, MOTOR_LEDC_SPEED_MODE, j, MOTOR_PWM_FREQ_HZ,
                                /* pwm pins  */
                                std::array<gpio_num_t, 4>{
                                  static_cast<gpio_num_t>(CONFIG_MOTOR_FL_PWM_GPIO),
                                  static_cast<gpio_num_t>(CONFIG_MOTOR_BL_PWM_GPIO),
                                  static_cast<gpio_num_t>(CONFIG_MOTOR_BR_PWM_GPIO),
                                  static_cast<gpio_num_t>(CONFIG_MOTOR_FR_PWM_GPIO),
                                },
                                /* dir pins  */
                                std::array<gpio_num_t, 4>{
                                  static_cast<gpio_num_t>(CONFIG_MOTOR_FL_DIR_GPIO),
                                  static_cast<gpio_num_t>(CONFIG_MOTOR_BL_DIR_GPIO),
                                  static_cast<gpio_num_t>(CONFIG_MOTOR_BR_DIR_GPIO),
                                  static_cast<gpio_num_t>(CONFIG_MOTOR_FR_DIR_GPIO),
                                },
                                /* channels  */
                                std::array<ledc_channel_t, 4>{
                                  LEDC_CHANNEL_0,
                                  LEDC_CHANNEL_1,
                                  LEDC_CHANNEL_2,
                                  LEDC_CHANNEL_3,
                                });

    ESP_ERROR_CHECK(WheelController::get_instance().init(&motor_driver));

    // ====================================================================
    // Phase 7: Radio + ProtocolHandler + TelemetryService
    // ====================================================================
    static NRF24L01 radio(static_cast<gpio_num_t>(CONFIG_IRQ_GPIO));

    // Kinematics model: wheel radius and robot centre-to-wheel distance
    // from the omni_robot config namespace.
    static OmnidirectionalRobot robot(config::kinematic::OMNI_WHEEL_RADIUS, config::kinematic::OMNI_WHEEL_DISTANCE);

    if (radio.init(CONFIG_RADIO_CHANNEL, 32, "ADMIN", "ESP32") == ESP_OK)
    {
        ProtocolHandler::get_instance().init(&radio, &WheelController::get_instance(), &robot);
        TelemetryService::get_instance().init(&radio);

        radio.start(
          [](const uint8_t * payload, uint8_t len) { ProtocolHandler::get_instance().on_packet(payload, len); });
        ESP_LOGI(TAG, "Radio initialized on channel %d.", CONFIG_RADIO_CHANNEL);
    }
    else
    {
        ESP_LOGE(TAG, "Radio initialization failed! Proceeding without radio link.");
    }

    // ====================================================================
    // Phase 8: Calibration (blocking; loads NVS or runs interactive routine)
    // ====================================================================
    ESP_LOGI(TAG, "Checking wheel encoder calibration...");
    if (wse.load_calibration() != ESP_OK)
    {
        ESP_LOGW(TAG, "Wheel encoder calibration missing. Robot will operate in fallback mode.");
        ESP_LOGW(TAG, "Send CONFIG_FLAG_RUN_CALIBRATION to calibrate encoders.");
    }

    // Load IMU calibrations from NVS (no auto-calibration on boot)
    if (OrientationHandler::get_instance().load_calibration() != ESP_OK)
    {
        ESP_LOGW(TAG, "Some IMU calibrations missing. Robot will operate in fallback mode.");
        ESP_LOGW(TAG, "Send CONFIG_FLAG_RUN_CALIBRATION to calibrate sensors.");
    }

    // ====================================================================
    // Phase 9: Start orientation task (100 Hz)
    // ====================================================================
    OrientationHandler::get_instance().start_task();

    ESP_LOGI(TAG, "--- All subsystems up. Entering idle loop. ---");

    // ====================================================================
    // Idle loop — Teleplot serial output
    // Uncomment the blocks you need; nothing else lives here.
    // ====================================================================
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000 / SERIAL_PRINT_RATE_HZ));

        // -- Wheel encoder telemetry (Teleplot) --------------------------
        // const auto rpms = wse.get_filtered_rpm();
        // for (int i = 0; i < 4; ++i)
        //     printf(">w_%d_rpm:%f\n", i + 1, rpms[i]);

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
