/**
 * @brief Application entry point — pure initialization sequencer.
 *
 * Responsibilities of this file:
 *   1. Initialize each subsystem in dependency order.
 *   2. Launch FreeRTOS tasks via each subsystem's start method.
 *   3. Enter the idle loop.
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
 *   6.: Shared SPI bus, 74HC595 (motor direction, CD4052B select and radio
 *       CE), then the radio hardware
 *   7.: Motor driver + WheelController
 *   8.: Calibration (blocking; loads NVS or runs interactive routine)
 *   9.: Start orientation task (OrientationHandler @ 100 Hz)
 *   Idle: idle loop.
 */

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <nvs_flash.h>
#include <sdkconfig.h>

#include "74HC595.h"
#include "BL48250.h"
#include "CD4052B.h"
#include "NRF24L01.h"
#include "NVSManager.h"
#include "OrientationHandler.h"
#include "ProtocolHandler.h"
#include "RobotState.h"
#include "SPIdev.h"
#include "Telemetry.h"
#include "ToFPerceptionService.h"
#include "WheelController.h"
#include "omni_robot.h"
#include "ssl_robot_protocol_bp.h"
#include "wheel_state_estimator.h"

// Unconditional so the PlatformIO library finder picks up lib/SerialDebug;
// the service only starts (and only emits code) when CONFIG_SERIAL_DEBUG is set.
#include "SerialDebug.h"

static const char * TAG = "MAIN";

// Motor LEDC configuration
static constexpr ledc_timer_t MOTOR_LEDC_TIMER = LEDC_TIMER_0;
static constexpr ledc_mode_t MOTOR_LEDC_SPEED_MODE = LEDC_LOW_SPEED_MODE;
static constexpr ledc_timer_bit_t MOTOR_DUTY_RESOLUTION = LEDC_TIMER_10_BIT;
static constexpr uint32_t MOTOR_PWM_FREQ_HZ = 20000;

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

    // 4: IMU hardware driver init (IMUGY85). Mandatory hardware: on failure
    // the system latches SAFE_STOP and keeps running (radio + telemetry up,
    // actuation inhibited) instead of restarting forever.
    const bool imu_ready = OrientationHandler::get_instance().init() == ESP_OK;
    if (imu_ready)
        ESP_LOGI(TAG, "IMU hardware initialized.");
    else
    {
        ESP_LOGE(TAG, "IMU hardware unavailable. Entering SAFE_STOP.");
        RobotState::get_instance().set_error(ERROR_IMU_FAIL);
        ProtocolHandler::get_instance().force_safe_stop();
    }

    // 4: IMU calibration load from NVS (must precede ADC task init) NVS flash
    // operations disable the cache; the ADC task would preempt the NVS writer
    // and crash on flash code.
    if (imu_ready && OrientationHandler::get_instance().load_calibration() != ESP_OK)
    {
        ESP_LOGW(TAG, "Some IMU calibrations missing. Robot will operate in fallback mode.");
        ESP_LOGW(TAG, "Send CONFIG_FLAG_RUN_CALIBRATION to calibrate sensors.");
    }

    // The VL53L5CX shares the IMU I2C bus. ToF is an optional perception
    // capability, so a missing sensor does not prevent safe wheel control.
    static ToFPerceptionService tof_service;
    if (tof_service.init(static_cast<gpio_num_t>(CONFIG_TOF_INT_GPIO)) == ESP_OK)
    {
        if (tof_service.start() != ESP_OK)
        {
            ESP_LOGW(TAG, "VL53L5CX ranging task failed to start.");
            RobotState::get_instance().set_error(ERROR_TOF_FAIL);
        }
        else
        {
            ESP_LOGI(TAG, "VL53L5CX perception service started.");
        }
    }
    else
    {
        ESP_LOGW(TAG, "VL53L5CX unavailable. Continuing without ToF perception.");
        RobotState::get_instance().set_error(ERROR_TOF_FAIL);
    }

    // 5: Wheel encoders. The estimator creates the shared ADC stream (encoder
    // channels plus the VMON supply channel) and registers its sample handlers.
    const std::array<adc_channel_t, 4> enc_channels = {
      static_cast<adc_channel_t>(CONFIG_MOTOR_FL_ENC_CHANNEL),
      static_cast<adc_channel_t>(CONFIG_MOTOR_BL_ENC_CHANNEL),
      static_cast<adc_channel_t>(CONFIG_MOTOR_BR_ENC_CHANNEL),
      static_cast<adc_channel_t>(CONFIG_MOTOR_FR_ENC_CHANNEL),
    };

    WheelStateEstimator & wse = WheelStateEstimator::get_instance();
    ESP_ERROR_CHECK(wse.init(enc_channels, ADC_ATTEN_DB_12));

    // 6: Radio and shared-bus peripherals. SPIdev::init() creates the shared
    // SPI bus from the sdkconfig pins; the 74HC595 and the radio both register
    // on it, and all bus traffic — radio, 74HC595 and CD4052B — goes through
    // SPIdev's transaction mutex. The 74HC595 carries the four motor direction
    // outputs, the CD4052B select lines and the radio CE line, so the shift
    // register is initialized and attached to the radio before the radio
    // hardware stage.
    SPIdev::init();

    static HC595 shift_register;
    HC595::Config output_config = {
      .latch_gpio = static_cast<gpio_num_t>(CONFIG_SHIFT_REGISTER_LATCH_GPIO),
      .device_key = static_cast<uint8_t>(CONFIG_SHIFT_REGISTER_DEVICE_KEY),
      .clock_hz = 1000000,
    };
    ESP_ERROR_CHECK(shift_register.init(output_config));

    static CD4052B cd4052b;
    CD4052B::Config mux_config = {
      .select_a_bit = static_cast<uint8_t>(CONFIG_SHIFT_REGISTER_MUX_A_BIT),
      .select_b_bit = static_cast<uint8_t>(CONFIG_SHIFT_REGISTER_MUX_B_BIT),
      .inhibit_bit = static_cast<uint8_t>(CONFIG_SHIFT_REGISTER_MUX_INH_BIT),
    };
    ESP_ERROR_CHECK(cd4052b.init(&shift_register, mux_config));
    wse.set_cd4052b(&cd4052b);

    // Radio CE is a 74HC595 output bit: the radio driver writes it through the
    // generic CE pin callback, so it stays independent of the shift register.
    static HC595::Pin radio_ce_pin{&shift_register,
                                   static_cast<uint8_t>(1U << CONFIG_SHIFT_REGISTER_RF_CE_BIT)};
    NRF24_CePin radio_ce = {};
    radio_ce.gpio = GPIO_NUM_NC;
    radio_ce.write = &HC595::Pin::write;
    radio_ce.context = &radio_ce_pin;
    static NRF24L01 radio(static_cast<gpio_num_t>(CONFIG_IRQ_GPIO), radio_ce);

    radio.init_hw();
    const bool radio_ready = radio.init(CONFIG_RADIO_CHANNEL, 32, "ADMIN", "ESP32") == ESP_OK;

    // 7: Motor driver + WheelController
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
    // Digital pulse-feedback inputs (FBM1-4) feed the hardware pulse counters.
    motor_config.motor_fb_pins = {
      static_cast<gpio_num_t>(CONFIG_MOTOR_FL_FB_GPIO),
      static_cast<gpio_num_t>(CONFIG_MOTOR_BL_FB_GPIO),
      static_cast<gpio_num_t>(CONFIG_MOTOR_BR_FB_GPIO),
      static_cast<gpio_num_t>(CONFIG_MOTOR_FR_FB_GPIO),
    };
    motor_config.direction_shift_register = &shift_register;
    motor_config.direction_bits = {
      static_cast<uint8_t>(CONFIG_SHIFT_REGISTER_DIR1_BIT),
      static_cast<uint8_t>(CONFIG_SHIFT_REGISTER_DIR2_BIT),
      static_cast<uint8_t>(CONFIG_SHIFT_REGISTER_DIR3_BIT),
      static_cast<uint8_t>(CONFIG_SHIFT_REGISTER_DIR4_BIT),
    };
    motor_config.motor_channels = {
      LEDC_CHANNEL_0,
      LEDC_CHANNEL_1,
      LEDC_CHANNEL_2,
      LEDC_CHANNEL_3,
    };

    ESP_ERROR_CHECK(BL48250::get_instance().configure(motor_config));
    ESP_ERROR_CHECK(WheelController::get_instance().init());

    // Kinematics model: wheel radius and robot centre-to-wheel distance from
    // the omni_robot config namespace.
    static OmnidirectionalRobot robot(config::kinematic::OMNI_WHEEL_RADIUS,
                                      config::kinematic::OMNI_WHEEL_DISTANCE);

    // 8: ProtocolHandler + Telemetry
    if (radio_ready)
    {
        ProtocolHandler::get_instance().init(&radio, &WheelController::get_instance(), &robot);
        Telemetry::get_instance().init(&radio);

        radio.start(
          [](void * pvParameters) {
              auto * event = static_cast<DataReceivedEvent *>(pvParameters);
              ProtocolHandler::get_instance().on_packet(event->payload, event->length);
          },
          nullptr);
        ESP_LOGI(TAG, "Radio initialized on channel %d.", CONFIG_RADIO_CHANNEL);
    }
    else
    {
        ESP_LOGE(TAG, "Radio initialization failed! Proceeding without radio link.");
        RobotState::get_instance().set_error(ERROR_COMM_LOSS);
    }

    // 9: Calibration (blocking; loads NVS or runs interactive routine)
    ESP_LOGI(TAG, "Checking wheel encoder calibration...");
    if (wse.load_calibration() != ESP_OK)
    {
        ESP_LOGW(TAG, "Wheel encoder calibration missing. Robot will operate in fallback mode.");
        ESP_LOGW(TAG, "Send CONFIG_FLAG_RUN_CALIBRATION to calibrate encoders.");
    }

    // 10: Start orientation task (100 Hz); skipped when the IMU is absent —
    // the task would only poll a dead driver.
    if (imu_ready)
        OrientationHandler::get_instance().start_task();

#if CONFIG_SERIAL_DEBUG
    // 11: Serial debug dashboard. Streams over UART0 only while a dashboard
    // client keeps the connection alive.
    if (SerialDebug::get_instance().init(&tof_service) == ESP_OK)
        SerialDebug::get_instance().start_task();
    else
        ESP_LOGW(TAG, "Serial debug service unavailable.");
#endif

    ESP_LOGI(TAG, "--- All subsystems up. Entering idle loop. ---");

    // Idle loop — services the radio protocol handler.
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(50));

        if (radio_ready)
            ProtocolHandler::get_instance().update(static_cast<uint64_t>(esp_timer_get_time()));
    }
}
