/**
 * @brief Application entry point — pure initialization sequencer.
 *
 * Responsibilities of this file:
 *   1. Initialize each subsystem in dependency order.
 *   2. Launch FreeRTOS tasks via each subsystem's start method.
 *   3. Enter the idle loop.
 *
 * This file contains no global service state and no free-standing runtime
 * functions: the module tag and compile-time configuration constants are the
 * only file-scope declarations, and the service objects are local to
 * app_main(). All runtime logic lives in the appropriate component under lib/.
 *
 * Boot sequence:
 *   1.: NVS (no dependencies)
 *   2.: Robot state (depends on NVS)
 *   3.: GPIO ISR service (must precede any GPIO interrupt registration)
 *   4.: Shared SPI bus and shift register, with the encoder mux disabled
 *   5.: Shared I2C bus and IMU hardware driver
 *   6.: IMU calibration load from NVS
 *   7.: Kicker and ToF perception (optional, use the shared I2C bus)
 *   8.: Wheel encoders (ADC) and encoder calibration
 *   9.: Radio hardware
 *  10.: Motor driver + WheelController
 *  11.: Kinematics, ProtocolHandler and Telemetry
 *  12.: Start orientation and kicker tasks
 *  13.: Serial debug dashboard (optional)
 *  Idle: idle loop.
 */

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <sdkconfig.h>

#include "74HC595.h"
#include "AS5600.h"
#include "BL48250.h"
#include "CD4052B.h"
#include "I2Cdev.h"
#include "NRF24L01.h"
#include "NVSManager.h"
#include "OrientationHandler.h"
#include "ProtocolHandler.h"
#include "RadioAddressing.h"
#include "RobotState.h"
#include "SPIdev.h"
#include "Telemetry.h"
#include "ToFPerceptionService.h"
#include "WheelController.h"
#include "kicker_controller.h"
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
static constexpr uint32_t MOTOR_PWM_FREQ_HZ = CONFIG_MOTOR_PWM_FREQ_HZ;

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
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    // 4: Disable the encoder mux before checking the shared bus. The shift
    // register can retain a selected encoder across an ESP32 reset.
    ESP_ERROR_CHECK(SPIdev::init());

    static HC595 shift_register;
    HC595::Config output_config = {
      .latch_gpio = static_cast<gpio_num_t>(CONFIG_SHIFT_REGISTER_LATCH_GPIO),
      .device_key = static_cast<uint8_t>(CONFIG_SHIFT_REGISTER_DEVICE_KEY),
      .clock_hz = 1000000,
    };
    const uint8_t initial_shift_outputs =
      static_cast<uint8_t>(1U << CONFIG_SHIFT_REGISTER_MUX_INH_BIT);
    ESP_ERROR_CHECK(shift_register.init(output_config, initial_shift_outputs));

    static CD4052B cd4052b;
    CD4052B::Config mux_config = {
      .select_a_bit = static_cast<uint8_t>(CONFIG_SHIFT_REGISTER_MUX_A_BIT),
      .select_b_bit = static_cast<uint8_t>(CONFIG_SHIFT_REGISTER_MUX_B_BIT),
      .inhibit_bit = static_cast<uint8_t>(CONFIG_SHIFT_REGISTER_MUX_INH_BIT),
    };
    ESP_ERROR_CHECK(cd4052b.init(&shift_register, mux_config));

    // 5: Bring up the shared bus before probing any of its devices.
    const esp_err_t i2c_result = I2Cdev::init();
    if (i2c_result != ESP_OK)
        ESP_LOGW(TAG, "Shared I2C bus unavailable: %s", esp_err_to_name(i2c_result));

    const auto i2c_bus = I2Cdev::get_bus_handle();
    const bool imu_ready =
      i2c_bus != nullptr && OrientationHandler::get_instance().init(i2c_bus) == ESP_OK;
    if (imu_ready)
        ESP_LOGI(TAG, "IMU hardware initialized.");
    else
    {
        ESP_LOGW(TAG, "IMU hardware unavailable.");
        RobotState::get_instance().set_error(ERROR_IMU_FAIL);
    }

    // 6: IMU calibration load from NVS (must precede ADC task init). NVS flash
    // operations disable the cache; the ADC task would preempt the NVS writer
    // and crash on flash code.
    if (imu_ready && OrientationHandler::get_instance().load_calibration() != ESP_OK)
    {
        ESP_LOGW(TAG, "Some IMU calibrations missing. Robot will operate in fallback mode.");
        ESP_LOGW(TAG, "Send CONFIG_FLAG_RUN_CALIBRATION to calibrate sensors.");
    }

    // 7: Kicker and ToF perception service (optional).
    auto & kicker = KickerController::get_instance();
    const bool kicker_ready = kicker.init(i2c_bus) == ESP_OK;
    if (!kicker_ready)
    {
        ESP_LOGW(TAG, "Kicker unavailable. Continuing without kicker.");
        RobotState::get_instance().set_error(ERROR_KICKER_NO_CHARGE);
    }

    static ToFPerceptionService tof_service;
    if (tof_service.init(i2c_bus, static_cast<gpio_num_t>(CONFIG_TOF_INT_GPIO)) == ESP_OK)
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

    // 8: Wheel encoders. The estimator creates the shared ADC stream (encoder
    // channels plus the VMON supply channel) and registers its sample handlers.
    const std::array<adc_channel_t, 4> enc_channels = {
      static_cast<adc_channel_t>(CONFIG_MOTOR_FL_ENC_CHANNEL),
      static_cast<adc_channel_t>(CONFIG_MOTOR_BL_ENC_CHANNEL),
      static_cast<adc_channel_t>(CONFIG_MOTOR_BR_ENC_CHANNEL),
      static_cast<adc_channel_t>(CONFIG_MOTOR_FR_ENC_CHANNEL),
    };

    WheelStateEstimator & wse = WheelStateEstimator::get_instance();
    ESP_ERROR_CHECK(wse.init(enc_channels, ADC_ATTEN_DB_12));
    wse.set_cd4052b(&cd4052b);

    // The ADC range calibration assumes the AS5600 reduced analog output
    // (0.1..0.9 VCC, the ESP32 ADC linear range): enforce it on every boot.
    if (wse.configure_encoder_output_stage(AS5600::OutputStage::ANALOG_REDUCED) != ESP_OK)
        ESP_LOGW(TAG, "Not all encoders are in the reduced output stage.");

    // 9: Radio hardware. The radio driver writes the 74HC595 CE output through the
    // generic CE pin callback, so it stays independent of the shift register.
    static HC595::Pin radio_ce_pin{&shift_register,
                                   static_cast<uint8_t>(1U << CONFIG_SHIFT_REGISTER_RF_CE_BIT)};
    NRF24_CePin radio_ce = {};
    radio_ce.gpio = GPIO_NUM_NC;
    radio_ce.write = &HC595::Pin::write;
    radio_ce.context = &radio_ce_pin;
    static NRF24L01 radio(static_cast<gpio_num_t>(CONFIG_IRQ_GPIO), radio_ce);

    radio.init_hw();
    const uint8_t robot_id = RobotState::get_instance().get_id();
    const uint8_t radio_channel = RobotState::get_instance().get_radio_channel();
    const std::array<uint8_t, 5> base_addr = radio_addressing::make_base_address();
    const std::array<uint8_t, 5> robot_addr = radio_addressing::make_robot_address(robot_id);
    bool radio_ready =
      radio.init_bytes(radio_channel, 32, base_addr.data(), robot_addr.data()) == ESP_OK;
    if (radio_ready)
        radio.enable_broadcast_pipe(radio_addressing::BROADCAST_ADDR_LSB);

    // 10: Motor driver + WheelController
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

    // Load wheel encoder calibration before starting the radio receiver.
    ESP_LOGI(TAG, "Checking wheel encoder calibration...");
    if (wse.load_calibration() != ESP_OK)
        ESP_LOGW(TAG, "Wheel encoder calibration missing. Calibrate encoders from Robot Bench.");

    // Kinematics model: wheel radius and robot centre-to-wheel distance from
    // the omni_robot config namespace.
    static OmnidirectionalRobot robot(config::kinematic::OMNI_WHEEL_RADIUS,
                                      config::kinematic::OMNI_WHEEL_DISTANCE);

    // 11: Kinematics, ProtocolHandler and Telemetry
    auto & protocol = ProtocolHandler::get_instance();
    protocol.init(radio_ready ? &radio : nullptr, &WheelController::get_instance(), &robot);
    if (radio_ready)
    {
        Telemetry::get_instance().init(&radio);

        if (radio.start(
              [](void * pvParameters) {
                  auto * event = static_cast<DataReceivedEvent *>(pvParameters);
                  ProtocolHandler::get_instance().on_packet(event->payload, event->length);
              },
              nullptr))
        {
            ESP_LOGI(TAG, "Radio initialized on channel %d.", CONFIG_RADIO_CHANNEL);
        }
        else
        {
            ESP_LOGE(TAG, "Radio receiver task failed to start. Proceeding without radio link.");
            RobotState::get_instance().set_error(ERROR_COMM_LOSS);
        }
    }
    else
    {
        ESP_LOGW(TAG, "Radio initialization failed. Proceeding without radio link.");
        RobotState::get_instance().set_error(ERROR_COMM_LOSS);
    }

    // 12: Start orientation task (100 Hz); skipped when the IMU is absent —
    // the task would only poll a dead driver.
    if (imu_ready)
        OrientationHandler::get_instance().start_task();
    if (kicker_ready)
    {
        if (kicker.start() != ESP_OK)
        {
            ESP_LOGW(TAG, "Kicker task failed to start.");
            RobotState::get_instance().set_error(ERROR_KICKER_NO_CHARGE);
        }
        else
        {
            ESP_LOGI(TAG, "Kicker controller started");
        }
    }
#if CONFIG_SERIAL_DEBUG
    // 13: Serial debug dashboard. Streams over UART0 only while a dashboard
    // client keeps the connection alive.
    if (SerialDebug::get_instance().init(&tof_service, &cd4052b) == ESP_OK)
        SerialDebug::get_instance().start_task();
    else
        ESP_LOGW(TAG, "Serial debug service unavailable.");
#endif

    ESP_LOGI(TAG, "--- All subsystems up. Entering idle loop. ---");

    // Idle loop — services the radio protocol handler.
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(50));

        protocol.update();
    }
}
