#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdkconfig.h>
#include <stdio.h>
#include <unity.h>

#include <array>
#include <cmath>

#include "BL48250.h"
#include "I2Cdev.h"
#include "NVSManager.h"
#include "WheelController.h"
#include "constants.h"
#include "wheel_state_estimator.h"

// ============================================================================
// CONSTANTS AND CONFIGURATION
// ============================================================================

namespace config
{
namespace test
{
constexpr std::array<gpio_num_t, 4> MOTOR_PWM_PINS = {(gpio_num_t)CONFIG_MOTOR_FL_PWM_GPIO,
                                                      (gpio_num_t)CONFIG_MOTOR_BL_PWM_GPIO,
                                                      (gpio_num_t)CONFIG_MOTOR_BR_PWM_GPIO,
                                                      (gpio_num_t)CONFIG_MOTOR_FR_PWM_GPIO};

constexpr std::array<gpio_num_t, 4> MOTOR_DIR_PINS = {(gpio_num_t)CONFIG_MOTOR_FL_DIR_GPIO,
                                                      (gpio_num_t)CONFIG_MOTOR_BL_DIR_GPIO,
                                                      (gpio_num_t)CONFIG_MOTOR_BR_DIR_GPIO,
                                                      (gpio_num_t)CONFIG_MOTOR_FR_DIR_GPIO};

constexpr std::array<ledc_channel_t, 4> MOTOR_CHANNELS = {
  LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3};

constexpr ledc_timer_bit_t DUTY_RESOLUTION = LEDC_TIMER_10_BIT;
constexpr uint32_t PWM_FREQ = 20000;

const std::array<adc_channel_t, 4> WHEEL_ADC_CHANNELS = {
  static_cast<adc_channel_t>(CONFIG_MOTOR_FL_ENC_CHANNEL),
  static_cast<adc_channel_t>(CONFIG_MOTOR_BL_ENC_CHANNEL),
  static_cast<adc_channel_t>(CONFIG_MOTOR_BR_ENC_CHANNEL),
  static_cast<adc_channel_t>(CONFIG_MOTOR_FR_ENC_CHANNEL)};
}  // namespace test
}  // namespace config

// Serial buffer
static constexpr int SERIAL_BUFFER_SIZE = 64;
static char serialBuffer[SERIAL_BUFFER_SIZE];
static uint8_t serialIndex = 0;
static bool newCommand = false;
static std::array<float, 3> s_last_pid = {1.0f, 0.5f, 0.01f};

// ============================================================================
// UART / SERIAL HELPERS
// ============================================================================

static void init_uart()
{
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
}

// ============================================================================
// COMMAND PARSER
// ============================================================================

static void read_serial_data()
{
    uint8_t c;
    while (uart_read_bytes(UART_NUM_0, &c, 1, 0) > 0)
    {
        if (c == '\n' || c == '\r')
        {
            if (serialIndex > 0)
            {
                serialBuffer[serialIndex] = '\0';
                newCommand = true;
                serialIndex = 0;
                return;
            }
        }
        else if (serialIndex < (SERIAL_BUFFER_SIZE - 1))
            serialBuffer[serialIndex++] = (char)c;
        else
            serialIndex = 0;
    }
}

static bool process_serial_command(WheelController & wc)
{
    if (!newCommand)
        return false;

    newCommand = false;
    char * pointer = serialBuffer;

    // Trim leading whitespace
    while (*pointer == ' ' || *pointer == '\t')
        ++pointer;



    // T<rpm> — set target RPM for all 4 wheels
    // T<fl>,<bl>,<br>,<fr> — per-wheel RPM
    if (*pointer == 'T')
    {
        ++pointer;
        char * endptr = nullptr;
        float rpm0 = strtof(pointer, &endptr);
        if (endptr == pointer)
            return false;

        if (*endptr == ',')
        {
            // Per-wheel: T<fl>,<bl>,<br>,<fr>
            pointer = endptr + 1;
            float rpm1 = strtof(pointer, &endptr);
            pointer = endptr + 1;
            float rpm2 = strtof(pointer, &endptr);
            pointer = endptr + 1;
            float rpm3 = strtof(pointer, &endptr);

            std::array<float, 4> targets = {
              rpm0 * RPM_TO_RAD_S, rpm1 * RPM_TO_RAD_S, rpm2 * RPM_TO_RAD_S, rpm3 * RPM_TO_RAD_S};
            wc.set_target_velocities(targets);
            printf("ACK T: FL=%.1f BL=%.1f BR=%.1f FR=%.1f RPM\n", rpm0, rpm1, rpm2, rpm3);
        }
        else
        {
            // Single RPM for all wheels
            std::array<float, 4> targets = {rpm0 * RPM_TO_RAD_S,
                                            rpm0 * RPM_TO_RAD_S,
                                            rpm0 * RPM_TO_RAD_S,
                                            rpm0 * RPM_TO_RAD_S};
            wc.set_target_velocities(targets);
            printf("ACK T: all wheels %.1f RPM\n", rpm0);
        }
        return true;
    }

    // P<kp>,<ki>,<kd> — set PID gains for all wheels
    if (*pointer == 'P')
    {
        ++pointer;
        char * endptr = nullptr;
        float new_kp = strtof(pointer, &endptr);
        if (endptr == pointer)
            return false;
        pointer = endptr + 1;
        float new_ki = strtof(pointer, &endptr);
        pointer = endptr + 1;
        float new_kd = strtof(pointer, &endptr);

        std::array<float, 4> kp = {new_kp, new_kp, new_kp, new_kp};
        std::array<float, 4> ki = {new_ki, new_ki, new_ki, new_ki};
        std::array<float, 4> kd = {new_kd, new_kd, new_kd, new_kd};
        wc.set_pid_tunings(kp, ki, kd);
        s_last_pid = {new_kp, new_ki, new_kd};
        printf("ACK P: Kp=%.3f Ki=%.3f Kd=%.3f\n", new_kp, new_ki, new_kd);
        return true;
    }

    // S<kp_fl>,<ki_fl>,<kd_fl>,<kp_bl>,<ki_bl>,<kd_bl>,<kp_br>,<ki_br>,<kd_br>,<kp_fr>,<ki_fr>,<kd_fr>
    if (*pointer == 'S')
    {
        ++pointer;
        char * endptr = nullptr;
        float gains[12];
        bool ok = true;
        for (int i = 0; i < 12; ++i)
        {
            gains[i] = strtof(pointer, &endptr);
            if (endptr == pointer)
            {
                ok = false;
                break;
            }
            pointer = (*endptr == ',') ? endptr + 1 : endptr;
        }
        if (ok)
        {
            std::array<float, 4> kp = {gains[0], gains[3], gains[6], gains[9]};
            std::array<float, 4> ki = {gains[1], gains[4], gains[7], gains[10]};
            std::array<float, 4> kd = {gains[2], gains[5], gains[8], gains[11]};
            wc.set_pid_tunings(kp, ki, kd);
            s_last_pid = {kp[0], ki[0], kd[0]};
            printf("ACK S: per-wheel PID set\n");
        }
        return true;
    }

    // 0 — stop all wheels
    if (*pointer == '0')
    {
        wc.set_target_velocities({0.0f, 0.0f, 0.0f, 0.0f});
        printf("ACK 0: all wheels stopped\n");
        return true;
    }

    // D — save current PID gains to NVS
    if (*pointer == 'D')
    {
        wc.save_all_pid_to_nvs();
        printf("ACK D: PID gains saved to NVS\n");
        return true;
    }

    printf("ERR: unknown command '%s'\n", serialBuffer);
    return false;
}

// ============================================================================
// TELEMETRY (Teleplot format)
// ============================================================================

static void serial_print_telemetry(uint32_t now_ms)
{
    auto & wc = WheelController::get_instance();
    auto targets = wc.get_target_velocities();
    auto current = wc.get_current_velocities();
    auto outputs = wc.get_pid_outputs();

    // --- Essential traces only (reduced from 24 to 12 lines) ---
    // Command RPM targets (convert rad/s → rpm for display)
    printf(">FL_cmd_rpm:%.1f\n", targets[0] * RAD_S_TO_RPM);
    printf(">BL_cmd_rpm:%.1f\n", targets[1] * RAD_S_TO_RPM);
    printf(">BR_cmd_rpm:%.1f\n", targets[2] * RAD_S_TO_RPM);
    printf(">FR_cmd_rpm:%.1f\n", targets[3] * RAD_S_TO_RPM);

    // PID gains
    printf(">Kp:%.3f\n", s_last_pid[0]);
    printf(">Ki:%.3f\n", s_last_pid[1]);
    printf(">Kd:%.3f\n", s_last_pid[2]);

    // PWM outputs (signed)
    printf(">FL_pwm:%.0f\n", outputs[0]);
    printf(">BL_pwm:%.0f\n", outputs[1]);
    printf(">BR_pwm:%.0f\n", outputs[2]);
    printf(">FR_pwm:%.0f\n", outputs[3]);

    // Actual RPM (convert rad/s → rpm for display)
    printf(">FL_rpm:%.1f\n", current[0] * RAD_S_TO_RPM);
    printf(">BL_rpm:%.1f\n", current[1] * RAD_S_TO_RPM);
    printf(">BR_rpm:%.1f\n", current[2] * RAD_S_TO_RPM);
    printf(">FR_rpm:%.1f\n", current[3] * RAD_S_TO_RPM);

    (void)now_ms;
}

// ============================================================================
// MAIN TEST
// ============================================================================

void test_wheel_controller_interactive()
{
    static bool initialized = false;
    if (!initialized)
    {
        // NVS
        NVSManager::init();

        // I2C for AS5600
        static i2c_master_bus_handle_t bus_handle;
        i2c_master_bus_config_t i2c_mst_config = {
          .i2c_port = (i2c_port_t)CONFIG_I2C_PORT_NUM,
          .sda_io_num = (gpio_num_t)CONFIG_SDA_GPIO,
          .scl_io_num = (gpio_num_t)CONFIG_SCL_GPIO,
          .clk_source = I2C_CLK_SRC_DEFAULT,
          .glitch_ignore_cnt = 7,
          .intr_priority = 0,
          .trans_queue_depth = 0,
          .flags = {.enable_internal_pullup = true, .allow_pd = false}};
        i2c_new_master_bus(&i2c_mst_config, &bus_handle);
        I2Cdev::init(bus_handle);

        // ADC / Estimator
        WheelStateEstimator::get_instance().init(config::test::WHEEL_ADC_CHANNELS, ADC_ATTEN_DB_12);

        // Motor driver
        config::driver::MotorDriverConfig motor_cfg;
        motor_cfg.timer = LEDC_TIMER_0;
        motor_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
        motor_cfg.duty_resolution = config::test::DUTY_RESOLUTION;
        motor_cfg.pwm_freq = config::test::PWM_FREQ;
        motor_cfg.motor_pwm_pins = config::test::MOTOR_PWM_PINS;
        motor_cfg.motor_dir_pins = config::test::MOTOR_DIR_PINS;
        motor_cfg.motor_channels = config::test::MOTOR_CHANNELS;
        BL48250::get_instance().configure(motor_cfg);

        // WheelController
        WheelController::get_instance().init();

        initialized = true;
    }

    WheelController & wc = WheelController::get_instance();

    printf("\n=======================================================\n");
    printf("    WheelController Interactive PID Test\n");
    printf("=======================================================\n");
    printf("Features: FastPID (fixed-point), feed-forward,\n");
    printf("  startup boost, slew-rate limiting, anti-windup,\n");
    printf("  relay-feedback auto-tuning\n");
    printf("=======================================================\n");
    printf("Commands:\n");
    printf("  T<rpm>              - Set target RPM for all wheels\n");
    printf("  T<fl>,<bl>,<br>,<fr> - Set per-wheel RPM targets\n");
    printf("  P<kp>,<ki>,<kd>     - Set PID gains (all wheels)\n");
    printf("  S<kp_fl>,<ki_fl>,<kd_fl>,... - Per-wheel PID\n");
    printf("  0                   - Stop all wheels\n");
    printf("  D                   - Save PID gains to NVS\n");
    printf("=======================================================\n");
    printf("Telemetry at 20 Hz — use with Teleplot VS Code extension\n");
    printf("  >FL_cmd_rpm, >FL_target, >FL_current, >FL_pwm, >FL_rpm\n");
    printf("  >Kp, >Ki, >Kd\n");
    printf("=======================================================\n");

    uint32_t last_telem_ms = 0;
    constexpr uint32_t TELEMETRY_INTERVAL_MS = 50;  // 20 Hz

    while (true)
    {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        // Service serial commands
        read_serial_data();
        process_serial_command(wc);

        // Telemetry at 20 Hz
        if (now_ms - last_telem_ms >= TELEMETRY_INTERVAL_MS)
        {
            serial_print_telemetry(now_ms);
            last_telem_ms = now_ms;
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // 1 tick at 100 Hz → guarantees IDLE runs
    }
}

extern "C" void app_main(void)
{
    init_uart();
    vTaskDelay(pdMS_TO_TICKS(2000));

    UNITY_BEGIN();
    RUN_TEST(test_wheel_controller_interactive);
    UNITY_END();
}
