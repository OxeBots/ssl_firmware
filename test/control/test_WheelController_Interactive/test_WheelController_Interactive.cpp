#include <74HC595.h>
#include <CD4052B.h>
#include <SPIdev.h>
#include <driver/i2c_master.h>
#include <driver/spi_master.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdkconfig.h>
#include <stdio.h>
#include <unity.h>

#include <array>
#include <bitset>
#include <cmath>

#include "AS5600.h"
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

constexpr std::array<uint8_t, 4> MOTOR_DIR_BITS = {CONFIG_SHIFT_REGISTER_DIR1_BIT,
                                                   CONFIG_SHIFT_REGISTER_DIR2_BIT,
                                                   CONFIG_SHIFT_REGISTER_DIR3_BIT,
                                                   CONFIG_SHIFT_REGISTER_DIR4_BIT};

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

static HC595 s_shift_register;
static CD4052B s_cd4052b;

// I2C-only handle used to probe the AS5600 encoders on demand (E command).
// The ADC constructor parameters are unused; raw angles come over I2C through
// the CD4052B mux, one wheel at a time. The ADC estimator task never touches
// the mux, so both paths can run concurrently.
static AS5600 s_stream_encoder(ADC_CHANNEL_0, nullptr, false);
static i2c_master_bus_handle_t s_i2c_bus = nullptr;
static bool s_manual_mode = false;  // true = control task paused, raw 595 writes allowed

// Serial buffer
static constexpr int SERIAL_BUFFER_SIZE = 64;
static char serialBuffer[SERIAL_BUFFER_SIZE];
static uint8_t serialIndex = 0;
static bool newCommand = false;

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
    while (*pointer == ' ' || *pointer == '\t') ++pointer;

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
            std::array<float, 4> targets = {
              rpm0 * RPM_TO_RAD_S, rpm0 * RPM_TO_RAD_S, rpm0 * RPM_TO_RAD_S, rpm0 * RPM_TO_RAD_S};
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

    // E<channel> — probe the AS5600 on mux channel 0-3 (wheel order
    //             FL/BL/BR/FR) over I2C and report whether it answers
    // E-1 — release the mux (inhibit)
    if (*pointer == 'E')
    {
        ++pointer;
        char * endptr = nullptr;
        float channel = strtof(pointer, &endptr);
        if (endptr == pointer)
            return false;

        if (channel < 0)
        {
            esp_err_t err = s_cd4052b.disable();
            printf("ACK E: mux released (disable %s)\n", esp_err_to_name(err));
            return true;
        }

        if (channel > 3)
        {
            printf("ERR E: channel %.0f out of range 0-3\n", channel);
            return true;
        }

        uint8_t ch = (uint8_t)channel;
        if (s_cd4052b.select(ch) != ESP_OK)
        {
            printf("ERR E: mux select ch%u failed\n", ch);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));

        if (!s_stream_encoder.is_i2c_initialized() && s_stream_encoder.init_i2c() != ESP_OK)
        {
            printf("ERR E: no AS5600 answered on mux ch%u\n", ch);
            return true;
        }

        uint16_t raw = 0;
        if (s_stream_encoder.get_i2c_raw_angle(&raw) == ESP_OK)
            printf("ACK E: AS5600 on mux ch%u, raw angle %u (%.1f deg)\n",
                   ch,
                   (unsigned)raw,
                   raw * (360.0f / 4096.0f));
        else
            printf("ERR E: AS5600 I2C read failed on mux ch%u\n", ch);
        return true;
    }

    // I — probe every valid I2C address; the AS5600 answers at 0x36 when its
    // mux channel is selected (E<channel>)
    if (*pointer == 'I')
    {
        printf("I2C scan:");
        int found = 0;
        for (uint8_t addr = 0x08; addr <= 0x77; ++addr)
        {
            if (i2c_master_probe(s_i2c_bus, addr, 20) == ESP_OK)
            {
                printf(" 0x%02X%s", addr, (addr == AS5600::AS5600_ADDR) ? "(AS5600)" : "");
                ++found;
            }
        }
        if (found == 0)
            printf(" none");
        printf("\n");
        return true;
    }

    // M — toggle manual mode: pause/resume the motor control task so raw
    // 74HC595 patterns written with the G command are not overwritten every
    // control cycle. Motors are stopped before pausing.
    if (*pointer == 'M')
    {
        if (!s_manual_mode)
        {
            wc.set_target_velocities({0.0f, 0.0f, 0.0f, 0.0f});
            vTaskDelay(pdMS_TO_TICKS(100));  // let the control task settle at zero duty
            wc.deinit();
            s_manual_mode = true;
            printf("ACK M: manual mode ON - control task paused, motors stopped\n");
        }
        else
        {
            wc.init();
            s_manual_mode = false;
            printf("ACK M: manual mode OFF - control task running\n");
        }
        return true;
    }

    // G<hex> — write a raw byte to the 74HC595 for probing the outputs with a
    // meter (manual mode only). Bit map: 0..3 = DIR FL/BL/BR/FR, 4/5 = mux A/B,
    // 6 = mux inhibit, 7 = spare. 74HC595 pins: bit n -> Qn (Q0 = pin 15 ...).
    if (*pointer == 'G')
    {
        ++pointer;
        if (!s_manual_mode)
        {
            printf("ERR G: send M first to pause the control task\n");
            return true;
        }
        char * endptr = nullptr;
        long value = strtol(pointer, &endptr, 16);
        if (endptr == pointer || value < 0 || value > 0xFF)
        {
            printf("ERR G: expected hex byte 00-FF\n");
            return true;
        }
        esp_err_t err = s_shift_register.write(static_cast<uint8_t>(value));
        printf("ACK G: 595 = 0x%02X (0b%s) mux=ch%lu%s err=%s\n",
               (unsigned)value,
               std::bitset<8>(static_cast<uint8_t>(value)).to_string().c_str(),
               (unsigned long)((value >> 4) & 0x3),
               (value & 0x40) ? " inhibited" : " enabled",
               esp_err_to_name(err));
        return true;
    }

    printf("ERR: unknown command '%s'\n", serialBuffer);
    return false;
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
        NVSManager::start();

        // SPI for the 74HC595 direction and encoder selector outputs
        spi_bus_config_t spi_config = {};
        spi_config.mosi_io_num = (gpio_num_t)CONFIG_MOSI_GPIO;
        spi_config.miso_io_num = (gpio_num_t)CONFIG_MISO_GPIO;
        spi_config.sclk_io_num = (gpio_num_t)CONFIG_SCLK_GPIO;
        spi_config.quadwp_io_num = -1;
        spi_config.quadhd_io_num = -1;
        spi_bus_initialize(SPI2_HOST, &spi_config, SPI_DMA_CH_AUTO);
        SPIdev::init(SPI2_HOST);
        HC595::Config expander_config = {.latch_gpio = (gpio_num_t)CONFIG_SHIFT_REGISTER_LATCH_GPIO,
                                         .device_key = CONFIG_SHIFT_REGISTER_DEVICE_KEY,
                                         .clock_hz = 1000000};
        TEST_ASSERT_EQUAL(ESP_OK, s_shift_register.init(expander_config));
        CD4052B::Config mux_config = {.select_a_bit = CONFIG_SHIFT_REGISTER_MUX_A_BIT,
                                      .select_b_bit = CONFIG_SHIFT_REGISTER_MUX_B_BIT,
                                      .inhibit_bit = CONFIG_SHIFT_REGISTER_MUX_INH_BIT};
        TEST_ASSERT_EQUAL(ESP_OK, s_cd4052b.init(&s_shift_register, mux_config));

        // I2C for AS5600
        i2c_master_bus_config_t i2c_mst_config = {
          .i2c_port = (i2c_port_t)CONFIG_I2C_PORT_NUM,
          .sda_io_num = (gpio_num_t)CONFIG_SDA_GPIO,
          .scl_io_num = (gpio_num_t)CONFIG_SCL_GPIO,
          .clk_source = I2C_CLK_SRC_DEFAULT,
          .glitch_ignore_cnt = 7,
          .intr_priority = 0,
          .trans_queue_depth = 0,
          .flags = {.enable_internal_pullup = true, .allow_pd = false}};
        i2c_new_master_bus(&i2c_mst_config, &s_i2c_bus);
        I2Cdev::init(s_i2c_bus);

        // ADC / Estimator
        WheelStateEstimator::get_instance().init(
          config::test::WHEEL_ADC_CHANNELS, ADC_ATTEN_DB_12, &s_cd4052b);

        // Motor driver
        config::driver::MotorDriverConfig motor_cfg;
        motor_cfg.timer = LEDC_TIMER_0;
        motor_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
        motor_cfg.duty_resolution = config::test::DUTY_RESOLUTION;
        motor_cfg.pwm_freq = config::test::PWM_FREQ;
        motor_cfg.motor_pwm_pins = config::test::MOTOR_PWM_PINS;
        motor_cfg.motor_channels = config::test::MOTOR_CHANNELS;
        motor_cfg.direction_shift_register = &s_shift_register;
        motor_cfg.direction_bits = config::test::MOTOR_DIR_BITS;
        TEST_ASSERT_EQUAL(ESP_OK, BL48250::get_instance().configure(motor_cfg));

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
    printf("  E<channel>          - Probe the AS5600 on mux channel 0-3 (FL/BL/BR/FR)\n");
    printf("  E-1                 - Release the mux\n");
    printf("  I                   - Scan I2C bus for devices (AS5600 = 0x36)\n");
    printf("  M                   - Toggle manual mode (pause control task)\n");
    printf("  G<hex>              - Write raw 74HC595 byte (manual mode)\n");
    printf("=======================================================\n");

    while (true)
    {
        // Service serial commands
        read_serial_data();
        process_serial_command(wc);

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
