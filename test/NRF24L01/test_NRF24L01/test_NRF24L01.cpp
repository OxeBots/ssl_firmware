/**
 * @file test_NRF24L01.cpp
 * @brief Hardware diagnostic tests for the NRF24L01 module on ESP32.
 *
 * Tests are ordered from most fundamental to most specific so that an early
 * failure immediately points to the root cause (GPIO, SPI bus, or RF module).
 *
 * Test execution order:
 *   1. test_gpio_irq_pin         — IRQ input pin is configurable and reads a stable level
 *   2. test_gpio_ce_csn_pins     — CE / CSN can be driven as digital outputs
 *   3. test_spi_status_plausible — STATUS byte is not 0x00 or 0xFF after Nrf24_init()
 *   4. test_status_reset_value   — STATUS[3:0] matches the nRF24L01(+) power-on reset value
 *   5. test_config_register_rw   — CONFIG register round-trip write / read-back
 *   6. test_rf_channel_rw        — RF_CH register two-value round-trip write / read-back
 *   7. test_set_rx_tx_address    — Nrf24_setRADDR + Nrf24_setTADDR succeed with address verify
 *
 * Pin mapping (from sdkconfig.defaults):
 *   MISO  → GPIO 19   (CONFIG_MISO_GPIO)
 *   MOSI  → GPIO 23   (CONFIG_MOSI_GPIO)
 *   SCLK  → GPIO 18   (CONFIG_SCLK_GPIO)
 *   CE    → GPIO  2   (CONFIG_CE_GPIO)
 *   CSN   → GPIO  5   (CONFIG_CSN_GPIO)
 *   IRQ   → GPIO  4   (CONFIG_IRQ_GPIO)   ← active-LOW, pulled up internally
 *
 * Hardware tips:
 *   - Power the module from 3.3 V only — never 5 V.
 *   - Place a 10 µF electrolytic + 100 nF ceramic capacitor between VCC and GND
 *     as close to the module as possible.  Missing decoupling is the #1 cause of
 *     Nrf24_setRADDR failures during multi-byte SPI burst writes.
 */

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <unity.h>

#include "mirf.h"  // NRF24_t, Nrf24_*, Nrf24_readRegister, Nrf24_configRegister, etc.

// ============================================================================
// Pin definitions — must match sdkconfig.defaults
// (These echo the CONFIG_* macros that mirf.c reads at compile time.)
// ============================================================================

#define PIN_MISO CONFIG_MISO_GPIO  // 19
#define PIN_MOSI CONFIG_MOSI_GPIO  // 23
#define PIN_SCLK CONFIG_SCLK_GPIO  // 18
#define PIN_CE CONFIG_CE_GPIO      //  2
#define PIN_CSN CONFIG_CSN_GPIO    //  5
#define PIN_IRQ CONFIG_IRQ_GPIO    //  4

// RF settings used in main.cpp — kept identical for a realistic test
#define RF_CHANNEL CONFIG_RADIO_CHANNEL  // 76
#define PAYLOAD_SIZE 32

// The same addresses used in main.cpp.
// NOTE: intentionally NOT named RX_ADDR / TX_ADDR — those names are already
// taken by mirf.h register macros (0x0A and 0x10) and would cause type errors.
static const char * PIPE_RX_ADDR = "ESP32";  // → Nrf24_setRADDR → RX_ADDR_P1
static const char * PIPE_TX_ADDR = "ADMIN";  // → Nrf24_setTADDR → RX_ADDR_P0 + TX_ADDR

static const char * TAG = "NRF24L01_TEST";

// ============================================================================
// Module-level device handle (shared across all tests in one run)
// ============================================================================

static NRF24_t s_dev;

// ============================================================================
// Internal helpers — use only real mirf API functions
// ============================================================================

/**
 * @brief Read one register byte via Nrf24_readRegister().
 */
static uint8_t read_reg_byte(uint8_t reg)
{
    uint8_t value = 0;
    Nrf24_readRegister(&s_dev, reg, &value, 1);
    return value;
}

/**
 * @brief Write one register byte with Nrf24_configRegister(), then read back.
 * @return The value read back from the register.
 */
static uint8_t write_and_readback(uint8_t reg, uint8_t value)
{
    Nrf24_configRegister(&s_dev, reg, value);
    vTaskDelay(pdMS_TO_TICKS(2));
    return read_reg_byte(reg);
}

// ============================================================================
// SETUP — called once in app_main before the SPI-dependent UNITY_BEGIN()
// ============================================================================

/**
 * @brief Initialize the SPI bus and attach the NRF24L01 device.
 *
 * Delegates entirely to Nrf24_init() which internally calls
 * spi_bus_initialize() + spi_bus_add_device() and configures CE / CSN GPIOs.
 * If the SPI bus cannot be initialised, Nrf24_init() will assert() — that
 * crash itself is a diagnostic result (SPI pin conflict or GPIO already in use).
 */
static void setup_spi()
{
    memset(&s_dev, 0, sizeof(NRF24_t));
    Nrf24_init(&s_dev);
    vTaskDelay(pdMS_TO_TICKS(20));  // let the module power up after CE/CSN settle
}

// ============================================================================
// TEST 1 — IRQ pin
//
// The IRQ pin (GPIO 4) is active-LOW and driven by the NRF24L01 module.
// Configure it as an input with the internal pull-up enabled.
// On an idle, powered module the line should read HIGH (not asserted).
// A stuck-LOW result means either the pin is shorted to GND or the module
// is asserting IRQ continuously (STATUS interrupt flags not cleared).
// ============================================================================

void test_gpio_irq_pin(void)
{
    ESP_LOGI(TAG, "--- Test 1: IRQ pin (GPIO %d) ---", PIN_IRQ);

    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << PIN_IRQ);
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&cfg);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err,
                              "gpio_config failed for IRQ pin. Check CONFIG_IRQ_GPIO value (should be 4).");

    vTaskDelay(pdMS_TO_TICKS(5));  // let pull-up settle

    int level = gpio_get_level((gpio_num_t)PIN_IRQ);
    ESP_LOGI(TAG, "  IRQ GPIO %d level = %d  (expected 1 = idle / not asserted)", PIN_IRQ, level);

    TEST_ASSERT_EQUAL_MESSAGE(1, level,
                              "IRQ pin reads LOW on idle module. "
                              "Possible causes: (a) pin shorted to GND, "
                              "(b) module asserting IRQ due to uncleared STATUS flags — power-cycle and re-run, "
                              "(c) CONFIG_IRQ_GPIO wrong — check sdkconfig.defaults.");
}

// ============================================================================
// TEST 2 — CE and CSN GPIO output drive
//
// Verify the pin numbers are valid and driveable as outputs before
// Nrf24_init() runs. Invalid pin numbers cause gpio_config() to return
// ESP_ERR_INVALID_ARG.
// ============================================================================

void test_gpio_ce_csn_pins(void)
{
    ESP_LOGI(TAG, "--- Test 2: CE (GPIO %d) / CSN (GPIO %d) output drive ---", PIN_CE, PIN_CSN);

    const gpio_num_t pins[] = {(gpio_num_t)PIN_CE, (gpio_num_t)PIN_CSN};
    const char * labels[] = {"CE", "CSN"};

    for (int i = 0; i < 2; ++i)
    {
        gpio_config_t cfg = {};
        cfg.pin_bit_mask = (1ULL << pins[i]);
        cfg.mode = GPIO_MODE_OUTPUT;
        cfg.pull_up_en = GPIO_PULLUP_DISABLE;
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        cfg.intr_type = GPIO_INTR_DISABLE;

        esp_err_t err = gpio_config(&cfg);
        char msg[80];
        snprintf(msg, sizeof(msg), "%s gpio_config() failed for GPIO %d — invalid pin or already in use?", labels[i],
                 (int)pins[i]);
        TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err, msg);

        // Toggle HIGH → LOW → HIGH; a driver error would have fired above.
        gpio_set_level(pins[i], 1);
        vTaskDelay(pdMS_TO_TICKS(5));
        gpio_set_level(pins[i], 0);
        vTaskDelay(pdMS_TO_TICKS(5));
        gpio_set_level(pins[i], 1);

        ESP_LOGI(TAG, "  %s GPIO %d toggled HIGH → LOW → HIGH  OK", labels[i], (int)pins[i]);
    }
}

// ============================================================================
// TEST 3 — SPI bus plausibility (STATUS byte after Nrf24_init)
//
// After Nrf24_init() the STATUS register is read. Any value other than 0xFF
// (MISO floating) or 0x00 (MOSI/SCLK shorted to GND) means the SPI bus and
// CSN are working at a basic level.
// ============================================================================

void test_spi_status_plausible(void)
{
    ESP_LOGI(TAG, "--- Test 3: SPI bus plausibility (STATUS register) ---");
    ESP_LOGI(TAG, "  MISO=%-2d  MOSI=%-2d  SCLK=%-2d  CE=%-2d  CSN=%-2d", PIN_MISO, PIN_MOSI, PIN_SCLK, PIN_CE,
             PIN_CSN);

    uint8_t status = Nrf24_getStatus(&s_dev);
    ESP_LOGI(TAG, "  STATUS = 0x%02X", status);

    TEST_ASSERT_NOT_EQUAL_MESSAGE(0xFF, status,
                                  "STATUS=0xFF — MISO is floating or CSN is never asserted. "
                                  "Check MISO wiring (GPIO 19), CSN wiring (GPIO 5), and module VCC (3.3V).");

    TEST_ASSERT_NOT_EQUAL_MESSAGE(0x00, status,
                                  "STATUS=0x00 — MOSI or SCLK appears shorted to GND, or the module is not powered. "
                                  "Check MOSI (GPIO 23), SCLK (GPIO 18), VCC (3.3V only!), and GND.");
}

// ============================================================================
// TEST 4 — STATUS register reset value
//
// On a freshly powered nRF24L01(+) the STATUS register reset value is 0x0E:
//   RX_DR=0  TX_DS=0  MAX_RT=0  RX_P_NO=111b  TX_FULL=0
// We mask off the upper interrupt bits (which may be set if the module was
// active before this test) and only verify the stable lower nibble.
//
// If this test fails but test 3 passes: a power cycle usually resets STATUS.
// ============================================================================

void test_status_reset_value(void)
{
    ESP_LOGI(TAG, "--- Test 4: STATUS register reset value ---");

    uint8_t status = Nrf24_getStatus(&s_dev);
    // RX_P_NO[2:0]=111b and TX_FULL=0 → lower nibble = 0x0E
    uint8_t lower_nibble = status & 0x0F;

    ESP_LOGI(TAG, "  STATUS=0x%02X  [3:0]=0x%02X  (expected 0x0E on clean power-up)", status, lower_nibble);

    TEST_ASSERT_EQUAL_HEX8_MESSAGE(0x0E, lower_nibble,
                                   "STATUS[3:0] != 0x0E — module may not be in power-on state. "
                                   "Power-cycle the module and re-run. "
                                   "If still failing and STATUS != 0xFF/0x00, the module is responding but in an "
                                   "unexpected mode (leftover config from a previous firmware run).");
}

// ============================================================================
// TEST 5 — CONFIG register write / read-back
//
// Exercises a full single-byte SPI write + read-back.
// CONFIG (0x00) is safe to modify; we write 0x0A (EN_CRC | PWR_UP, TX mode)
// and verify the read-back matches exactly.
//
// Failure here with tests 3–4 passing means write transactions are broken
// while read transactions work — check MOSI line integrity (GPIO 23).
// ============================================================================

void test_config_register_rw(void)
{
    ESP_LOGI(TAG, "--- Test 5: CONFIG register write / read-back ---");

    // 0x0A = EN_CRC(bit3)=1 | PWR_UP(bit1)=1  — a valid CONFIG value
    const uint8_t WRITE_VALUE = 0x0A;

    uint8_t readback = write_and_readback(CONFIG, WRITE_VALUE);
    ESP_LOGI(TAG, "  CONFIG written=0x%02X  read=0x%02X", WRITE_VALUE, readback);

    TEST_ASSERT_EQUAL_HEX8_MESSAGE(WRITE_VALUE, readback,
                                   "CONFIG register read-back mismatch. "
                                   "If read=0xFF: MISO floating (GPIO 19). "
                                   "If read=0x00 or wrong value: MOSI (GPIO 23) issue, or module not responding.");

    // Restore to RX mode for the address tests
    Nrf24_configRegister(&s_dev, CONFIG, mirf_CONFIG | (1 << PWR_UP) | (1 << PRIM_RX));
}

// ============================================================================
// TEST 6 — RF_CH register write / read-back (two distinct values)
//
// RF_CH (0x05) stores the 7-bit RF channel (0–125). Writing two clearly
// different values and reading each back rules out a stuck data line.
// The channel is restored to the production value (76) at the end.
// ============================================================================

void test_rf_channel_rw(void)
{
    ESP_LOGI(TAG, "--- Test 6: RF_CH register write / read-back ---");

    const uint8_t CH_A = 40;
    const uint8_t CH_B = RF_CHANNEL;  // 76 — restore production channel

    uint8_t rb_a = write_and_readback(RF_CH, CH_A);
    ESP_LOGI(TAG, "  RF_CH written=%3d  read=%3d", CH_A, rb_a);
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(CH_A, rb_a,
                                   "RF_CH first value read-back failed. "
                                   "Check MOSI (GPIO 23) and MISO (GPIO 19) are not swapped.");

    uint8_t rb_b = write_and_readback(RF_CH, CH_B);
    ESP_LOGI(TAG, "  RF_CH written=%3d  read=%3d", CH_B, rb_b);
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(CH_B, rb_b, "RF_CH second value read-back failed.");
}

// ============================================================================
// TEST 7 — Nrf24_setRADDR and Nrf24_setTADDR
//
// This is the exact call that fails in the reported error log:
//   E (466) NRF24L01: nrf24l01 not installed / failed to set RADDR
//
// mirf internals (mirf.c):
//   Nrf24_setRADDR()  → writes 5 bytes to RX_ADDR_P1, reads back to verify
//   Nrf24_setTADDR()  → writes 5 bytes to RX_ADDR_P0 and TX_ADDR, reads back P0
//
// Both functions return ESP_FAIL on any byte mismatch in their internal
// read-back.  We additionally log the raw read-back for byte-level visibility.
//
// If tests 1–6 pass but this one fails:
//   The single-byte SPI path works but the 5-byte burst write is corrupted.
//   Most common cause: VCC brown-out during the longer SPI transaction.
//   Fix → add a 10 µF electrolytic capacitor on the module's VCC/GND pins.
// ============================================================================

void test_set_rx_tx_address(void)
{
    ESP_LOGI(TAG, "--- Test 7: Nrf24_setRADDR(\"%s\") / Nrf24_setTADDR(\"%s\") ---", PIPE_RX_ADDR, PIPE_TX_ADDR);

    // ---- RX address — written to RX_ADDR_P1 by mirf ----
    esp_err_t err_rx = Nrf24_setRADDR(&s_dev, (uint8_t *)PIPE_RX_ADDR);

    uint8_t rx_rb[5] = {0};
    Nrf24_readRegister(&s_dev, RX_ADDR_P1, rx_rb, 5);
    ESP_LOGI(TAG, "  setRADDR → %s", (err_rx == ESP_OK) ? "OK" : "FAIL");
    ESP_LOGI(TAG, "  RX_ADDR_P1 readback: [%c%c%c%c%c]  hex: %02X %02X %02X %02X %02X", rx_rb[0], rx_rb[1], rx_rb[2],
             rx_rb[3], rx_rb[4], rx_rb[0], rx_rb[1], rx_rb[2], rx_rb[3], rx_rb[4]);

    TEST_ASSERT_EQUAL_MESSAGE(
      ESP_OK, err_rx,
      "Nrf24_setRADDR failed — 5-byte burst write to RX_ADDR_P1 read back with errors. "
      "MOST LIKELY FIX: add a 10uF electrolytic capacitor between VCC and GND on the NRF module. "
      "Also check: MOSI (GPIO 23) and MISO (GPIO 19) are not swapped, "
      "and that no other SPI device shares the bus without proper CSN isolation.");

    // ---- TX address — written to RX_ADDR_P0 + TX_ADDR register by mirf ----
    esp_err_t err_tx = Nrf24_setTADDR(&s_dev, (uint8_t *)PIPE_TX_ADDR);

    uint8_t p0_rb[5] = {0};
    uint8_t tx_rb[5] = {0};
    Nrf24_readRegister(&s_dev, RX_ADDR_P0, p0_rb, 5);
    Nrf24_readRegister(&s_dev, TX_ADDR, tx_rb, 5);  // TX_ADDR = 0x10 from mirf.h

    ESP_LOGI(TAG, "  setTADDR  → %s", (err_tx == ESP_OK) ? "OK" : "FAIL");
    ESP_LOGI(TAG, "  RX_ADDR_P0 readback: [%c%c%c%c%c]  hex: %02X %02X %02X %02X %02X", p0_rb[0], p0_rb[1], p0_rb[2],
             p0_rb[3], p0_rb[4], p0_rb[0], p0_rb[1], p0_rb[2], p0_rb[3], p0_rb[4]);
    ESP_LOGI(TAG, "  TX_ADDR    readback: [%c%c%c%c%c]  hex: %02X %02X %02X %02X %02X", tx_rb[0], tx_rb[1], tx_rb[2],
             tx_rb[3], tx_rb[4], tx_rb[0], tx_rb[1], tx_rb[2], tx_rb[3], tx_rb[4]);

    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err_tx,
                              "Nrf24_setTADDR failed — same root cause as RADDR. "
                              "Add a 10uF decoupling capacitor on the module VCC pin.");
}

// ============================================================================
// MAIN
// ============================================================================

extern "C" void app_main(void)
{
    // Allow power rails (especially NRF VCC) to fully stabilise after boot.
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "=================================================================");
    ESP_LOGI(TAG, " NRF24L01 Hardware Diagnostic");
    ESP_LOGI(TAG, " MISO=%-2d  MOSI=%-2d  SCLK=%-2d  CE=%-2d  CSN=%-2d  IRQ=%-2d", PIN_MISO, PIN_MOSI, PIN_SCLK, PIN_CE,
             PIN_CSN, PIN_IRQ);
    ESP_LOGI(TAG, "=================================================================");

    // --- Phase 1: raw GPIO tests (before Nrf24_init touches the pins) ---
    UNITY_BEGIN();
    RUN_TEST(test_gpio_irq_pin);
    RUN_TEST(test_gpio_ce_csn_pins);
    UNITY_END();

    // --- SPI init ---
    // Nrf24_init() asserts on SPI failure, so a crash here is diagnostic too.
    // The log will show "spi_bus_initialize=<err>" and "spi_bus_add_device=<err>"
    // before the assert fires.
    ESP_LOGI(TAG, "Calling Nrf24_init()...");
    setup_spi();
    ESP_LOGI(TAG, "Nrf24_init() OK — proceeding with register tests.");

    // --- Phase 2: SPI / register tests ---
    UNITY_BEGIN();
    RUN_TEST(test_spi_status_plausible);
    RUN_TEST(test_status_reset_value);
    RUN_TEST(test_config_register_rw);
    RUN_TEST(test_rf_channel_rw);
    RUN_TEST(test_set_rx_tx_address);
    UNITY_END();

    // Full register dump — printed regardless of test results for extra context.
    ESP_LOGI(TAG, "--- Full register dump (Nrf24_printDetails) ---");
    Nrf24_printDetails(&s_dev);
}
