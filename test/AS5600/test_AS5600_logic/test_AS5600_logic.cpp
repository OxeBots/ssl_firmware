#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <unity.h>

#include "AS5600.h"
#include "NVSManager.h"

// Runs on the ESP32 with no encoder hardware attached. The analog path is
// exercised against a real ADC line-fitting calibration handle, which is a
// software conversion and needs no external sensor.
namespace as5600_test
{
constexpr adc_bitwidth_t BITWIDTH = ADC_BITWIDTH_12;
constexpr adc_atten_t ATTENUATION = ADC_ATTEN_DB_12;
constexpr float TWO_PI = 6.2831853f;
constexpr float EPSILON = 0.01f;
constexpr int CAL_MIN_MV = 500;
constexpr int CAL_MAX_MV = 2500;
// Production encoder calibration lives on channels 0, 3, 6 and 7; the NVS
// roundtrip must never touch those keys.
constexpr adc_channel_t NVS_TEST_CHANNEL = ADC_CHANNEL_4;
constexpr uint32_t NVS_WRITE_DELAY_MS = 250;
}  // namespace as5600_test

static adc_cali_handle_t make_voltage_calibration()
{
    adc_cali_line_fitting_config_t cali_config = {
      .unit_id = ADC_UNIT_1,
      .atten = as5600_test::ATTENUATION,
      .bitwidth = as5600_test::BITWIDTH,
      .default_vref = ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF};
    adc_cali_handle_t handle = nullptr;
    TEST_ASSERT_EQUAL(ESP_OK, adc_cali_create_scheme_line_fitting(&cali_config, &handle));
    return handle;
}

// The ADC stream owns the oversampling, so every process_new_reading
// call converts exactly one published raw block mean.
void test_uncalibrated_raw_value_maps_full_turn()
{
    AS5600 encoder(ADC_CHANNEL_0, nullptr, false);
    float angle = -1.0f;

    encoder.process_new_reading(0, &angle);
    TEST_ASSERT_FLOAT_WITHIN(as5600_test::EPSILON, 0.0f, angle);

    encoder.process_new_reading(2048, &angle);
    TEST_ASSERT_FLOAT_WITHIN(
      as5600_test::EPSILON, (2048.0f / 4095.0f) * as5600_test::TWO_PI, angle);

    encoder.process_new_reading(4095, &angle);
    TEST_ASSERT_FLOAT_WITHIN(as5600_test::EPSILON, as5600_test::TWO_PI, angle);
}

void test_calibration_range_validation()
{
    AS5600 encoder(ADC_CHANNEL_0, nullptr, false);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, encoder.set_calibration_range(2000, 1000));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, encoder.set_calibration_range(-5, 2500));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, encoder.set_calibration_range(0, 4000));

    TEST_ASSERT_EQUAL(
      ESP_OK, encoder.set_calibration_range(as5600_test::CAL_MIN_MV, as5600_test::CAL_MAX_MV));
    TEST_ASSERT_EQUAL(as5600_test::CAL_MIN_MV, encoder.get_calib_min());
    TEST_ASSERT_EQUAL(as5600_test::CAL_MAX_MV, encoder.get_calib_max());
}

void test_calibrated_voltage_clamps_to_calibration_range()
{
    adc_cali_handle_t cali = make_voltage_calibration();
    AS5600 encoder(ADC_CHANNEL_0, cali, true);
    TEST_ASSERT_EQUAL(ESP_OK, encoder.set_calibration_range(1000, 2000));
    float angle = -1.0f;

    // Outside the calibrated range on both ends: the ratio must clamp to the
    // exact interval limits instead of wrapping around.
    encoder.process_new_reading(0, &angle);
    TEST_ASSERT_FLOAT_WITHIN(as5600_test::EPSILON, 0.0f, angle);

    encoder.process_new_reading(4095, &angle);
    TEST_ASSERT_FLOAT_WITHIN(as5600_test::EPSILON, as5600_test::TWO_PI, angle);
}

void test_calibration_mode_tracks_voltage_extremes()
{
    adc_cali_handle_t cali = make_voltage_calibration();
    AS5600 encoder(ADC_CHANNEL_0, cali, true);
    float angle = -1.0f;

    encoder.reset_calibration_min_max();
    TEST_ASSERT_EQUAL(5000, encoder.get_calib_min());
    TEST_ASSERT_EQUAL(0, encoder.get_calib_max());

    encoder.start_calibration_mode();
    encoder.process_new_reading(4095, &angle);
    encoder.process_new_reading(0, &angle);
    encoder.stop_calibration_mode();

    int swing_mv = encoder.get_calib_max() - encoder.get_calib_min();
    TEST_ASSERT_TRUE_MESSAGE(swing_mv > 2500, "Calibration swing should cover most of 0-3.3 V");
}

void test_load_calibration_missing_entry()
{
    AS5600 encoder(ADC_CHANNEL_5, nullptr, false);
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, encoder.load_calibration_from_nvs());
}

void test_nvs_calibration_roundtrip()
{
    TEST_ASSERT_EQUAL(ESP_OK, NVSManager::init());
    static bool nvs_started = false;
    if (!nvs_started)
    {
        NVSManager::start();
        nvs_started = true;
    }

    AS5600 writer(as5600_test::NVS_TEST_CHANNEL, nullptr, false);
    TEST_ASSERT_EQUAL(
      ESP_OK, writer.set_calibration_range(as5600_test::CAL_MIN_MV, as5600_test::CAL_MAX_MV));
    TEST_ASSERT_EQUAL(ESP_OK, writer.save_calibration_to_nvs());

    // save_i32 is queued; give the sole NVS writer task time to persist both keys.
    vTaskDelay(pdMS_TO_TICKS(as5600_test::NVS_WRITE_DELAY_MS));

    AS5600 reader(as5600_test::NVS_TEST_CHANNEL, nullptr, false);
    TEST_ASSERT_EQUAL(ESP_OK, reader.load_calibration_from_nvs());
    TEST_ASSERT_EQUAL(as5600_test::CAL_MIN_MV, reader.get_calib_min());
    TEST_ASSERT_EQUAL(as5600_test::CAL_MAX_MV, reader.get_calib_max());
}

void test_i2c_state_machine_without_bus()
{
    AS5600 encoder(ADC_CHANNEL_5, nullptr, false);

    TEST_ASSERT_FALSE(encoder.is_i2c_initialized());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      encoder.set_output_stage(AS5600::OutputStage::ANALOG_FULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      encoder.set_slow_filter(AS5600::SlowFilter::FILTER_2X));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, encoder.read_configuration(nullptr, nullptr, nullptr));

    uint16_t angle = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, encoder.get_i2c_raw_angle(&angle));

    // No I2C master bus was ever registered with I2Cdev, so the probe fails.
    TEST_ASSERT_EQUAL(ESP_FAIL, encoder.init_i2c());
}

extern "C" void app_main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_uncalibrated_raw_value_maps_full_turn);
    RUN_TEST(test_calibration_range_validation);
    RUN_TEST(test_calibrated_voltage_clamps_to_calibration_range);
    RUN_TEST(test_calibration_mode_tracks_voltage_extremes);
    RUN_TEST(test_load_calibration_missing_entry);
    RUN_TEST(test_nvs_calibration_roundtrip);
    RUN_TEST(test_i2c_state_machine_without_bus);

    UNITY_END();
}
