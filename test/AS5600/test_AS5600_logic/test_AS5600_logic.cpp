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
constexpr adc_channel_t MAPPING_TEST_CHANNEL = ADC_CHANNEL_1;
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

// Creates the NVS writer task once; subsequent calls are no-ops.
static void ensure_nvs()
{
    static bool nvs_ready = false;
    if (nvs_ready)
        return;
    TEST_ASSERT_EQUAL(ESP_OK, NVSManager::init());
    NVSManager::start();
    nvs_ready = true;
}

// Feeds one raw block mean `count` times so the calibration median window
// fills up and updates the tracking endpoints.
static void feed_raw(AS5600 & encoder, uint16_t raw, int count)
{
    float angle = -1.0f;
    for (int i = 0; i < count; ++i) encoder.process_new_reading(raw, &angle);
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

    encoder.reset_calibration_min_max();
    TEST_ASSERT_EQUAL(5000, encoder.get_calib_min());
    TEST_ASSERT_EQUAL(0, encoder.get_calib_max());

    encoder.start_calibration_mode();
    feed_raw(encoder, 200, 5);
    feed_raw(encoder, 3900, 5);
    encoder.stop_calibration_mode();

    int swing_mv = encoder.get_track_max_mv() - encoder.get_track_min_mv();
    TEST_ASSERT_TRUE_MESSAGE(swing_mv > 2500, "Calibration swing should cover most of 0-3.3 V");

    // The tracking accumulators hold the endpoints; the applied range stays
    // untouched until a successful save promotes them.
    TEST_ASSERT_EQUAL_UINT16(200, encoder.get_track_min_raw());
    TEST_ASSERT_EQUAL_UINT16(3900, encoder.get_track_max_raw());
    TEST_ASSERT_EQUAL_UINT16(65535, encoder.get_calib_min_raw());
    TEST_ASSERT_EQUAL_UINT16(0, encoder.get_calib_max_raw());
}

void test_rail_samples_do_not_corrupt_range()
{
    adc_cali_handle_t cali = make_voltage_calibration();
    AS5600 encoder(ADC_CHANNEL_0, cali, true);
    float angle = -1.0f;

    encoder.reset_calibration_min_max();
    encoder.start_calibration_mode();
    // Rail/glitch block means must be ignored for both endpoints: an
    // intermittent analog input reading 0 or full scale is not physical for
    // the REDUCED output band.
    encoder.process_new_reading(0, &angle);
    encoder.process_new_reading(4095, &angle);
    feed_raw(encoder, 1200, 5);
    // A single sample just above the rail filter must be rejected by the
    // median window instead of pinning the minimum.
    encoder.process_new_reading(150, &angle);
    feed_raw(encoder, 2800, 5);
    encoder.stop_calibration_mode();

    TEST_ASSERT_EQUAL_UINT16(1200, encoder.get_track_min_raw());
    TEST_ASSERT_EQUAL_UINT16(2800, encoder.get_track_max_raw());

    int expected_min_mv = 0;
    int expected_max_mv = 0;
    TEST_ASSERT_EQUAL(ESP_OK, adc_cali_raw_to_voltage(cali, 1200, &expected_min_mv));
    TEST_ASSERT_EQUAL(ESP_OK, adc_cali_raw_to_voltage(cali, 2800, &expected_max_mv));
    TEST_ASSERT_EQUAL(expected_min_mv, encoder.get_track_min_mv());
    TEST_ASSERT_EQUAL(expected_max_mv, encoder.get_track_max_mv());
    TEST_ASSERT_EQUAL_UINT16(65535, encoder.get_calib_min_raw());
}

void test_implausible_range_is_rejected()
{
    AS5600 encoder(ADC_CHANNEL_2, nullptr, false);
    float angle = -1.0f;

    // Only rail samples: no plausible endpoints are tracked at all, so nothing
    // can be saved and the applied calibration stays untouched.
    encoder.reset_calibration_min_max();
    encoder.start_calibration_mode();
    encoder.process_new_reading(0, &angle);
    encoder.process_new_reading(4095, &angle);
    encoder.stop_calibration_mode();
    TEST_ASSERT_EQUAL_UINT16(65535, encoder.get_track_min_raw());
    TEST_ASSERT_EQUAL_UINT16(0, encoder.get_track_max_raw());
    TEST_ASSERT_EQUAL(ESP_FAIL, encoder.save_calibration_to_nvs());
    TEST_ASSERT_EQUAL_UINT16(65535, encoder.get_calib_min_raw());
    TEST_ASSERT_EQUAL_UINT16(0, encoder.get_calib_max_raw());

    // A partial capture (span below MIN_VALID_RAW_RANGE) must be rejected
    // instead of saved: the angle mapping needs the full-turn span.
    encoder.reset_calibration_min_max();
    encoder.start_calibration_mode();
    feed_raw(encoder, 1500, 5);
    feed_raw(encoder, 2800, 5);
    encoder.stop_calibration_mode();
    TEST_ASSERT_EQUAL_UINT16(1500, encoder.get_track_min_raw());
    TEST_ASSERT_EQUAL_UINT16(2800, encoder.get_track_max_raw());
    TEST_ASSERT_EQUAL(ESP_FAIL, encoder.save_calibration_to_nvs());
    TEST_ASSERT_EQUAL_UINT16(65535, encoder.get_calib_min_raw());

    // Nothing was persisted for this channel.
    AS5600 reader(ADC_CHANNEL_2, nullptr, false);
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, reader.load_calibration_from_nvs());
}

void test_calibrated_raw_range_maps_ends()
{
    ensure_nvs();

    // A successful save promotes the tracked range to the applied range.
    AS5600 writer(as5600_test::MAPPING_TEST_CHANNEL, make_voltage_calibration(), true);
    writer.reset_calibration_min_max();
    writer.start_calibration_mode();
    feed_raw(writer, 300, 5);
    feed_raw(writer, 3500, 5);
    writer.stop_calibration_mode();
    TEST_ASSERT_EQUAL_UINT16(300, writer.get_track_min_raw());
    TEST_ASSERT_EQUAL_UINT16(3500, writer.get_track_max_raw());
    TEST_ASSERT_EQUAL(ESP_OK, writer.save_calibration_to_nvs());
    TEST_ASSERT_EQUAL_UINT16(300, writer.get_calib_min_raw());
    TEST_ASSERT_EQUAL_UINT16(3500, writer.get_calib_max_raw());

    vTaskDelay(pdMS_TO_TICKS(as5600_test::NVS_WRITE_DELAY_MS));

    AS5600 encoder(as5600_test::MAPPING_TEST_CHANNEL, nullptr, false);
    TEST_ASSERT_EQUAL(ESP_OK, encoder.load_calibration_from_nvs());
    TEST_ASSERT_EQUAL_UINT16(300, encoder.get_calib_min_raw());
    TEST_ASSERT_EQUAL_UINT16(3500, encoder.get_calib_max_raw());

    // The raw range takes priority over the millivolt range: the stored
    // endpoints map to exactly 0 and 2*pi, and values outside the range clamp
    // to the endpoints instead of wrapping.
    float angle = -1.0f;
    encoder.process_new_reading(300, &angle);
    TEST_ASSERT_FLOAT_WITHIN(as5600_test::EPSILON, 0.0f, angle);
    encoder.process_new_reading(3500, &angle);
    TEST_ASSERT_FLOAT_WITHIN(as5600_test::EPSILON, as5600_test::TWO_PI, angle);
    encoder.process_new_reading(100, &angle);
    TEST_ASSERT_FLOAT_WITHIN(as5600_test::EPSILON, 0.0f, angle);
    encoder.process_new_reading(3700, &angle);
    TEST_ASSERT_FLOAT_WITHIN(as5600_test::EPSILON, as5600_test::TWO_PI, angle);
    encoder.process_new_reading(1900, &angle);
    TEST_ASSERT_FLOAT_WITHIN(as5600_test::EPSILON, 0.5f * as5600_test::TWO_PI, angle);

    // Starting an aborted/failed run only resets the tracking accumulators: the
    // applied range keeps driving the mapping.
    encoder.reset_calibration_min_max();
    encoder.start_calibration_mode();
    encoder.process_new_reading(300, &angle);
    encoder.stop_calibration_mode();
    TEST_ASSERT_EQUAL_UINT16(300, encoder.get_calib_min_raw());
    TEST_ASSERT_EQUAL_UINT16(3500, encoder.get_calib_max_raw());
    encoder.process_new_reading(1900, &angle);
    TEST_ASSERT_FLOAT_WITHIN(as5600_test::EPSILON, 0.5f * as5600_test::TWO_PI, angle);
}

void test_load_rejects_implausible_raw_range()
{
    ensure_nvs();

    // Simulate a range stored by an earlier run with a railed endpoint (the M2
    // incident): a 0-count minimum with an otherwise plausible span.
    TEST_ASSERT_EQUAL(ESP_OK, NVSManager::save_i32("wheel_calib", "ch1_min", 500));
    TEST_ASSERT_EQUAL(ESP_OK, NVSManager::save_i32("wheel_calib", "ch1_max", 2500));
    TEST_ASSERT_EQUAL(ESP_OK, NVSManager::save_i32("wheel_calib", "ch1_rmin", 0));
    TEST_ASSERT_EQUAL(ESP_OK, NVSManager::save_i32("wheel_calib", "ch1_rmax", 3565));
    vTaskDelay(pdMS_TO_TICKS(as5600_test::NVS_WRITE_DELAY_MS));

    adc_cali_handle_t cali = make_voltage_calibration();
    AS5600 encoder(as5600_test::MAPPING_TEST_CHANNEL, cali, true);
    TEST_ASSERT_EQUAL(ESP_OK, encoder.load_calibration_from_nvs());

    // The implausible raw pair is rejected on load: the applied raw range stays
    // invalid and the millivolt range drives the mapping instead.
    TEST_ASSERT_EQUAL_UINT16(65535, encoder.get_calib_min_raw());
    TEST_ASSERT_EQUAL_UINT16(0, encoder.get_calib_max_raw());
    TEST_ASSERT_EQUAL(500, encoder.get_calib_min());
    TEST_ASSERT_EQUAL(2500, encoder.get_calib_max());

    float angle = -1.0f;
    encoder.process_new_reading(0, &angle);
    TEST_ASSERT_FLOAT_WITHIN(as5600_test::EPSILON, 0.0f, angle);
    encoder.process_new_reading(4095, &angle);
    TEST_ASSERT_FLOAT_WITHIN(as5600_test::EPSILON, as5600_test::TWO_PI, angle);
}

void test_load_calibration_missing_entry()
{
    AS5600 encoder(ADC_CHANNEL_5, nullptr, false);
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, encoder.load_calibration_from_nvs());
}

void test_nvs_calibration_roundtrip()
{
    ensure_nvs();

    adc_cali_handle_t cali = make_voltage_calibration();
    AS5600 writer(as5600_test::NVS_TEST_CHANNEL, cali, true);
    // Track a raw endpoint pair so both kinds of range are persisted.
    writer.reset_calibration_min_max();
    writer.start_calibration_mode();
    feed_raw(writer, 300, 5);
    feed_raw(writer, 3500, 5);
    writer.stop_calibration_mode();
    TEST_ASSERT_EQUAL_UINT16(300, writer.get_track_min_raw());
    TEST_ASSERT_EQUAL_UINT16(3500, writer.get_track_max_raw());

    int expected_min_mv = 0;
    int expected_max_mv = 0;
    TEST_ASSERT_EQUAL(ESP_OK, adc_cali_raw_to_voltage(cali, 300, &expected_min_mv));
    TEST_ASSERT_EQUAL(ESP_OK, adc_cali_raw_to_voltage(cali, 3500, &expected_max_mv));
    TEST_ASSERT_EQUAL(expected_min_mv, writer.get_track_min_mv());
    TEST_ASSERT_EQUAL(expected_max_mv, writer.get_track_max_mv());

    TEST_ASSERT_EQUAL(ESP_OK, writer.save_calibration_to_nvs());
    // A successful save promotes the tracked range to the applied range.
    TEST_ASSERT_EQUAL(expected_min_mv, writer.get_calib_min());
    TEST_ASSERT_EQUAL(expected_max_mv, writer.get_calib_max());
    TEST_ASSERT_EQUAL_UINT16(300, writer.get_calib_min_raw());
    TEST_ASSERT_EQUAL_UINT16(3500, writer.get_calib_max_raw());

    // save_i32 is queued; give the sole NVS writer task time to persist the keys.
    vTaskDelay(pdMS_TO_TICKS(as5600_test::NVS_WRITE_DELAY_MS));

    AS5600 reader(as5600_test::NVS_TEST_CHANNEL, nullptr, false);
    TEST_ASSERT_EQUAL(ESP_OK, reader.load_calibration_from_nvs());
    TEST_ASSERT_EQUAL(expected_min_mv, reader.get_calib_min());
    TEST_ASSERT_EQUAL(expected_max_mv, reader.get_calib_max());
    TEST_ASSERT_EQUAL_UINT16(300, reader.get_calib_min_raw());
    TEST_ASSERT_EQUAL_UINT16(3500, reader.get_calib_max_raw());
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

    // Stateless reads bypass the instance state but still need the bus.
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, AS5600::read_output_stage(nullptr));
    AS5600::OutputStage stage = AS5600::OutputStage::ANALOG_FULL;
    TEST_ASSERT_EQUAL(ESP_FAIL, AS5600::read_output_stage(&stage));

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
    RUN_TEST(test_rail_samples_do_not_corrupt_range);
    RUN_TEST(test_implausible_range_is_rejected);
    RUN_TEST(test_calibrated_raw_range_maps_ends);
    RUN_TEST(test_load_rejects_implausible_raw_range);
    RUN_TEST(test_load_calibration_missing_entry);
    RUN_TEST(test_nvs_calibration_roundtrip);
    RUN_TEST(test_i2c_state_machine_without_bus);

    UNITY_END();
}
