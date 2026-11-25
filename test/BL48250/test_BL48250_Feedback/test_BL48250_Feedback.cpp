#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <sdkconfig.h>
#include <unity.h>

#include <array>
#include <cinttypes>
#include <cmath>
#include <cstdint>

#include "../shared_test_bus.h"
#include "../shared_test_spi.h"
#include "74HC595.h"
#include "BL48250.h"
#include "wheel_state_estimator.h"

static const char * TAG = "BL48250_feedback";

// Test configuration
namespace config
{
namespace test
{
constexpr std::array<gpio_num_t, 4> MOTOR_PWM_PINS = {
  static_cast<gpio_num_t>(CONFIG_MOTOR_FL_PWM_GPIO),
  static_cast<gpio_num_t>(CONFIG_MOTOR_BL_PWM_GPIO),
  static_cast<gpio_num_t>(CONFIG_MOTOR_BR_PWM_GPIO),
  static_cast<gpio_num_t>(CONFIG_MOTOR_FR_PWM_GPIO)};

constexpr std::array<uint8_t, 4> MOTOR_DIR_BITS = {CONFIG_SHIFT_REGISTER_DIR1_BIT,
                                                   CONFIG_SHIFT_REGISTER_DIR2_BIT,
                                                   CONFIG_SHIFT_REGISTER_DIR3_BIT,
                                                   CONFIG_SHIFT_REGISTER_DIR4_BIT};

constexpr std::array<gpio_num_t, 4> MOTOR_FB_PINS = {
  static_cast<gpio_num_t>(CONFIG_MOTOR_FL_FB_GPIO),
  static_cast<gpio_num_t>(CONFIG_MOTOR_BL_FB_GPIO),
  static_cast<gpio_num_t>(CONFIG_MOTOR_BR_FB_GPIO),
  static_cast<gpio_num_t>(CONFIG_MOTOR_FR_FB_GPIO)};

constexpr std::array<ledc_channel_t, 4> MOTOR_CHANNELS = {
  LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3, LEDC_CHANNEL_4};

constexpr ledc_timer_bit_t DUTY_RESOLUTION = LEDC_TIMER_10_BIT;
constexpr uint32_t PWM_FREQ = 5000;
constexpr uint32_t MAX_DUTY = (1 << static_cast<int>(DUTY_RESOLUTION)) - 1;
constexpr uint32_t TEST_DUTY = MAX_DUTY * 0.2;  // fixed PWM for every scenario

// Minimum wheel-shaft speed a free-spinning motor at the fixed duty must show.
constexpr float SPIN_VELOCITY_RAD_S = 0.3f;
// Minimum feedback counts a free-spinning motor must accumulate in 500 ms.
constexpr int32_t MIN_SPIN_COUNTS = 20;

const std::array<adc_channel_t, 4> WHEEL_ADC_CHANNELS = {
  static_cast<adc_channel_t>(CONFIG_MOTOR_FL_ENC_CHANNEL),
  static_cast<adc_channel_t>(CONFIG_MOTOR_BL_ENC_CHANNEL),
  static_cast<adc_channel_t>(CONFIG_MOTOR_BR_ENC_CHANNEL),
  static_cast<adc_channel_t>(CONFIG_MOTOR_FR_ENC_CHANNEL)};
}  // namespace test
}  // namespace config

#define DELAY_MS(ms) vTaskDelay((ms) / portTICK_PERIOD_MS)

static void configure_driver(bool with_feedback_pins)
{
    shared_test_spi_bus();

    HC595::Config expander_config = {.latch_gpio = (gpio_num_t)CONFIG_SHIFT_REGISTER_LATCH_GPIO,
                                     .device_key = CONFIG_SHIFT_REGISTER_DEVICE_KEY,
                                     .clock_hz = 1000000};
    static HC595 expander;
    static bool expander_initialized = false;
    if (!expander_initialized)
    {
        TEST_ASSERT_EQUAL(ESP_OK, expander.init(expander_config));
        expander_initialized = true;
    }

    config::driver::MotorDriverConfig motor_cfg;
    motor_cfg.timer = LEDC_TIMER_0;
    motor_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    motor_cfg.duty_resolution = config::test::DUTY_RESOLUTION;
    motor_cfg.pwm_freq = config::test::PWM_FREQ;
    motor_cfg.motor_pwm_pins = config::test::MOTOR_PWM_PINS;
    motor_cfg.motor_channels = config::test::MOTOR_CHANNELS;
    motor_cfg.direction_shift_register = &expander;
    motor_cfg.direction_bits = config::test::MOTOR_DIR_BITS;
    if (with_feedback_pins)
        motor_cfg.motor_fb_pins = config::test::MOTOR_FB_PINS;

    // The driver is a singleton and earlier suites may have configured it
    // without feedback pins, so force a fresh configuration.
    BL48250::get_instance().deinit();
    TEST_ASSERT_EQUAL(ESP_OK, BL48250::get_instance().configure(motor_cfg));
}

static void spin_motor(size_t motor_idx, uint32_t duty, uint8_t direction)
{
    std::array<uint32_t, 4> duties = {0, 0, 0, 0};
    std::array<uint8_t, 4> dirs = {config::driver::MOTOR_CW,
                                   config::driver::MOTOR_CW,
                                   config::driver::MOTOR_CW,
                                   config::driver::MOTOR_CW};
    duties[motor_idx] = duty;
    dirs[motor_idx] = direction;
    BL48250::get_instance().set_duties(duties, dirs);
}

// The estimator init is idempotent, so suites that share the singleton can
// call it unconditionally.
static void ensure_estimator()
{
    TEST_ASSERT_EQUAL(
      ESP_OK,
      WheelStateEstimator::get_instance().init(config::test::WHEEL_ADC_CHANNELS, ADC_ATTEN_DB_12));
}

// ============================================================================
// Pulse-feedback gathering (driver level)
// ============================================================================

// Feedback counts accumulated on every motor across a delay.
static std::array<int32_t, 4> count_delta_over(uint32_t delay_ms)
{
    const std::array<int32_t, 4> before = BL48250::get_instance().pulse_counts();
    DELAY_MS(delay_ms);
    const std::array<int32_t, 4> after = BL48250::get_instance().pulse_counts();

    std::array<int32_t, 4> delta = {0, 0, 0, 0};
    for (size_t i = 0; i < 4; ++i) delta[i] = after[i] - before[i];
    return delta;
}

void test_feedback_counts_while_spinning()
{
    spin_motor(0, config::test::TEST_DUTY, config::driver::MOTOR_CW);
    DELAY_MS(1000);  // ramp up
    const std::array<int32_t, 4> delta = count_delta_over(500);
    spin_motor(0, 0, config::driver::MOTOR_CW);

    ESP_LOGI(TAG,
             "CW 500 ms counts: M1=%" PRIi32 " M2=%" PRIi32 " M3=%" PRIi32 " M4=%" PRIi32,
             delta[0],
             delta[1],
             delta[2],
             delta[3]);

    TEST_ASSERT_TRUE_MESSAGE(delta[0] > config::test::MIN_SPIN_COUNTS,
                             "Motor 1 pulse counter did not count while spinning CW");
    for (size_t i = 1; i < 4; ++i)
        TEST_ASSERT_TRUE_MESSAGE(delta[i] * 3 < delta[0], "Idle motor counted feedback pulses");
}

// The sliding-window velocity follows the commanded direction sign.
void test_feedback_reverses_with_direction()
{
    spin_motor(0, config::test::TEST_DUTY, config::driver::MOTOR_CW);
    DELAY_MS(1000);
    float cw_mean = 0.0f;
    for (size_t i = 0; i < 10; ++i)
    {
        DELAY_MS(60);
        float velocity = 0.0f;
        TEST_ASSERT_TRUE(BL48250::get_instance().pulse_velocity_rad_s(0, &velocity));
        cw_mean += velocity;
    }
    spin_motor(0, 0, config::driver::MOTOR_CW);
    DELAY_MS(300);

    spin_motor(0, config::test::TEST_DUTY, config::driver::MOTOR_CCW);
    DELAY_MS(1000);
    float ccw_mean = 0.0f;
    for (size_t i = 0; i < 10; ++i)
    {
        DELAY_MS(60);
        float velocity = 0.0f;
        TEST_ASSERT_TRUE(BL48250::get_instance().pulse_velocity_rad_s(0, &velocity));
        ccw_mean += velocity;
    }
    spin_motor(0, 0, config::driver::MOTOR_CCW);

    cw_mean /= 10.0f;
    ccw_mean /= 10.0f;
    ESP_LOGI(TAG, "Pulse velocity: CW %.2f rad/s, CCW %.2f rad/s", cw_mean, ccw_mean);

    TEST_ASSERT_TRUE_MESSAGE(cw_mean > config::test::SPIN_VELOCITY_RAD_S,
                             "CW run reported no positive velocity");
    TEST_ASSERT_TRUE_MESSAGE(ccw_mean < -config::test::SPIN_VELOCITY_RAD_S,
                             "CCW run reported no negative velocity");
}

void test_feedback_reads_zero_when_stopped()
{
    spin_motor(0, config::test::TEST_DUTY, config::driver::MOTOR_CW);
    DELAY_MS(500);
    spin_motor(0, 0, config::driver::MOTOR_CW);
    DELAY_MS(700);  // let the wheel spin down

    const std::array<int32_t, 4> delta = count_delta_over(500);
    TEST_ASSERT_TRUE_MESSAGE(delta[0] < 5, "Stopped motor counted feedback pulses");
}

// Without feedback pins the counter is absent, and the driver must report the
// measurement as unavailable instead of a zero velocity.
void test_feedback_disabled_without_pins()
{
    configure_driver(false);

    float velocity = 1.0f;
    TEST_ASSERT_FALSE(BL48250::get_instance().pulse_velocity_rad_s(0, &velocity));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, velocity);
}

// ============================================================================
// Estimator scenario at fixed PWM (production fusion path)
// ============================================================================

struct ScenarioStats
{
    float velocity_mean;
    float velocity_std;
    float pulse_mean;
    float filtered_angle_delta;
    float pulse_angle_delta;
};

// Runs the production estimation path: spins motor 1 at the fixed duty and
// samples the wheel state while it turns. The pulse velocity, read
// independently of the filter, is the reference.
static ScenarioStats run_estimation_scenario(const char * label)
{
    auto & estimator = WheelStateEstimator::get_instance();

    spin_motor(0, config::test::TEST_DUTY, config::driver::MOTOR_CW);
    DELAY_MS(1200);  // ramp up and settle

    constexpr size_t SAMPLES = 100;
    std::array<float, SAMPLES> samples = {};
    float pulse_mean = 0.0f;
    for (size_t i = 0; i < SAMPLES; ++i)
    {
        DELAY_MS(10);
        samples[i] = estimator.get_filtered_velocity_rad_s()[0];
        float velocity = 0.0f;
        if (BL48250::get_instance().pulse_velocity_rad_s(0, &velocity))
            pulse_mean += velocity;
    }
    spin_motor(0, 0, config::driver::MOTOR_CW);
    pulse_mean /= static_cast<float>(SAMPLES);

    // Position cross-check over a short measured interval: the filtered angle
    // delta against the angle the pulse counter traveled in the same time.
    const float angle_start = estimator.get_filtered_angle_rad()[0];
    const int32_t count_start = BL48250::get_instance().pulse_counts()[0];
    DELAY_MS(20);
    const float angle_end = estimator.get_filtered_angle_rad()[0];
    const int32_t count_end = BL48250::get_instance().pulse_counts()[0];

    ScenarioStats stats = {};
    for (const float sample : samples) stats.velocity_mean += sample;
    stats.velocity_mean /= static_cast<float>(SAMPLES);
    for (const float sample : samples)
        stats.velocity_std += (sample - stats.velocity_mean) * (sample - stats.velocity_mean);
    stats.velocity_std = std::sqrt(stats.velocity_std / static_cast<float>(SAMPLES));
    stats.pulse_mean = pulse_mean;
    stats.filtered_angle_delta = angle_end - angle_start;
    stats.pulse_angle_delta =
      static_cast<float>(count_end - count_start) * config::driver::BL48250_FB_RAD_S_PER_COUNT;

    ESP_LOGI(TAG,
             "[%s] est vel %.2f +- %.2f rad/s | pulse %.2f rad/s | dAngle est %.2f, pulse %.2f rad",
             label,
             stats.velocity_mean,
             stats.velocity_std,
             stats.pulse_mean,
             stats.filtered_angle_delta,
             stats.pulse_angle_delta);
    return stats;
}

void test_estimator_tracks_spin()
{
    ensure_estimator();
    const ScenarioStats stats = run_estimation_scenario("angle + motor");

    // The estimator reports the spin, agrees with the pulse reference on sign,
    // and lands in a sane magnitude band around it.
    TEST_ASSERT_TRUE_MESSAGE(std::fabs(stats.velocity_mean) > config::test::SPIN_VELOCITY_RAD_S,
                             "Estimator did not report the spin");
    TEST_ASSERT_TRUE_MESSAGE(stats.velocity_mean * -stats.pulse_mean > 0.0f,
                             "Estimator sign disagrees with the pulse reference");
    TEST_ASSERT_TRUE_MESSAGE(std::fabs(stats.velocity_mean) > 0.6f * std::fabs(stats.pulse_mean) &&
                               std::fabs(stats.velocity_mean) < stats.pulse_mean / 0.6f,
                             "Estimator velocity diverged from the pulse reference");
}

// How fast the fused estimate follows a direction command change.
void test_reversal_tracking()
{
    ensure_estimator();
    auto & estimator = WheelStateEstimator::get_instance();

    spin_motor(0, config::test::TEST_DUTY, config::driver::MOTOR_CW);
    DELAY_MS(1200);

    spin_motor(0, config::test::TEST_DUTY, config::driver::MOTOR_CCW);
    const uint64_t flip_us = static_cast<uint64_t>(esp_timer_get_time());
    uint64_t crossed_us = 0;
    for (size_t i = 0; i < 80; ++i)  // up to 800 ms of 10 ms samples
    {
        DELAY_MS(10);
        if (estimator.get_filtered_velocity_rad_s()[0] > 0.0f)
        {
            crossed_us = static_cast<uint64_t>(esp_timer_get_time()) - flip_us;
            break;
        }
    }
    spin_motor(0, 0, config::driver::MOTOR_CCW);

    ESP_LOGI(TAG, "Reversal sign crossing after %llu us", (unsigned long long)crossed_us);
    TEST_ASSERT_TRUE_MESSAGE(crossed_us != 0 && crossed_us < 800000,
                             "Estimator did not follow the reversal within 800 ms");
}

// The estimate decays with the wheel after the duty is cut.
void test_spin_down_to_rest()
{
    ensure_estimator();
    auto & estimator = WheelStateEstimator::get_instance();

    spin_motor(0, config::test::TEST_DUTY, config::driver::MOTOR_CW);
    DELAY_MS(1200);
    const float spinning_velocity = estimator.get_filtered_velocity_rad_s()[0];
    spin_motor(0, 0, config::driver::MOTOR_CW);
    DELAY_MS(1500);
    const float settled_velocity = estimator.get_filtered_velocity_rad_s()[0];

    ESP_LOGI(TAG, "Spin down: %.2f -> %.2f rad/s", spinning_velocity, settled_velocity);
    TEST_ASSERT_TRUE_MESSAGE(std::fabs(settled_velocity) < 0.5f * std::fabs(spinning_velocity),
                             "Estimator did not decay with the wheel after the duty was cut");
}

void setUp(void)
{
    configure_driver(true);
}

void tearDown(void)
{
    BL48250::get_instance().set_duties({0, 0, 0, 0}, {0, 0, 0, 0});
    BL48250::get_instance().deinit();
}

extern "C" void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_feedback_counts_while_spinning);
    RUN_TEST(test_feedback_reverses_with_direction);
    RUN_TEST(test_feedback_reads_zero_when_stopped);
    RUN_TEST(test_feedback_disabled_without_pins);
    RUN_TEST(test_estimator_tracks_spin);
    RUN_TEST(test_reversal_tracking);
    RUN_TEST(test_spin_down_to_rest);
    UNITY_END();
}
