/**
 * @file test_RippleLimit.cpp
 * @brief Autonomous bench sketch: what low-speed limit and minimum ripple can
 *        the BL48250 drivetrain reach at 10 RPM?
 *
 * Flash this suite (test_calibration environment, 1 kHz FreeRTOS tick) and open
 * the serial monitor. The sketch initializes the encoders and the motor driver
 * directly (no WheelController task), then runs three experiment phases on the
 * wheels with a trustworthy analog encoder:
 *
 *   1. KICK: open-loop kick duty/duration from rest; reports end/peak speed.
 *   2. PI:   1 kHz duty PI around the measured breakaway center, sweeping gains
 *            and the velocity window; reports mean/std/p-p in 10/20 ms windows.
 *   3. DITHER: kick followed by a 1 kHz high/low duty pattern; reports the same
 *            metrics. This checks whether fast command chopping can hold a
 *            low average speed without stopping the wheel.
 *
 * Every run prints one machine-readable line starting with "RES". At the end it
 * prints the best result of each phase and "LIMIT_DONE". The sketch always
 * leaves the motors stopped; a speed guard aborts a run above 250 RPM.
 *
 * Expected verdict (from the 2026-11-19/20 campaign): no combination holds
 * 9-11 RPM in 10-20 ms windows; the phase reports quantify the actual limit
 * (mean reachable, minimum p-p) instead of assuming it.
 */

#include <driver/i2c_master.h>
#include <driver/uart.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdkconfig.h>
#include <stdio.h>

#include <array>
#include <cmath>

#include "../../shared_test_spi.h"
#include "74HC595.h"
#include "AS5600.h"
#include "BL48250.h"
#include "CD4052B.h"
#include "I2Cdev.h"
#include "NVSManager.h"
#include "constants.h"
#include "wheel_state_estimator.h"

namespace config
{
namespace ripple
{
constexpr std::array<gpio_num_t, 4> MOTOR_PWM_PINS = {(gpio_num_t)CONFIG_MOTOR_FL_PWM_GPIO,
                                                      (gpio_num_t)CONFIG_MOTOR_BL_PWM_GPIO,
                                                      (gpio_num_t)CONFIG_MOTOR_BR_PWM_GPIO,
                                                      (gpio_num_t)CONFIG_MOTOR_FR_PWM_GPIO};
constexpr std::array<uint8_t, 4> MOTOR_DIR_BITS = {CONFIG_SHIFT_REGISTER_DIR1_BIT,
                                                   CONFIG_SHIFT_REGISTER_DIR2_BIT,
                                                   CONFIG_SHIFT_REGISTER_DIR3_BIT,
                                                   CONFIG_SHIFT_REGISTER_DIR4_BIT};
constexpr std::array<gpio_num_t, 4> MOTOR_FB_PINS = {(gpio_num_t)CONFIG_MOTOR_FL_FB_GPIO,
                                                     (gpio_num_t)CONFIG_MOTOR_BL_FB_GPIO,
                                                     (gpio_num_t)CONFIG_MOTOR_BR_FB_GPIO,
                                                     (gpio_num_t)CONFIG_MOTOR_FR_FB_GPIO};
constexpr std::array<ledc_channel_t, 4> MOTOR_CHANNELS = {
  LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
constexpr ledc_timer_bit_t DUTY_RESOLUTION = LEDC_TIMER_10_BIT;
constexpr uint32_t PWM_FREQ = 20000;
const std::array<adc_channel_t, 4> WHEEL_ADC_CHANNELS = {
  static_cast<adc_channel_t>(CONFIG_MOTOR_FL_ENC_CHANNEL),
  static_cast<adc_channel_t>(CONFIG_MOTOR_BL_ENC_CHANNEL),
  static_cast<adc_channel_t>(CONFIG_MOTOR_BR_ENC_CHANNEL),
  static_cast<adc_channel_t>(CONFIG_MOTOR_FR_ENC_CHANNEL)};
}  // namespace ripple
}  // namespace config

static HC595 s_shift_register;
static CD4052B s_cd4052b;

// ---------------------------------------------------------------------------
// Motor output helpers (direct driver access; no WheelController task)
// ---------------------------------------------------------------------------
static std::array<int16_t, 4> s_duty = {0, 0, 0, 0};

static void apply_duties()
{
    std::array<uint32_t, 4> duties = {0, 0, 0, 0};
    std::array<uint8_t, 4> directions = {config::driver::MOTOR_CW,
                                         config::driver::MOTOR_CW,
                                         config::driver::MOTOR_CW,
                                         config::driver::MOTOR_CW};
    for (size_t wheel = 0; wheel < 4; ++wheel)
    {
        const int16_t duty = s_duty[wheel];
        duties[wheel] = static_cast<uint32_t>(duty < 0 ? -static_cast<int>(duty) : duty);
        directions[wheel] = (duty >= 0) ? config::driver::MOTOR_CW : config::driver::MOTOR_CCW;
    }
    BL48250::get_instance().set_duties(duties, directions);
}

static void stop_all()
{
    s_duty.fill(0);
    apply_duties();
}

static void set_wheel_duty(int wheel, int16_t signed_duty)
{
    s_duty[wheel] = signed_duty;
    apply_duties();
}

// ---------------------------------------------------------------------------
// Fast velocity from the analog AS5600 angle (same idea as the production
// fast estimator: unwrap the motor-shaft angle, least-squares slope over a few
// milliseconds).
// ---------------------------------------------------------------------------
static constexpr size_t VEL_WINDOW = 8;
static std::array<float, VEL_WINDOW> s_angle_ring = {};
static std::array<int64_t, VEL_WINDOW> s_time_ring = {};
static size_t s_ring_head = 0;
static size_t s_ring_count = 0;
static float s_wheel_angle_rad = 0.0f;
static float s_last_motor_angle_rad = 0.0f;
static bool s_angle_seeded = false;
static float s_velocity_rad_s = 0.0f;  // motor frame, positive for positive duty

static constexpr float WHEEL_ANGLE_PERIOD_RAD =
  2.0f * static_cast<float>(M_PI) / config::driver::WHEEL_REDUCTION;

static void update_velocity(int wheel)
{
    const float motor_angle = WheelStateEstimator::get_instance().get_measured_angle_rad()[wheel];
    const int64_t now_us = esp_timer_get_time();
    if (!s_angle_seeded)
    {
        s_last_motor_angle_rad = motor_angle;
        s_wheel_angle_rad = motor_angle / config::driver::WHEEL_REDUCTION;
        s_angle_seeded = true;
        return;
    }
    float difference = motor_angle - s_last_motor_angle_rad;
    const float half_period = 0.5f * static_cast<float>(M_PI);
    if (difference > half_period)
        difference -= static_cast<float>(M_PI);
    else if (difference < -half_period)
        difference += static_cast<float>(M_PI);
    s_last_motor_angle_rad = motor_angle;
    s_wheel_angle_rad += difference / config::driver::WHEEL_REDUCTION;

    s_angle_ring[s_ring_head] = s_wheel_angle_rad;
    s_time_ring[s_ring_head] = now_us;
    s_ring_head = (s_ring_head + 1) % VEL_WINDOW;
    if (s_ring_count < VEL_WINDOW)
        ++s_ring_count;
    if (s_ring_count == VEL_WINDOW)
    {
        const size_t oldest = s_ring_head;
        const int64_t origin = s_time_ring[oldest];
        double sum_t = 0.0, sum_tt = 0.0, sum_a = 0.0, sum_ta = 0.0;
        for (size_t k = 0; k < VEL_WINDOW; ++k)
        {
            const size_t pos = (oldest + k) % VEL_WINDOW;
            const double t = static_cast<double>(s_time_ring[pos] - origin) * 1e-6;
            const double a = static_cast<double>(s_angle_ring[pos]);
            sum_t += t;
            sum_tt += t * t;
            sum_a += a;
            sum_ta += t * a;
        }
        const double n = static_cast<double>(VEL_WINDOW);
        const double denominator = n * sum_tt - sum_t * sum_t;
        if (denominator > 1e-12)
        {
            // Controller frame is the motor frame; the analog angle decreases
            // for a positive rotation, so negate. A one-count glitch of an
            // intermittent analog channel can look like a huge speed: reject
            // implausible samples instead of aborting the whole run.
            const float candidate = static_cast<float>(-(n * sum_ta - sum_t * sum_a) / denominator);
            const float candidate_rpm = candidate * 60.0f / (2.0f * static_cast<float>(M_PI));
            if (std::fabs(candidate_rpm) < 300.0f)
                s_velocity_rad_s = candidate;
        }
    }
}

static float velocity_rpm()
{
    return s_velocity_rad_s * 60.0f / (2.0f * static_cast<float>(M_PI));
}

// ---------------------------------------------------------------------------
// Window metrics: store 1 kHz samples for one run and summarize.
// ---------------------------------------------------------------------------
static constexpr size_t RUN_MAX_SAMPLES = 2600;  // 2.6 s at 1 kHz
static std::array<float, RUN_MAX_SAMPLES> s_run_speed = {};
static size_t s_run_samples = 0;

static void run_begin()
{
    s_run_samples = 0;
    s_ring_count = 0;
    s_ring_head = 0;
    s_angle_seeded = false;
    s_velocity_rad_s = 0.0f;
}

static void run_sample(int wheel)
{
    update_velocity(wheel);
    if (s_run_samples < RUN_MAX_SAMPLES)
        s_run_speed[s_run_samples++] = velocity_rpm();
}

struct RunMetrics
{
    float mean = 0.0f;
    float std = 0.0f;
    float min = 0.0f;
    float max = 0.0f;
    float pp10 = 0.0f;
    float pp20 = 0.0f;
    float outside10_pct = 0.0f;
    float outside20_pct = 0.0f;
    float stuck_frac = 0.0f;
    float mean_abs = 0.0f;
    float motion_frac = 0.0f;  // fraction of samples with |speed| > 2 RPM
    size_t samples = 0;
    bool aborted = false;
};

static RunMetrics run_metrics()
{
    RunMetrics result;
    result.samples = s_run_samples;
    if (s_run_samples < 100)
        return result;
    const size_t count = s_run_samples;
    double sum = 0.0;
    result.min = s_run_speed[0];
    result.max = s_run_speed[0];
    size_t stuck = 0;
    size_t moving = 0;
    double sum_abs = 0.0;
    for (size_t i = 0; i < count; ++i)
    {
        const float value = s_run_speed[i];
        sum += value;
        sum_abs += std::fabs(value);
        if (value < result.min)
            result.min = value;
        if (value > result.max)
            result.max = value;
        if (std::fabs(value) < 1.0f)
            ++stuck;
        if (std::fabs(value) > 2.0f)
            ++moving;
    }
    result.mean_abs = static_cast<float>(sum_abs / static_cast<double>(count));
    result.motion_frac = static_cast<float>(moving) / static_cast<float>(count);
    result.mean = static_cast<float>(sum / static_cast<double>(count));
    double variance = 0.0;
    for (size_t i = 0; i < count; ++i)
    {
        const double delta = s_run_speed[i] - result.mean;
        variance += delta * delta;
    }
    result.std = static_cast<float>(std::sqrt(variance / static_cast<double>(count)));
    result.stuck_frac = static_cast<float>(stuck) / static_cast<float>(count);

    auto window_stats = [&](size_t window, float * pp, float * outside_pct) {
        *pp = 0.0f;
        *outside_pct = 0.0f;
        if (count < window)
            return;
        float low = 0.0f;
        float high = 0.0f;
        size_t windows = 0;
        size_t outside = 0;
        for (size_t start = 0; start + window <= count; start += window)
        {
            double mean = 0.0;
            for (size_t k = 0; k < window; ++k) mean += s_run_speed[start + k];
            mean /= static_cast<double>(window);
            if (windows == 0 || mean < low)
                low = static_cast<float>(mean);
            if (windows == 0 || mean > high)
                high = static_cast<float>(mean);
            ++windows;
            if (mean < 9.0 || mean > 11.0)
                ++outside;
        }
        *pp = high - low;
        *outside_pct =
          windows > 0 ? 100.0f * static_cast<float>(outside) / static_cast<float>(windows) : 0.0f;
    };
    window_stats(10, &result.pp10, &result.outside10_pct);
    window_stats(20, &result.pp20, &result.outside20_pct);
    return result;
}

// ---------------------------------------------------------------------------
// Best-limit tracking: lowest p-p20 among runs whose average is in 5..15 RPM.
// ---------------------------------------------------------------------------
struct BestLimit
{
    bool valid = false;
    float pp20 = 1e9f;
    float pp10 = 0.0f;
    float mean = 0.0f;
    float param1 = 0.0f;
    float param2 = 0.0f;
    float param3 = 0.0f;
    float param4 = 0.0f;
};
static std::array<BestLimit, 4> s_best_pi = {};
static std::array<BestLimit, 4> s_best_dither = {};

static void record_best(
  BestLimit * best, const RunMetrics & metrics, float p1, float p2, float p3, float p4)
{
    // Only real low-speed regulation counts: positive average in the 5..15 RPM
    // range, the wheel must actually be turning most of the time, and the run
    // must not have been aborted by the speed guard. A stopped wheel with
    // measurement noise is not a candidate for the 9-11 RPM band.
    if (metrics.aborted || metrics.motion_frac < 0.3f)
        return;
    if (metrics.mean < 5.0f || metrics.mean > 15.0f)
        return;
    if (metrics.pp20 >= best->pp20)
        return;
    best->valid = true;
    best->pp20 = metrics.pp20;
    best->pp10 = metrics.pp10;
    best->mean = metrics.mean;
    best->param1 = p1;
    best->param2 = p2;
    best->param3 = p3;
    best->param4 = p4;
}

// ---------------------------------------------------------------------------
// Phase 1: kick calibration (open loop from rest).
// ---------------------------------------------------------------------------
static void phase_kick(int wheel)
{
    printf("PHASE KICK wheel=%d\n", wheel + 1);
    const int16_t duties[] = {350, 380, 400, 450, 500};
    const uint32_t durations_ms[] = {60, 100, 150, 200};
    for (int16_t duty : duties)
    {
        for (uint32_t duration : durations_ms)
        {
            stop_all();
            vTaskDelay(pdMS_TO_TICKS(300));
            run_begin();
            const int64_t start = esp_timer_get_time();
            const int64_t kick_end = start + static_cast<int64_t>(duration) * 1000;
            while (esp_timer_get_time() - start < 400000)
            {
                const int64_t now = esp_timer_get_time();
                set_wheel_duty(wheel, now < kick_end ? duty : 0);
                run_sample(wheel);
                vTaskDelay(1);
            }
            stop_all();
            const RunMetrics metrics = run_metrics();
            float end_speed = 0.0f;
            const size_t end_index = static_cast<size_t>(duration);
            if (end_index < s_run_samples)
                end_speed = s_run_speed[end_index];
            printf("RES KICK wheel=%d duty=%d ms=%lu end_rpm=%.1f peak=%.1f mean=%.1f\n",
                   wheel + 1,
                   static_cast<int>(duty),
                   static_cast<unsigned long>(duration),
                   static_cast<double>(end_speed),
                   static_cast<double>(metrics.max),
                   static_cast<double>(metrics.mean));
        }
    }
}

// ---------------------------------------------------------------------------
// Phase 2: 1 kHz PI on the fast velocity estimate.
// ---------------------------------------------------------------------------
static void run_pi(int wheel, float kp, float ki, int16_t center, uint32_t duration_ms)
{
    stop_all();
    vTaskDelay(pdMS_TO_TICKS(300));
    run_begin();
    float integral = 0.0f;
    uint32_t over_speed = 0;
    bool aborted = false;
    const int64_t start = esp_timer_get_time();
    while (esp_timer_get_time() - start < static_cast<int64_t>(duration_ms) * 1000)
    {
        run_sample(wheel);
        const float error = 10.0f - velocity_rpm();  // target 10 RPM
        integral += ki * error * 0.001f;
        if (integral > 250.0f)
            integral = 250.0f;
        else if (integral < -250.0f)
            integral = -250.0f;
        float duty = center + kp * error + integral;
        if (duty < 0.0f)
            duty = 0.0f;
        else if (duty > 520.0f)
            duty = 520.0f;
        set_wheel_duty(wheel, static_cast<int16_t>(std::lround(duty)));
        over_speed =
          (s_ring_count == VEL_WINDOW && std::fabs(velocity_rpm()) > 250.0f) ? over_speed + 1 : 0;
        if (over_speed >= 3)
        {
            aborted = true;
            break;
        }
        vTaskDelay(1);
    }
    stop_all();
    RunMetrics metrics = run_metrics();
    metrics.aborted = aborted;
    record_best(&s_best_pi[wheel], metrics, kp, ki, static_cast<float>(center), 0.0f);
    printf(
      "RES PI wheel=%d kp=%.1f ki=%.0f center=%d mean=%.2f std=%.2f pp10=%.2f pp20=%.2f "
      "out=%.1f stuck=%.2f mabs=%.2f mov=%.2f duty=%.0f samples=%u aborted=%d\n",
      wheel + 1,
      static_cast<double>(kp),
      static_cast<double>(ki),
      static_cast<int>(center),
      static_cast<double>(metrics.mean),
      static_cast<double>(metrics.std),
      static_cast<double>(metrics.pp10),
      static_cast<double>(metrics.pp20),
      static_cast<double>(metrics.outside20_pct),
      static_cast<double>(metrics.stuck_frac),
      static_cast<double>(metrics.mean_abs),
      static_cast<double>(metrics.motion_frac),
      static_cast<double>(center + integral),
      static_cast<unsigned>(metrics.samples),
      aborted ? 1 : 0);
}

static void phase_pi(int wheel, int16_t center)
{
    printf("PHASE PI wheel=%d center=%d\n", wheel + 1, static_cast<int>(center));
    const float kps[] = {1.0f, 2.0f, 5.0f};
    const float kis[] = {10.0f, 50.0f};
    for (float kp : kps)
        for (float ki : kis) run_pi(wheel, kp, ki, center, 2000);
}

// ---------------------------------------------------------------------------
// Phase 3: kick, then a 1 kHz high/low duty pattern (no stop between).
// ---------------------------------------------------------------------------
static void run_dither(int wheel,
                       int16_t kick_duty,
                       uint32_t kick_ms,
                       int16_t high,
                       int16_t low,
                       uint32_t on_ms,
                       uint32_t period_ms,
                       uint32_t duration_ms)
{
    stop_all();
    vTaskDelay(pdMS_TO_TICKS(300));
    run_begin();
    const int64_t start = esp_timer_get_time();
    const int64_t kick_end = start + static_cast<int64_t>(kick_ms) * 1000;
    uint32_t tick = 0;
    uint32_t over_speed = 0;
    bool aborted = false;
    while (esp_timer_get_time() - start < static_cast<int64_t>(duration_ms) * 1000)
    {
        const int64_t now = esp_timer_get_time();
        int16_t duty = 0;
        if (now < kick_end)
            duty = kick_duty;
        else
        {
            const uint32_t phase = tick % (period_ms > 0 ? period_ms : 1);
            duty = (phase < on_ms) ? high : low;
        }
        set_wheel_duty(wheel, duty);
        run_sample(wheel);
        ++tick;
        over_speed =
          (s_ring_count == VEL_WINDOW && std::fabs(velocity_rpm()) > 250.0f) ? over_speed + 1 : 0;
        if (over_speed >= 3)
        {
            aborted = true;
            break;
        }
        vTaskDelay(1);
    }
    stop_all();
    RunMetrics metrics = run_metrics();
    metrics.aborted = aborted;
    record_best(&s_best_dither[wheel],
                metrics,
                static_cast<float>(high),
                static_cast<float>(low),
                static_cast<float>(period_ms),
                static_cast<float>(on_ms));
    printf(
      "RES DITHER wheel=%d kick=%d/%lu hi=%d lo=%d on=%lu p=%lu mean=%.2f std=%.2f "
      "pp10=%.2f pp20=%.2f out=%.1f stuck=%.2f mabs=%.2f mov=%.2f samples=%u aborted=%d\n",
      wheel + 1,
      static_cast<int>(kick_duty),
      static_cast<unsigned long>(kick_ms),
      static_cast<int>(high),
      static_cast<int>(low),
      static_cast<unsigned long>(on_ms),
      static_cast<unsigned long>(period_ms),
      static_cast<double>(metrics.mean),
      static_cast<double>(metrics.std),
      static_cast<double>(metrics.pp10),
      static_cast<double>(metrics.pp20),
      static_cast<double>(metrics.outside20_pct),
      static_cast<double>(metrics.stuck_frac),
      static_cast<double>(metrics.mean_abs),
      static_cast<double>(metrics.motion_frac),
      static_cast<unsigned>(metrics.samples),
      aborted ? 1 : 0);
}

static void phase_dither(int wheel, int16_t kick_duty, uint32_t kick_ms)
{
    printf("PHASE DITHER wheel=%d kick=%d/%lu\n",
           wheel + 1,
           static_cast<int>(kick_duty),
           static_cast<unsigned long>(kick_ms));
    const int16_t highs[] = {360, 400};
    const int16_t lows[] = {0, 300, 330};
    struct Pattern
    {
        uint32_t period;
        uint32_t on;
    };
    const Pattern patterns[] = {{2, 1}, {5, 2}, {10, 4}, {20, 10}};
    for (int16_t high : highs)
        for (int16_t low : lows)
            for (const Pattern & pattern : patterns)
                run_dither(wheel, kick_duty, kick_ms, high, low, pattern.on, pattern.period, 2000);
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------
static void init_uart()
{
    uart_config_t uart_config = {};
    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_driver_install(UART_NUM_0, 1024, 0, 0, nullptr, 0);
    uart_param_config(UART_NUM_0, &uart_config);
}

static void init_hardware()
{
    NVSManager::init();
    NVSManager::start();
    shared_test_spi_bus();
    HC595::Config expander_config = {.latch_gpio = (gpio_num_t)CONFIG_SHIFT_REGISTER_LATCH_GPIO,
                                     .device_key = CONFIG_SHIFT_REGISTER_DEVICE_KEY,
                                     .clock_hz = 1000000};
    ESP_ERROR_CHECK(s_shift_register.init(expander_config));
    CD4052B::Config mux_config = {.select_a_bit = CONFIG_SHIFT_REGISTER_MUX_A_BIT,
                                  .select_b_bit = CONFIG_SHIFT_REGISTER_MUX_B_BIT,
                                  .inhibit_bit = CONFIG_SHIFT_REGISTER_MUX_INH_BIT};
    ESP_ERROR_CHECK(s_cd4052b.init(&s_shift_register, mux_config));

    i2c_master_bus_config_t i2c_mst_config = {
      .i2c_port = (i2c_port_t)CONFIG_I2C_PORT_NUM,
      .sda_io_num = (gpio_num_t)CONFIG_SDA_GPIO,
      .scl_io_num = (gpio_num_t)CONFIG_SCL_GPIO,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags = {.enable_internal_pullup = true, .allow_pd = false}};
    i2c_master_bus_handle_t i2c_bus = nullptr;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_bus));
    I2Cdev::init(i2c_bus);

    WheelStateEstimator::get_instance().init(
      config::ripple::WHEEL_ADC_CHANNELS, ADC_ATTEN_DB_12, &s_cd4052b);

    config::driver::MotorDriverConfig motor_cfg;
    motor_cfg.timer = LEDC_TIMER_0;
    motor_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    motor_cfg.duty_resolution = config::ripple::DUTY_RESOLUTION;
    motor_cfg.pwm_freq = config::ripple::PWM_FREQ;
    motor_cfg.motor_pwm_pins = config::ripple::MOTOR_PWM_PINS;
    motor_cfg.motor_channels = config::ripple::MOTOR_CHANNELS;
    motor_cfg.direction_shift_register = &s_shift_register;
    motor_cfg.direction_bits = config::ripple::MOTOR_DIR_BITS;
    motor_cfg.motor_fb_pins = config::ripple::MOTOR_FB_PINS;
    ESP_ERROR_CHECK(BL48250::get_instance().configure(motor_cfg));
    stop_all();
}

extern "C" void app_main(void)
{
    init_uart();
    vTaskDelay(pdMS_TO_TICKS(500));
    printf("\nRIPPLE_LIMIT_SKETCH start (wheel 1/3/4; M2 analog unreliable)\n");
    init_hardware();
    printf("RIPPLE_LIMIT_SKETCH hardware ready\n");

    // Wheels 0 (M1), 2 (M3) and 3 (M4) have a trustworthy analog encoder.
    const int wheels[] = {0, 2, 3};
    for (int wheel : wheels)
    {
        phase_kick(wheel);
        // Rough center: 0.9 x the breakaway measured in the 2026-11-19 campaign.
        int16_t center = 337;
        if (wheel == 2)
            center = 309;
        else if (wheel == 3)
            center = 136;
        phase_pi(wheel, center);
        // M4 needs a longer kick (its T_break is ~180 ms); M1/M3 break away
        // within ~40 ms at 400 duty.
        if (wheel == 3)
            phase_dither(wheel, 300, 260);
        else
            phase_dither(wheel, 400, 150);

        if (s_best_pi[wheel].valid)
            printf("BEST PI wheel=%d mean=%.2f pp10=%.2f pp20=%.2f kp=%.1f ki=%.0f center=%.0f\n",
                   wheel + 1,
                   static_cast<double>(s_best_pi[wheel].mean),
                   static_cast<double>(s_best_pi[wheel].pp10),
                   static_cast<double>(s_best_pi[wheel].pp20),
                   static_cast<double>(s_best_pi[wheel].param1),
                   static_cast<double>(s_best_pi[wheel].param2),
                   static_cast<double>(s_best_pi[wheel].param3));
        else
            printf("BEST PI wheel=%d none (no run with mean 5..15 RPM)\n", wheel + 1);
        if (s_best_dither[wheel].valid)
            printf(
              "BEST DITHER wheel=%d mean=%.2f pp10=%.2f pp20=%.2f hi=%.0f lo=%.0f "
              "period=%.0f on=%.0f\n",
              wheel + 1,
              static_cast<double>(s_best_dither[wheel].mean),
              static_cast<double>(s_best_dither[wheel].pp10),
              static_cast<double>(s_best_dither[wheel].pp20),
              static_cast<double>(s_best_dither[wheel].param1),
              static_cast<double>(s_best_dither[wheel].param2),
              static_cast<double>(s_best_dither[wheel].param3),
              static_cast<double>(s_best_dither[wheel].param4));
        else
            printf("BEST DITHER wheel=%d none (no run with mean 5..15 RPM)\n", wheel + 1);
    }

    stop_all();
    printf("LIMIT_RESULT best_pp20_PI_M1=%.2f best_pp20_DITHER_M1=%.2f (see BEST lines)\n",
           static_cast<double>(s_best_pi[0].valid ? s_best_pi[0].pp20 : -1.0f),
           static_cast<double>(s_best_dither[0].valid ? s_best_dither[0].pp20 : -1.0f));
    printf("RIPPLE_LIMIT_SKETCH done; motors stopped\n");
    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
}
