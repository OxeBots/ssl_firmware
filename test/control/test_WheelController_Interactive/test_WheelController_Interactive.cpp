#include <driver/i2c_master.h>
#include <driver/spi_master.h>
#include <driver/uart.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdkconfig.h>
#include <stdio.h>
#include <unity.h>

#include <array>
#include <bitset>
#include <cmath>

#include "74HC595.h"
#include "AS5600.h"
#include "BL48250.h"
#include "CD4052B.h"
#include "I2Cdev.h"
#include "NVSManager.h"
#include "SPIdev.h"
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
}  // namespace test
}  // namespace config

static HC595 s_shift_register;
static CD4052B s_cd4052b;

// Applies new PID gains to all wheels, keeping their feedforward terms.
static void set_all_gains(WheelController & controller,
                          const std::array<float, 4> & kp,
                          const std::array<float, 4> & ki,
                          const std::array<float, 4> & kd)
{
    const auto current = controller.get_parameters();
    for (int wheel = 0; wheel < 4; ++wheel)
    {
        WheelController::Parameters parameters = current[wheel];
        parameters.kp = kp[wheel];
        parameters.ki = ki[wheel];
        parameters.kd = kd[wheel];
        controller.apply_parameters(wheel, parameters);
    }
}

// I2C-only handle used to probe the AS5600 encoders on demand (E command).
// The ADC constructor parameters are unused; raw angles come over I2C through
// the CD4052B mux, one wheel at a time. The ADC estimator task never touches
// the mux, so both paths can run concurrently.
static AS5600 s_stream_encoder(ADC_CHANNEL_0, nullptr, false);
static i2c_master_bus_handle_t s_i2c_bus = nullptr;
static bool s_manual_mode = false;  // true = control task paused, raw 595 writes allowed

// Manual signed duty per wheel, applied directly through the BL48250 driver.
// Manual mode (M) pauses the control task first, so it cannot overwrite these.
static std::array<int16_t, 4> s_raw_duty = {0, 0, 0, 0};
// Wheel whose AS5600 the CD4052B mux is focused on, or -1 when released.
static int8_t s_i2c_wheel = -1;
// Last accepted serial command (manual-mode dead-man) and the harness timeout.
static int64_t s_last_command_us = 0;
static constexpr int64_t MANUAL_TIMEOUT_US = 5000000;

// ============================================================================
// OPEN-LOOP HARNESS: raw duties, pulse timelines and interval recording
// ============================================================================
// The bench characterization needs three things the velocity loop cannot give:
// raw duty steps (the BL48250 module behaviour dominates below breakaway), pulse
// trains whose edges must land within a few hundred microseconds, and a
// high-rate record of the FG counter plus the AS5600 angle. Manual mode pauses
// the control task, so the harness owns the four motor outputs.

// Skips a driver write when the outputs did not change: the 1 kHz loop calls
// this every tick and the four LEDC updates plus the shift-register write cost
// more than the control step itself otherwise.
static bool s_raw_written = false;

static void invalidate_raw_outputs_cache()
{
    s_raw_written = false;
}

static void write_raw_duties()
{
    static std::array<int16_t, 4> last_duty = {-32768, -32768, -32768, -32768};
    if (s_raw_written && last_duty == s_raw_duty)
        return;
    std::array<uint32_t, 4> duties = {0, 0, 0, 0};
    std::array<uint8_t, 4> directions = {config::driver::MOTOR_CW,
                                         config::driver::MOTOR_CW,
                                         config::driver::MOTOR_CW,
                                         config::driver::MOTOR_CW};
    for (size_t wheel = 0; wheel < 4; ++wheel)
    {
        const int16_t signed_duty = s_raw_duty[wheel];
        duties[wheel] =
          static_cast<uint32_t>(signed_duty < 0 ? -static_cast<int>(signed_duty) : signed_duty);
        directions[wheel] =
          (signed_duty >= 0) ? config::driver::MOTOR_CW : config::driver::MOTOR_CCW;
    }
    BL48250::get_instance().set_duties(duties, directions);
    last_duty = s_raw_duty;
    s_raw_written = true;
}

static void stop_raw_outputs()
{
    s_raw_duty.fill(0);
    write_raw_duties();
}

// -- Timed duty timeline -------------------------------------------------
// Up to TIMELINE_MAX_EVENTS duty changes played by a high-priority task. Each
// edge is scheduled with a one-shot timer and the task sleeps in between, so a
// fast pulse train (10-30 ms on-time) does not starve the other tasks.
static constexpr size_t TIMELINE_MAX_EVENTS = 1024;
struct TimelineEvent
{
    int64_t time_us;  // relative to the timeline start
    uint8_t wheel;
    int16_t duty;
};
static TimelineEvent s_timeline[TIMELINE_MAX_EVENTS];
static size_t s_timeline_count = 0;
static size_t s_timeline_index = 0;
static int64_t s_timeline_start_us = 0;
static volatile bool s_timeline_active = false;
static TaskHandle_t s_player_task = nullptr;
static esp_timer_handle_t s_player_timer = nullptr;
// Diagnostics: how often the player task woke up and wrote an output state.
static uint32_t s_player_wakeups = 0;
static uint32_t s_player_writes = 0;

static bool timeline_add(int64_t time_us, int wheel, int16_t duty)
{
    if (s_timeline_count >= TIMELINE_MAX_EVENTS)
        return false;
    s_timeline[s_timeline_count++] = {time_us, static_cast<uint8_t>(wheel), duty};
    return true;
}

static void player_timer_callback(void * arg)
{
    (void)arg;
    if (s_player_task != nullptr)
        xTaskNotifyGive(s_player_task);
}

static void player_task(void * arg)
{
    (void)arg;
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ++s_player_wakeups;
        bool waiting_for_edge = false;
        while (s_timeline_active)
        {
            const int64_t relative_us = esp_timer_get_time() - s_timeline_start_us;
            if (s_timeline_index >= s_timeline_count)
            {
                s_timeline_active = false;
                break;
            }
            const int64_t remaining_us = s_timeline[s_timeline_index].time_us - relative_us;
            if (remaining_us > 0)
            {
                esp_timer_stop(s_player_timer);
                esp_timer_start_once(s_player_timer, static_cast<uint64_t>(remaining_us));
                waiting_for_edge = true;
                break;
            }
            while (s_timeline_index < s_timeline_count &&
                   s_timeline[s_timeline_index].time_us <= relative_us)
            {
                s_raw_duty[s_timeline[s_timeline_index].wheel] = s_timeline[s_timeline_index].duty;
                ++s_timeline_index;
            }
            write_raw_duties();
            ++s_player_writes;
        }
        // Normal end and cancellation both leave every wheel off; a scheduled
        // edge keeps the output state in place.
        if (!waiting_for_edge)
            stop_raw_outputs();
    }
}

static void timeline_start()
{
    stop_raw_outputs();
    s_timeline_start_us = esp_timer_get_time();
    s_timeline_index = 0;
    s_timeline_active = true;
    esp_timer_stop(s_player_timer);
    xTaskNotifyGive(s_player_task);
}

static void timeline_cancel()
{
    s_timeline_active = false;
    s_timeline_index = s_timeline_count;
    if (s_player_timer != nullptr)
        esp_timer_stop(s_player_timer);
}

// -- Fixed-interval recorder ---------------------------------------------
// Samples the FG counter of the test wheel, the commanded signed duty and,
// when the mux is focused there, the AS5600 raw angle. Samples stay in RAM and
// are printed as CSV after the window, so the UART cannot disturb the timing.
// The end of the recording always stops the outputs (safe end of a maneuver).
// Samples are stored compactly: the 1-2 kHz ripple recordings need thousands of
// samples in the limited RAM (8 bytes each here instead of 12 with absolute
// timestamps and counters).
static constexpr size_t RECORD_MAX_SAMPLES = 8192;
struct RecordSample
{
    uint16_t delta_us;  // time since the previous sample
    int16_t angle;      // AS5600 raw angle, or -1 when not read/failed
    int16_t duty;       // commanded signed duty
    uint16_t fg_delta;  // FG counter delta since the previous sample
};
static RecordSample s_record_buffer[RECORD_MAX_SAMPLES];
static size_t s_record_count = 0;
static volatile bool s_record_active = false;
static volatile bool s_record_done = false;
static uint8_t s_record_wheel = 0;
static bool s_record_i2c = false;
static int64_t s_record_start_us = 0;
static int64_t s_record_next_us = 0;
static int64_t s_record_last_us = 0;
static int32_t s_record_last_fg = 0;
static uint32_t s_record_interval_us = 2000;
static uint32_t s_record_duration_us = 1000000;
static esp_timer_handle_t s_record_timer = nullptr;
static TaskHandle_t s_recorder_task = nullptr;

static void record_timer_callback(void * arg)
{
    (void)arg;
    if (s_recorder_task != nullptr)
        xTaskNotifyGive(s_recorder_task);
}

static void recorder_task(void * arg)
{
    (void)arg;
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!s_record_active)
            continue;
        const int64_t now_us = esp_timer_get_time();
        if (now_us < s_record_next_us)
            continue;
        if (s_record_count < RECORD_MAX_SAMPLES)
        {
            const int32_t fg = BL48250::get_instance().pulse_counts()[s_record_wheel];
            RecordSample & sample = s_record_buffer[s_record_count];
            const int64_t delta = now_us - s_record_last_us;
            sample.delta_us =
              static_cast<uint16_t>(delta > 65535 ? 65535 : (delta < 0 ? 0 : delta));
            sample.duty = s_raw_duty[s_record_wheel];
            sample.angle = -1;
            if (s_record_i2c)
            {
                uint16_t angle = 0;
                if (AS5600::read_raw_angle(&angle) == ESP_OK)
                    sample.angle = static_cast<int16_t>(angle);
            }
            const int32_t fg_delta = fg - s_record_last_fg;
            sample.fg_delta =
              static_cast<uint16_t>(fg_delta > 65535 ? 65535 : (fg_delta < 0 ? 0 : fg_delta));
            s_record_last_fg = fg;
            s_record_last_us = now_us;
            ++s_record_count;
        }
        // Advance one nominal interval per sample: a late wakeup must not
        // shorten the following spacing.
        s_record_next_us += static_cast<int64_t>(s_record_interval_us);
        if (now_us - s_record_start_us >= static_cast<int64_t>(s_record_duration_us) ||
            s_record_count >= RECORD_MAX_SAMPLES)
        {
            esp_timer_stop(s_record_timer);
            s_record_active = false;
            s_record_done = true;
            timeline_cancel();
            stop_raw_outputs();
        }
    }
}

static void record_start(uint32_t interval_us, uint32_t duration_ms, int wheel, bool use_i2c)
{
    s_record_count = 0;
    s_record_wheel = static_cast<uint8_t>(wheel);
    s_record_i2c = use_i2c;
    s_record_interval_us = interval_us;
    s_record_duration_us = duration_ms * 1000U;
    s_record_start_us = esp_timer_get_time();
    s_record_next_us = s_record_start_us + static_cast<int64_t>(interval_us);
    s_record_last_us = s_record_start_us;
    s_record_last_fg = BL48250::get_instance().pulse_counts()[s_record_wheel];
    s_record_done = false;
    s_record_active = true;
    esp_timer_stop(s_record_timer);
    esp_timer_start_periodic(s_record_timer, interval_us);
}

static void record_cancel()
{
    if (s_record_active)
        esp_timer_stop(s_record_timer);
    s_record_active = false;
    s_record_done = false;
}

static void service_recorder()
{
    if (!s_record_done)
        return;
    s_record_done = false;
    printf("REC BEGIN %u %u %d\n",
           static_cast<unsigned>(s_record_count),
           static_cast<unsigned>(s_record_wheel),
           s_record_i2c ? 1 : 0);
    uint32_t time_us = 0;
    int32_t pulse_count = 0;
    for (size_t index = 0; index < s_record_count; ++index)
    {
        const RecordSample & sample = s_record_buffer[index];
        time_us += sample.delta_us;
        pulse_count += sample.fg_delta;
        printf("%lu,%ld,%d,%d\n",
               static_cast<unsigned long>(time_us),
               static_cast<long>(pulse_count),
               static_cast<int>(sample.duty),
               static_cast<int>(sample.angle));
    }
    printf("REC END\n");
    // The dump blocks the command loop: do not let the dead-man fire after it.
    s_last_command_us = esp_timer_get_time();
}

// -- Command helpers -----------------------------------------------------

// Parses up to count comma-separated integers, returns how many were read.
static int parse_ints(char * text, long * values, int count)
{
    char * endptr = nullptr;
    int parsed = 0;
    while (parsed < count)
    {
        values[parsed] = strtol(text, &endptr, 10);
        if (endptr == text)
            break;
        ++parsed;
        text = endptr;
        if (*text != ',')
            break;
        ++text;
    }
    return parsed;
}

// Prints one line per wheel: estimator speed, FG pulse speed and count, raw ADC
// value and the raw duty in effect, without touching the outputs.
static void print_speed_report()
{
    const auto pulse_counts = BL48250::get_instance().pulse_counts();
    const auto velocities = WheelStateEstimator::get_instance().get_smoothed_velocity_rad_s();
    const auto angles = WheelStateEstimator::get_instance().get_measured_angle_rad();
    const auto raw_values = WheelStateEstimator::get_instance().get_raw_values();
    const auto analog_map = WheelStateEstimator::get_instance().get_analog_mapping();
    printf("V BEGIN\n");
    printf("V t=%lld manual=%d timeline=%d record=%d evt=%u/%u wakeups=%lu writes=%lu\n",
           static_cast<long long>(esp_timer_get_time()),
           s_manual_mode ? 1 : 0,
           s_timeline_active ? 1 : 0,
           s_record_active ? 1 : 0,
           static_cast<unsigned>(s_timeline_index),
           static_cast<unsigned>(s_timeline_count),
           static_cast<unsigned long>(s_player_wakeups),
           static_cast<unsigned long>(s_player_writes));
    for (size_t wheel = 0; wheel < 4; ++wheel)
    {
        float pulse_rad_s = 0.0f;
        const bool pulse_ok = BL48250::get_instance().pulse_velocity_rad_s(wheel, &pulse_rad_s);
        printf("V M%u rpm=%8.2f pulse_rpm=%8.2f angle_deg=%8.2f adc=%5u slot=%u fg=%8ld duty=%5d\n",
               static_cast<unsigned>(wheel + 1),
               -velocities[wheel] * RAD_S_TO_RPM,
               pulse_ok ? pulse_rad_s * RAD_S_TO_RPM : 0.0f,
               angles[wheel] * (180.0f / M_PI),
               static_cast<unsigned>(raw_values[analog_map[wheel]]),
               static_cast<unsigned>(analog_map[wheel]),
               static_cast<long>(pulse_counts[wheel]),
               static_cast<int>(s_raw_duty[wheel]));
    }
    printf("V END\n");
}

// -- High-rate speed control (1 kHz bench loop) --------------------------
// One wheel is switched between a drive duty and a brake duty by a speed
// hysteresis updated every millisecond (CONFIG_FREERTOS_HZ=1000 in this test
// environment only). The measured speed comes from the estimator: the raw
// filtered angle over a short own window, the EKF velocity or the smoothed
// velocity. The result is reported as the mean speed in 5/10/20 ms windows,
// which is the metric used for the low-speed requirement instead of the raw
// mechanical ripple.
static constexpr size_t HIGH_RATE_MAX_SERIES = 4096;
struct HighRateSample
{
    uint16_t delta_us;
    int16_t speed_rpm_x10;
    int16_t duty;
};
struct HighRateWindow
{
    uint32_t count;
    double sum;
    double sumsq;
    float min;
    float max;
    uint32_t outside;
};
static constexpr size_t HIGH_RATE_WINDOW_MAX = 64;
static constexpr size_t HIGH_RATE_WINDOW_COUNT = 3;
static constexpr uint32_t HIGH_RATE_WINDOW_US[HIGH_RATE_WINDOW_COUNT] = {5000, 10000, 20000};

static HighRateSample s_high_rate_series[HIGH_RATE_MAX_SERIES];
static size_t s_high_rate_series_count = 0;
static TaskHandle_t s_high_rate_task = nullptr;
static volatile bool s_high_rate_active = false;
static volatile bool s_high_rate_done = false;
static uint8_t s_hr_wheel = 0;
static float s_hr_target_rpm = 10.0f;
static float s_hr_band_rpm = 1.0f;
static int16_t s_hr_drive_duty = 400;
static int16_t s_hr_brake_duty = 0;
static uint32_t s_hr_duration_us = 5000000;
static uint8_t s_hr_source = 0;  // 0 = own angle window, 1 = EKF, 2 = smoothed
static uint8_t s_hr_window = 8;  // ticks in the own angle window

// PI duty mode: modifies the applied duty continuously around a center value.
static volatile bool s_pi_active = false;
static volatile bool s_pi_done = false;
static uint8_t s_pi_wheel = 0;
static float s_pi_target_rpm = 10.0f;
static float s_pi_kp = 5.0f;       // duty per RPM
static float s_pi_ki = 50.0f;      // duty per RPM per second
static int16_t s_pi_center = 340;  // center duty
static uint32_t s_pi_duration_us = 5000000;
static uint8_t s_pi_source = 3;
static uint8_t s_pi_window = 8;
// 0 = continuous PI around the center, 1 = open-loop duty pattern (high for
// on_ms ticks, low for the rest of period_ms) on the 1 kHz tick.
static uint8_t s_pi_mode = 0;
static int16_t s_pi_high = 400;
static int16_t s_pi_low = 0;
static uint16_t s_pi_on_ms = 1;
static uint16_t s_pi_period_ms = 5;
// Mode 2: one start-up kick (magnitude with the pattern direction) before the
// duty pattern, without stopping the wheel in between.
static int16_t s_pi_kick_duty = 400;
static uint16_t s_pi_kick_ms = 150;

static void high_rate_finish_window(HighRateWindow * window, float mean_rpm, float target_rpm)
{
    ++window->count;
    window->sum += mean_rpm;
    window->sumsq += static_cast<double>(mean_rpm) * mean_rpm;
    if (mean_rpm < window->min)
        window->min = mean_rpm;
    if (mean_rpm > window->max)
        window->max = mean_rpm;
    const float magnitude = std::fabs(mean_rpm);
    if (magnitude < 0.9f * target_rpm || magnitude > 1.1f * target_rpm)
        ++window->outside;
}

static void run_high_rate()
{
    auto & estimator = WheelStateEstimator::get_instance();
    const int wheel = s_hr_wheel;
    const int8_t direction = (s_hr_target_rpm >= 0.0f) ? 1 : -1;
    const float target_rpm = std::fabs(s_hr_target_rpm);
    const float band_rpm = s_hr_band_rpm;
    const int drive_magnitude =
      s_hr_drive_duty < 0 ? -static_cast<int>(s_hr_drive_duty) : static_cast<int>(s_hr_drive_duty);
    const int brake_magnitude =
      s_hr_brake_duty < 0 ? -static_cast<int>(s_hr_brake_duty) : static_cast<int>(s_hr_brake_duty);
    // brake_duty >= 0 is applied in the drive direction (reduced torque);
    // a negative brake_duty is applied opposite to the drive (active brake).
    const int8_t brake_direction = (s_hr_brake_duty >= 0) ? direction : -direction;
    const uint8_t source = s_hr_source;
    const uint8_t window_ticks = s_hr_window;

    struct AngleSample
    {
        float angle;
        int64_t time_us;
    };
    AngleSample history[HIGH_RATE_WINDOW_MAX] = {};
    size_t history_count = 0;
    size_t history_index = 0;
    bool i2c_started = false;
    float i2c_previous = 0.0f;
    float i2c_unwrapped = 0.0f;
    uint8_t fast_ticks = 0;

    s_high_rate_series_count = 0;
    const int64_t start_us = esp_timer_get_time();
    int64_t last_series_us = start_us;
    uint32_t ticks = 0;
    uint32_t drive_ticks = 0;
    uint32_t commutations = 0;
    bool drive_on = true;
    bool safety = false;
    double speed_sum = 0.0;
    double speed_sumsq = 0.0;
    float speed_min = 1e9f;
    float speed_max = -1e9f;
    HighRateWindow windows[HIGH_RATE_WINDOW_COUNT];
    float window_sum[HIGH_RATE_WINDOW_COUNT] = {};
    uint32_t window_samples[HIGH_RATE_WINDOW_COUNT] = {};
    int64_t window_start_us[HIGH_RATE_WINDOW_COUNT] = {};
    for (size_t index = 0; index < HIGH_RATE_WINDOW_COUNT; ++index)
    {
        windows[index] = {0, 0.0, 0.0, 1e9f, -1e9f, 0};
    }

    while (s_high_rate_active &&
           esp_timer_get_time() - start_us < static_cast<int64_t>(s_hr_duration_us))
    {
        const int64_t now_us = esp_timer_get_time();
        float speed_rpm = 0.0f;
        if (source == 0)
        {
            const float angle = estimator.get_filtered_angle_rad()[wheel];
            const size_t slot = history_index;
            history[slot] = {angle, now_us};
            history_index = (history_index + 1) % window_ticks;
            if (history_count < window_ticks)
                ++history_count;
            if (history_count == window_ticks)
            {
                // The ring position that will be overwritten next holds the
                // oldest sample of the window.
                const AngleSample & oldest = history[history_index];
                const float dt_s = static_cast<float>(now_us - oldest.time_us) / 1000000.0f;
                if (dt_s > 1e-6f)
                    speed_rpm = -(angle - oldest.angle) / dt_s * RAD_S_TO_RPM;
            }
        }
        else if (source == 1)
        {
            speed_rpm = -estimator.get_filtered_velocity_rad_s()[wheel] * RAD_S_TO_RPM;
        }
        else if (source == 2)
        {
            speed_rpm = -estimator.get_smoothed_velocity_rad_s()[wheel] * RAD_S_TO_RPM;
        }
        else
        {
            // Own window over the AS5600 raw angle read through the focused mux.
            // The estimator getters have no invalid-sample flag (they return
            // zeros when the mutex times out) and their filtering adds lag the
            // 1 kHz loop cannot afford; one I2C read per tick is ~0.3 ms.
            uint16_t raw_angle = 0;
            if (AS5600::read_raw_angle(&raw_angle) == ESP_OK)
            {
                const float angle =
                  static_cast<float>(raw_angle) * (2.0f * M_PI / 4096.0f);  // motor shaft
                if (!i2c_started)
                {
                    i2c_previous = angle;
                    i2c_unwrapped = 0.0f;
                    i2c_started = true;
                }
                float delta = angle - i2c_previous;
                if (delta > M_PI)
                    delta -= 2.0f * M_PI;
                else if (delta < -M_PI)
                    delta += 2.0f * M_PI;
                i2c_unwrapped += delta;
                i2c_previous = angle;
                history[history_index] = {i2c_unwrapped, now_us};
                history_index = (history_index + 1) % window_ticks;
                if (history_count < window_ticks)
                    ++history_count;
                if (history_count == window_ticks)
                {
                    const AngleSample & oldest = history[history_index];
                    const float dt_s = static_cast<float>(now_us - oldest.time_us) / 1000000.0f;
                    if (dt_s > 1e-6f)
                        speed_rpm = -(i2c_unwrapped - oldest.angle) / dt_s /
                                    config::driver::WHEEL_REDUCTION * RAD_S_TO_RPM;
                }
            }
        }

        if (std::fabs(speed_rpm) > 150.0f)
        {
            if (++fast_ticks >= 3)
            {
                safety = true;
                break;
            }
        }
        else
        {
            fast_ticks = 0;
        }

        const float magnitude = std::fabs(speed_rpm);
        const bool previous_drive = drive_on;
        if (magnitude < target_rpm - band_rpm)
            drive_on = true;
        else if (magnitude > target_rpm + band_rpm)
            drive_on = false;
        if (drive_on != previous_drive)
            ++commutations;
        const int16_t output = drive_on ? static_cast<int16_t>(direction * drive_magnitude)
                                        : static_cast<int16_t>(brake_direction * brake_magnitude);
        s_raw_duty[wheel] = output;
        write_raw_duties();
        ++ticks;
        if (drive_on)
            ++drive_ticks;

        speed_sum += speed_rpm;
        speed_sumsq += static_cast<double>(speed_rpm) * speed_rpm;
        if (speed_rpm < speed_min)
            speed_min = speed_rpm;
        if (speed_rpm > speed_max)
            speed_max = speed_rpm;
        for (size_t index = 0; index < HIGH_RATE_WINDOW_COUNT; ++index)
        {
            if (window_samples[index] == 0)
                window_start_us[index] = now_us;
            window_sum[index] += speed_rpm;
            ++window_samples[index];
            if (now_us - window_start_us[index] >= static_cast<int64_t>(HIGH_RATE_WINDOW_US[index]))
            {
                high_rate_finish_window(
                  &windows[index],
                  window_sum[index] / static_cast<float>(window_samples[index]),
                  target_rpm);
                window_sum[index] = 0.0f;
                window_samples[index] = 0;
            }
        }
        // One series sample per control tick: the analysis needs the real 1 kHz
        // trajectory of the measured speed and the applied duty.
        if (s_high_rate_series_count < HIGH_RATE_MAX_SERIES && now_us - last_series_us >= 1000)
        {
            HighRateSample & sample = s_high_rate_series[s_high_rate_series_count];
            sample.delta_us = static_cast<uint16_t>(now_us - last_series_us);
            sample.speed_rpm_x10 = static_cast<int16_t>(std::lround(speed_rpm * 10.0f));
            sample.duty = output;
            ++s_high_rate_series_count;
            last_series_us = now_us;
        }
        vTaskDelay(1);  // 1 ms tick in this environment -> ~1 kHz control loop
    }

    s_raw_duty[wheel] = 0;
    write_raw_duties();
    s_high_rate_active = false;
    s_high_rate_done = true;

    const double mean = ticks > 0 ? speed_sum / ticks : 0.0;
    double variance = ticks > 0 ? speed_sumsq / ticks - mean * mean : 0.0;
    if (variance < 0.0)
        variance = 0.0;
    printf(
      "HBEGIN wheel=%d target=%.2f band=%.2f drive=%d brake=%d duration_ms=%lu source=%u "
      "window=%u\n",
      wheel + 1,
      static_cast<double>(s_hr_target_rpm),
      static_cast<double>(band_rpm),
      static_cast<int>(s_hr_drive_duty),
      static_cast<int>(s_hr_brake_duty),
      static_cast<unsigned long>(s_hr_duration_us / 1000U),
      static_cast<unsigned>(source),
      static_cast<unsigned>(window_ticks));
    printf("HSUM ticks=%lu actual_ms=%.1f drive_frac=%.3f commutations=%lu safety=%d heap=%u\n",
           static_cast<unsigned long>(ticks),
           static_cast<double>(esp_timer_get_time() - start_us) / 1000.0,
           ticks > 0 ? static_cast<double>(drive_ticks) / ticks : 0.0,
           static_cast<unsigned long>(commutations),
           safety ? 1 : 0,
           static_cast<unsigned>(esp_get_free_heap_size()));
    printf("HSTAT mean=%.2f std=%.2f min=%.2f max=%.2f\n",
           mean,
           std::sqrt(variance),
           speed_min > 1e8f ? 0.0 : static_cast<double>(speed_min),
           speed_max < -1e8f ? 0.0 : static_cast<double>(speed_max));
    for (size_t index = 0; index < HIGH_RATE_WINDOW_COUNT; ++index)
    {
        const HighRateWindow & window = windows[index];
        const double window_mean = window.count > 0 ? window.sum / window.count : 0.0;
        double window_variance =
          window.count > 0 ? window.sumsq / window.count - window_mean * window_mean : 0.0;
        if (window_variance < 0.0)
            window_variance = 0.0;
        printf(
          "HWIN win_ms=%lu windows=%lu mean=%.3f std=%.3f min=%.3f max=%.3f "
          "outside_pct=%.1f\n",
          static_cast<unsigned long>(HIGH_RATE_WINDOW_US[index] / 1000U),
          static_cast<unsigned long>(window.count),
          window_mean,
          std::sqrt(window_variance),
          window.count > 0 ? static_cast<double>(window.min) : 0.0,
          window.count > 0 ? static_cast<double>(window.max) : 0.0,
          window.count > 0 ? 100.0 * window.outside / window.count : 0.0);
    }
    printf("HSERIES count=%u\n", static_cast<unsigned>(s_high_rate_series_count));
    uint32_t series_time_us = 0;
    for (size_t index = 0; index < s_high_rate_series_count; ++index)
    {
        const HighRateSample & sample = s_high_rate_series[index];
        series_time_us += sample.delta_us;
        printf("%lu,%d,%d\n",
               static_cast<unsigned long>(series_time_us),
               static_cast<int>(sample.speed_rpm_x10),
               static_cast<int>(sample.duty));
    }
    printf("HEND\n");
    s_last_command_us = esp_timer_get_time();
}

static void run_pi()
{
    auto & estimator = WheelStateEstimator::get_instance();
    const int wheel = s_pi_wheel;
    const int8_t direction = (s_pi_target_rpm >= 0.0f) ? 1 : -1;
    const float target_rpm = std::fabs(s_pi_target_rpm);
    const float kp = s_pi_kp;
    const float ki = s_pi_ki;
    const float center = static_cast<float>(s_pi_center);
    const uint8_t source = s_pi_source;
    const uint8_t window_ticks = s_pi_window;

    struct AngleSample
    {
        float angle;
        int64_t time_us;
    };
    AngleSample history[HIGH_RATE_WINDOW_MAX] = {};
    size_t history_count = 0;
    size_t history_index = 0;
    bool i2c_started = false;
    float i2c_previous = 0.0f;
    float i2c_unwrapped = 0.0f;

    s_high_rate_series_count = 0;
    const int64_t start_us = esp_timer_get_time();
    const uint32_t series_divider =
      s_pi_duration_us > 8000000U ? 8U : (s_pi_duration_us > 4000000U ? 4U : 1U);
    int64_t last_series_us = start_us;
    int64_t last_tick_us = start_us;
    uint32_t ticks = 0;
    uint32_t safety_ticks = 0;
    bool safety = false;
    double speed_sum = 0.0;
    double speed_sumsq = 0.0;
    float speed_min = 1e9f;
    float speed_max = -1e9f;
    double duty_sum = 0.0;
    float duty_min = 1e9f;
    float duty_max = -1e9f;
    float integral = 0.0f;
    HighRateWindow windows[HIGH_RATE_WINDOW_COUNT];
    float window_sum[HIGH_RATE_WINDOW_COUNT] = {};
    uint32_t window_samples[HIGH_RATE_WINDOW_COUNT] = {};
    int64_t window_start_us[HIGH_RATE_WINDOW_COUNT] = {};
    for (size_t index = 0; index < HIGH_RATE_WINDOW_COUNT; ++index)
        windows[index] = {0, 0.0, 0.0, 1e9f, -1e9f, 0};

    while (s_pi_active && esp_timer_get_time() - start_us < static_cast<int64_t>(s_pi_duration_us))
    {
        const int64_t now_us = esp_timer_get_time();
        float dt_s = static_cast<float>(now_us - last_tick_us) / 1000000.0f;
        if (dt_s < 1e-5f)
            dt_s = 1e-3f;
        last_tick_us = now_us;

        float speed_rpm = 0.0f;
        if (source == 0)
        {
            const float angle = estimator.get_filtered_angle_rad()[wheel];
            history[history_index] = {angle, now_us};
            history_index = (history_index + 1) % window_ticks;
            if (history_count < window_ticks)
                ++history_count;
            if (history_count == window_ticks)
            {
                const AngleSample & oldest = history[history_index];
                const float dt = static_cast<float>(now_us - oldest.time_us) / 1000000.0f;
                if (dt > 1e-6f)
                    speed_rpm = -(angle - oldest.angle) / dt * RAD_S_TO_RPM;
            }
        }
        else if (source == 1)
        {
            speed_rpm = -estimator.get_filtered_velocity_rad_s()[wheel] * RAD_S_TO_RPM;
        }
        else if (source == 2)
        {
            speed_rpm = -estimator.get_smoothed_velocity_rad_s()[wheel] * RAD_S_TO_RPM;
        }
        else
        {
            uint16_t raw_angle = 0;
            if (AS5600::read_raw_angle(&raw_angle) == ESP_OK)
            {
                const float angle = static_cast<float>(raw_angle) * (2.0f * M_PI / 4096.0f);
                if (!i2c_started)
                {
                    i2c_previous = angle;
                    i2c_unwrapped = 0.0f;
                    i2c_started = true;
                }
                float delta = angle - i2c_previous;
                if (delta > M_PI)
                    delta -= 2.0f * M_PI;
                else if (delta < -M_PI)
                    delta += 2.0f * M_PI;
                i2c_unwrapped += delta;
                i2c_previous = angle;
                history[history_index] = {i2c_unwrapped, now_us};
                history_index = (history_index + 1) % window_ticks;
                if (history_count < window_ticks)
                    ++history_count;
                if (history_count == window_ticks)
                {
                    const AngleSample & oldest = history[history_index];
                    const float dt = static_cast<float>(now_us - oldest.time_us) / 1000000.0f;
                    if (dt > 1e-6f)
                        speed_rpm = -(i2c_unwrapped - oldest.angle) / dt /
                                    config::driver::WHEEL_REDUCTION * RAD_S_TO_RPM;
                }
            }
        }

        if (std::fabs(speed_rpm) > 150.0f)
        {
            if (++safety_ticks >= 3)
            {
                safety = true;
                break;
            }
        }
        else
        {
            safety_ticks = 0;
        }

        float duty = 0.0f;
        const int64_t elapsed_us = now_us - start_us;
        if (s_pi_mode == 2 && elapsed_us < static_cast<int64_t>(s_pi_kick_ms) * 1000)
        {
            // Start-up kick: drive long enough to break static friction, then
            // fall straight into the duty pattern (no stop in between).
            duty = static_cast<float>(s_pi_kick_duty < 0 ? -s_pi_kick_duty : s_pi_kick_duty);
        }
        else if (s_pi_mode == 1 || s_pi_mode == 2)
        {
            // Open-loop duty pattern: high on-time, low for the rest of the
            // period, both counted on the 1 ms control tick.
            const uint32_t phase = ticks % static_cast<uint32_t>(s_pi_period_ms);
            duty = phase < static_cast<uint32_t>(s_pi_on_ms) ? static_cast<float>(s_pi_high)
                                                             : static_cast<float>(s_pi_low);
        }
        else
        {
            const float signed_speed = speed_rpm * static_cast<float>(direction);
            const float error = target_rpm - signed_speed;
            integral += ki * error * dt_s;
            if (integral > 400.0f)
                integral = 400.0f;
            else if (integral < -400.0f)
                integral = -400.0f;
            duty = center + kp * error + integral;
            if (duty > 1023.0f)
                duty = 1023.0f;
            else if (duty < 0.0f)
                duty = 0.0f;
        }
        const int16_t output =
          static_cast<int16_t>(direction * static_cast<int>(std::lround(duty)));
        s_raw_duty[wheel] = output;
        write_raw_duties();
        ++ticks;

        speed_sum += speed_rpm;
        speed_sumsq += static_cast<double>(speed_rpm) * speed_rpm;
        if (speed_rpm < speed_min)
            speed_min = speed_rpm;
        if (speed_rpm > speed_max)
            speed_max = speed_rpm;
        duty_sum += duty;
        if (duty < duty_min)
            duty_min = duty;
        if (duty > duty_max)
            duty_max = duty;
        for (size_t index = 0; index < HIGH_RATE_WINDOW_COUNT; ++index)
        {
            if (window_samples[index] == 0)
                window_start_us[index] = now_us;
            window_sum[index] += speed_rpm;
            ++window_samples[index];
            if (now_us - window_start_us[index] >= static_cast<int64_t>(HIGH_RATE_WINDOW_US[index]))
            {
                high_rate_finish_window(
                  &windows[index],
                  window_sum[index] / static_cast<float>(window_samples[index]),
                  target_rpm);
                window_sum[index] = 0.0f;
                window_samples[index] = 0;
            }
        }
        if (s_high_rate_series_count < HIGH_RATE_MAX_SERIES && now_us - last_series_us >= 1000 &&
            (ticks % series_divider) == 0)
        {
            HighRateSample & sample = s_high_rate_series[s_high_rate_series_count];
            sample.delta_us = static_cast<uint16_t>(now_us - last_series_us);
            sample.speed_rpm_x10 = static_cast<int16_t>(std::lround(speed_rpm * 10.0f));
            sample.duty = output;
            ++s_high_rate_series_count;
            last_series_us = now_us;
        }
        vTaskDelay(1);
    }

    s_raw_duty[wheel] = 0;
    write_raw_duties();
    s_pi_active = false;
    s_pi_done = true;

    const double mean = ticks > 0 ? speed_sum / ticks : 0.0;
    double variance = ticks > 0 ? speed_sumsq / ticks - mean * mean : 0.0;
    if (variance < 0.0)
        variance = 0.0;
    printf(
      "HBEGIN wheel=%d target=%.2f mode=%s kp=%.3f ki=%.3f center=%d high=%d low=%d "
      "on_ms=%u period_ms=%u kick=%d kick_ms=%u duration_ms=%lu source=%u window=%u\n",
      wheel + 1,
      static_cast<double>(s_pi_target_rpm),
      s_pi_mode == 2 ? "kick_dither" : (s_pi_mode == 1 ? "dither" : "pi"),
      static_cast<double>(kp),
      static_cast<double>(ki),
      static_cast<int>(s_pi_center),
      static_cast<int>(s_pi_high),
      static_cast<int>(s_pi_low),
      static_cast<unsigned>(s_pi_on_ms),
      static_cast<unsigned>(s_pi_period_ms),
      static_cast<int>(s_pi_kick_duty),
      static_cast<unsigned>(s_pi_kick_ms),
      static_cast<unsigned long>(s_pi_duration_us / 1000U),
      static_cast<unsigned>(source),
      static_cast<unsigned>(window_ticks));
    printf(
      "HSUM ticks=%lu actual_ms=%.1f duty_mean=%.1f duty_min=%.1f duty_max=%.1f "
      "safety=%d heap=%u\n",
      static_cast<unsigned long>(ticks),
      static_cast<double>(esp_timer_get_time() - start_us) / 1000.0,
      ticks > 0 ? duty_sum / ticks : 0.0,
      ticks > 0 ? static_cast<double>(duty_min) : 0.0,
      ticks > 0 ? static_cast<double>(duty_max) : 0.0,
      safety ? 1 : 0,
      static_cast<unsigned>(esp_get_free_heap_size()));
    printf("HSTAT mean=%.2f std=%.2f min=%.2f max=%.2f\n",
           mean,
           std::sqrt(variance),
           speed_min > 1e8f ? 0.0 : static_cast<double>(speed_min),
           speed_max < -1e8f ? 0.0 : static_cast<double>(speed_max));
    for (size_t index = 0; index < HIGH_RATE_WINDOW_COUNT; ++index)
    {
        const HighRateWindow & window = windows[index];
        const double window_mean = window.count > 0 ? window.sum / window.count : 0.0;
        double window_variance =
          window.count > 0 ? window.sumsq / window.count - window_mean * window_mean : 0.0;
        if (window_variance < 0.0)
            window_variance = 0.0;
        printf(
          "HWIN win_ms=%lu windows=%lu mean=%.3f std=%.3f min=%.3f max=%.3f "
          "outside_pct=%.1f\n",
          static_cast<unsigned long>(HIGH_RATE_WINDOW_US[index] / 1000U),
          static_cast<unsigned long>(window.count),
          window_mean,
          std::sqrt(window_variance),
          window.count > 0 ? static_cast<double>(window.min) : 0.0,
          window.count > 0 ? static_cast<double>(window.max) : 0.0,
          window.count > 0 ? 100.0 * window.outside / window.count : 0.0);
    }
    printf("HSERIES count=%u\n", static_cast<unsigned>(s_high_rate_series_count));
    uint32_t series_time_us = 0;
    for (size_t index = 0; index < s_high_rate_series_count; ++index)
    {
        const HighRateSample & sample = s_high_rate_series[index];
        series_time_us += sample.delta_us;
        printf("%lu,%d,%d\n",
               static_cast<unsigned long>(series_time_us),
               static_cast<int>(sample.speed_rpm_x10),
               static_cast<int>(sample.duty));
    }
    printf("HEND\n");
    s_last_command_us = esp_timer_get_time();
}

static void high_rate_task(void * arg)
{
    (void)arg;
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (s_high_rate_active)
            run_high_rate();
        else if (s_pi_active)
            run_pi();
        else
            s_high_rate_done = true;
    }
}

// Serial buffer
static constexpr int SERIAL_BUFFER_SIZE = 64;
static char serial_buffer[SERIAL_BUFFER_SIZE];
static uint8_t serial_index = 0;
static bool new_command = false;

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
            if (serial_index > 0)
            {
                serial_buffer[serial_index] = '\0';
                new_command = true;
                serial_index = 0;
                return;
            }
        }
        else if (serial_index < (SERIAL_BUFFER_SIZE - 1))
            serial_buffer[serial_index++] = (char)c;
        else
            serial_index = 0;
    }
}

static bool process_serial_command(WheelController & wc)
{
    if (!new_command)
        return false;

    new_command = false;
    char * pointer = serial_buffer;

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
        set_all_gains(wc, kp, ki, kd);
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
            set_all_gains(wc, kp, ki, kd);
            printf("ACK S: per-wheel PID set\n");
        }
        return true;
    }

    // 0 — stop all wheels (also drops any raw harness output)
    if (*pointer == '0')
    {
        timeline_cancel();
        record_cancel();
        s_high_rate_active = false;
        s_pi_active = false;
        stop_raw_outputs();
        wc.set_target_velocities({0.0f, 0.0f, 0.0f, 0.0f});
        printf("ACK 0: all wheels stopped\n");
        return true;
    }

    // D — save current PID gains to NVS
    if (*pointer == 'D')
    {
        wc.save_all_parameters();
        printf("ACK D: control parameters saved to NVS\n");
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
            timeline_cancel();
            record_cancel();
            s_high_rate_active = false;
            s_pi_active = false;
            invalidate_raw_outputs_cache();
            stop_raw_outputs();
            s_last_command_us = esp_timer_get_time();
            s_manual_mode = true;
            printf("ACK M: manual mode ON - control task paused, motors stopped\n");
        }
        else
        {
            timeline_cancel();
            record_cancel();
            s_high_rate_active = false;
            stop_raw_outputs();
            s_cd4052b.disable();
            s_i2c_wheel = -1;
            wc.init();
            s_last_command_us = 0;
            s_manual_mode = false;
            printf("ACK M: manual mode OFF - control task running\n");
        }
        return true;
    }

    // G<hex> — write a raw byte to the 74HC595 for probing the outputs with a
    // meter (manual mode only). Bit map: 0..3 = DIR FL/BL/BR/FR, 4/5 = mux A/B,
    // 6 = mux inhibit, 7 = nRF24 CE (CERF24). 74HC595 pins: bit n -> Qn
    // (Q0 = pin 15 ...).
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

    // W<duty> — raw signed duty on all wheels (manual mode only)
    // W<fl>,<bl>,<br>,<fr> — raw signed duty per wheel
    if (*pointer == 'W')
    {
        ++pointer;
        if (!s_manual_mode)
        {
            printf("ERR W: send M first to pause the control task\n");
            return true;
        }
        if (s_timeline_active)
        {
            printf("ERR W: timeline in progress\n");
            return true;
        }
        long values[4] = {0, 0, 0, 0};
        const int parsed = parse_ints(pointer, values, 4);
        if (parsed == 0)
        {
            printf("ERR W: expected W<duty> or W<fl>,<bl>,<br>,<fr>\n");
            return true;
        }
        if (parsed == 1)
        {
            values[1] = values[0];
            values[2] = values[0];
            values[3] = values[0];
        }
        else if (parsed != 4)
        {
            printf("ERR W: expected 1 or 4 duties, got %d\n", parsed);
            return true;
        }
        for (int wheel = 0; wheel < 4; ++wheel)
        {
            if (values[wheel] < -1023 || values[wheel] > 1023)
            {
                printf("ERR W: duty %ld out of range -1023..1023\n", values[wheel]);
                return true;
            }
            s_raw_duty[wheel] = static_cast<int16_t>(values[wheel]);
        }
        write_raw_duties();
        printf("ACK W: raw duties %d %d %d %d\n",
               static_cast<int>(s_raw_duty[0]),
               static_cast<int>(s_raw_duty[1]),
               static_cast<int>(s_raw_duty[2]),
               static_cast<int>(s_raw_duty[3]));
        return true;
    }

    // Q<wheel>,<duty>,<on_ms>,<period_ms>,<cycles> — pulse train, ends stopped
    if (*pointer == 'Q')
    {
        ++pointer;
        if (!s_manual_mode)
        {
            printf("ERR Q: send M first to pause the control task\n");
            return true;
        }
        if (s_timeline_active)
        {
            printf("ERR Q: timeline in progress\n");
            return true;
        }
        long values[5] = {0, 0, 0, 0, 0};
        if (parse_ints(pointer, values, 5) != 5)
        {
            printf("ERR Q: expected Q<wheel>,<duty>,<on_ms>,<period_ms>,<cycles>\n");
            return true;
        }
        const long wheel = values[0];
        const long duty = values[1];
        const long on_ms = values[2];
        const long period_ms = values[3];
        const long cycles = values[4];
        if (wheel < 0 || wheel > 3 || duty == 0 || duty < -1023 || duty > 1023 || on_ms < 1 ||
            period_ms < 2 || on_ms >= period_ms || period_ms > 5000 || cycles < 1 ||
            cycles * 2 > static_cast<long>(TIMELINE_MAX_EVENTS))
        {
            printf(
              "ERR Q: bad parameters (wheel 0-3, duty +-1..1023, 1<=on<period<=5000 ms, "
              "1..%u cycles)\n",
              static_cast<unsigned>(TIMELINE_MAX_EVENTS / 2));
            return true;
        }
        s_timeline_count = 0;
        const int64_t period_us = static_cast<int64_t>(period_ms) * 1000;
        const int64_t on_us = static_cast<int64_t>(on_ms) * 1000;
        bool built = true;
        for (long cycle = 0; cycle < cycles; ++cycle)
        {
            const int64_t cycle_start_us = cycle * period_us;
            built =
              built &&
              timeline_add(cycle_start_us, static_cast<int>(wheel), static_cast<int16_t>(duty)) &&
              timeline_add(cycle_start_us + on_us, static_cast<int>(wheel), 0);
        }
        if (!built)
        {
            s_timeline_count = 0;
            printf("ERR Q: timeline too long\n");
            return true;
        }
        printf("ACK Q: wheel %ld duty %ld on %ld ms period %ld ms cycles %ld (%.2f s)\n",
               wheel,
               duty,
               on_ms,
               period_ms,
               cycles,
               static_cast<double>(period_ms) * static_cast<double>(cycles) / 1000.0);
        timeline_start();
        return true;
    }

    // X<wheel>,<duty>,<hold_ms>[,<duty>,<hold_ms>...] — step sequence, up to
    // four segments; the output always returns to zero at the end
    if (*pointer == 'X')
    {
        ++pointer;
        if (!s_manual_mode)
        {
            printf("ERR X: send M first to pause the control task\n");
            return true;
        }
        if (s_timeline_active)
        {
            printf("ERR X: timeline in progress\n");
            return true;
        }
        long values[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        const int parsed = parse_ints(pointer, values, 9);
        if (parsed < 3 || (parsed % 2) == 0)
        {
            printf("ERR X: expected X<wheel>,<duty>,<hold_ms>[,<duty>,<hold_ms>...]\n");
            return true;
        }
        const long wheel = values[0];
        const int pairs = (parsed - 1) / 2;
        if (wheel < 0 || wheel > 3)
        {
            printf("ERR X: wheel %ld out of range 0-3\n", wheel);
            return true;
        }
        s_timeline_count = 0;
        int64_t at_us = 0;
        double total_s = 0.0;
        bool ok = true;
        for (int pair = 0; pair < pairs && ok; ++pair)
        {
            const long duty = values[1 + 2 * pair];
            const long hold_ms = values[2 + 2 * pair];
            if (duty < -1023 || duty > 1023 || hold_ms < 1 || hold_ms > 120000)
            {
                printf("ERR X: segment %d needs duty +-1023 and hold 1..120000 ms\n", pair + 1);
                return true;
            }
            ok = timeline_add(at_us, static_cast<int>(wheel), static_cast<int16_t>(duty));
            at_us += static_cast<int64_t>(hold_ms) * 1000;
            total_s += static_cast<double>(hold_ms) / 1000.0;
        }
        if (!ok || !timeline_add(at_us, static_cast<int>(wheel), 0))
        {
            s_timeline_count = 0;
            printf("ERR X: timeline too long\n");
            return true;
        }
        printf("ACK X: wheel %ld, %d segments, %.2f s, ends stopped\n", wheel, pairs, total_s);
        timeline_start();
        return true;
    }

    // L<interval_us>,<duration_ms>,<wheel>,<i2c> — fixed-interval CSV record;
    // the recording end always stops every output
    if (*pointer == 'L')
    {
        ++pointer;
        long values[4] = {0, 0, 0, 0};
        if (parse_ints(pointer, values, 4) != 4)
        {
            printf("ERR L: expected L<interval_us>,<duration_ms>,<wheel>,<i2c>\n");
            return true;
        }
        const long interval_us = values[0];
        const long duration_ms = values[1];
        const long wheel = values[2];
        const long use_i2c = values[3];
        if (interval_us < 100 || interval_us > 200000 || duration_ms < 20 || duration_ms > 120000 ||
            wheel < 0 || wheel > 3 || (use_i2c != 0 && use_i2c != 1))
        {
            printf(
              "ERR L: bad parameters (interval 100..200000 us, duration 20..120000 ms, "
              "wheel 0-3, i2c 0/1)\n");
            return true;
        }
        const long samples = (duration_ms * 1000) / interval_us + 1;
        if (samples > static_cast<long>(RECORD_MAX_SAMPLES))
        {
            printf("ERR L: %ld samples > %u, shorten the window or raise the interval\n",
                   samples,
                   static_cast<unsigned>(RECORD_MAX_SAMPLES));
            return true;
        }
        if (s_record_active || s_timeline_active)
        {
            printf("ERR L: recording or timeline in progress\n");
            return true;
        }
        if (use_i2c != 0 && s_i2c_wheel != wheel)
        {
            printf("ERR L: focus the mux on wheel %ld first (C%ld)\n", wheel, wheel);
            return true;
        }
        record_start(static_cast<uint32_t>(interval_us),
                     static_cast<uint32_t>(duration_ms),
                     static_cast<int>(wheel),
                     use_i2c != 0);
        printf("ACK L: %ld samples, %ld ms window, %ld us interval, wheel %ld, i2c %ld\n",
               samples,
               duration_ms,
               interval_us,
               wheel,
               use_i2c);
        return true;
    }

    // C<wheel> — focus the CD4052B I2C mux on one wheel for fast AS5600 reads
    // C-1 — release the mux
    if (*pointer == 'C')
    {
        ++pointer;
        char * endptr = nullptr;
        const long wheel = strtol(pointer, &endptr, 10);
        if (endptr == pointer)
        {
            printf("ERR C: expected C<wheel 0-3> or C-1\n");
            return true;
        }
        if (wheel < 0)
        {
            const esp_err_t err = s_cd4052b.disable();
            s_i2c_wheel = -1;
            printf("ACK C: mux released (%s)\n", esp_err_to_name(err));
            return true;
        }
        if (wheel > 3)
        {
            printf("ERR C: wheel %ld out of range 0-3\n", wheel);
            return true;
        }
        const uint8_t mux_channel =
          WheelStateEstimator::get_instance().get_mux_mapping()[static_cast<size_t>(wheel)];
        if (s_cd4052b.select(mux_channel) != ESP_OK)
        {
            printf("ERR C: mux select ch%u failed\n", static_cast<unsigned>(mux_channel));
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // let the analog switch settle on the bus
        uint16_t angle = 0;
        if (AS5600::read_raw_angle(&angle) != ESP_OK)
        {
            s_i2c_wheel = -1;
            printf("ERR C: AS5600 read failed on mux ch%u (wheel %ld)\n",
                   static_cast<unsigned>(mux_channel),
                   wheel);
            return true;
        }
        s_i2c_wheel = static_cast<int8_t>(wheel);
        printf("ACK C: wheel %ld mux ch%u raw angle %u (%.1f deg motor)\n",
               wheel,
               static_cast<unsigned>(mux_channel),
               static_cast<unsigned>(angle),
               static_cast<double>(angle) * 360.0 / 4096.0);
        return true;
    }

    // H<wheel>,<target_rpm>,<band_rpm>,<drive_duty>,<brake_duty>,<duration_ms>
    //   [,<source>,<window>] — speed hysteresis at ~1 kHz on one wheel
    //   (manual mode). brake_duty >= 0 is applied in the drive direction,
    //   a negative brake_duty is applied opposite (active brake).
    if (*pointer == 'H')
    {
        ++pointer;
        if (!s_manual_mode)
        {
            printf("ERR H: send M first to pause the control task\n");
            return true;
        }
        if (s_timeline_active || s_record_active || s_high_rate_active)
        {
            printf("ERR H: another harness job is active\n");
            return true;
        }
        char * endptr = nullptr;
        const long wheel = strtol(pointer, &endptr, 10);
        if (endptr == pointer || wheel < 0 || wheel > 3)
        {
            printf(
              "ERR H: expected H<wheel 0-3>,<target_rpm>,<band_rpm>,<drive>,<brake>,"
              "<duration_ms>[,<source>,<window>]\n");
            return true;
        }
        pointer = (*endptr == ',') ? endptr + 1 : endptr;
        const float target_rpm = strtof(pointer, &endptr);
        if (endptr == pointer)
        {
            printf("ERR H: missing target RPM\n");
            return true;
        }
        pointer = (*endptr == ',') ? endptr + 1 : endptr;
        const float band_rpm = strtof(pointer, &endptr);
        if (endptr == pointer)
        {
            printf("ERR H: missing band RPM\n");
            return true;
        }
        pointer = (*endptr == ',') ? endptr + 1 : endptr;
        const long drive_duty = strtol(pointer, &endptr, 10);
        if (endptr == pointer)
        {
            printf("ERR H: missing drive duty\n");
            return true;
        }
        pointer = (*endptr == ',') ? endptr + 1 : endptr;
        const long brake_duty = strtol(pointer, &endptr, 10);
        if (endptr == pointer)
        {
            printf("ERR H: missing brake duty\n");
            return true;
        }
        pointer = (*endptr == ',') ? endptr + 1 : endptr;
        const long duration_ms = strtol(pointer, &endptr, 10);
        if (endptr == pointer)
        {
            printf("ERR H: missing duration\n");
            return true;
        }
        long source = 0;
        long window = 8;
        if (*endptr == ',')
        {
            pointer = endptr + 1;
            source = strtol(pointer, &endptr, 10);
            if (*endptr == ',')
            {
                pointer = endptr + 1;
                window = strtol(pointer, &endptr, 10);
            }
        }
        if (target_rpm == 0.0f || std::fabs(target_rpm) > 150.0f || band_rpm <= 0.0f ||
            band_rpm > 20.0f || drive_duty == 0 || drive_duty < -1023 || drive_duty > 1023 ||
            brake_duty < -1023 || brake_duty > 1023 || duration_ms < 200 || duration_ms > 10000 ||
            source < 0 || source > 3 || window < 2 ||
            window > static_cast<long>(HIGH_RATE_WINDOW_MAX))
        {
            printf(
              "ERR H: bad parameters (target +-0.1..150 RPM, band 0.1..20 RPM, drive "
              "+-1..1023, brake +-1023, duration 200..10000 ms, source 0-2, window 2..64)\n");
            return true;
        }
        stop_raw_outputs();
        if (source == 3 && s_i2c_wheel != wheel)
        {
            printf("ERR H: source 3 needs the mux focused on wheel %ld (C%ld)\n", wheel, wheel);
            return true;
        }
        s_hr_wheel = static_cast<uint8_t>(wheel);
        s_hr_target_rpm = target_rpm;
        s_hr_band_rpm = band_rpm;
        s_hr_drive_duty = static_cast<int16_t>(drive_duty);
        s_hr_brake_duty = static_cast<int16_t>(brake_duty);
        s_hr_duration_us = static_cast<uint32_t>(duration_ms) * 1000U;
        s_hr_source = static_cast<uint8_t>(source);
        s_hr_window = static_cast<uint8_t>(window);
        s_high_rate_done = false;
        s_high_rate_active = true;
        s_last_command_us = esp_timer_get_time();
        xTaskNotifyGive(s_high_rate_task);
        printf(
          "ACK H: wheel %ld target %.2f RPM band %.2f drive %ld brake %ld %ld ms source "
          "%ld window %ld\n",
          wheel,
          static_cast<double>(target_rpm),
          static_cast<double>(band_rpm),
          drive_duty,
          brake_duty,
          duration_ms,
          source,
          window);
        return true;
    }

    // J<wheel>,<target_rpm>,<kp>,<ki>,<center_duty>,<duration_ms>[,<source>,<window>]
    //   — continuous PI duty around a center value at ~1 kHz (manual mode)
    if (*pointer == 'J')
    {
        ++pointer;
        if (!s_manual_mode)
        {
            printf("ERR J: send M first to pause the control task\n");
            return true;
        }
        if (s_timeline_active || s_record_active || s_high_rate_active || s_pi_active)
        {
            printf("ERR J: another harness job is active\n");
            return true;
        }
        char * endptr = nullptr;
        const long wheel = strtol(pointer, &endptr, 10);
        if (endptr == pointer || wheel < 0 || wheel > 3)
        {
            printf(
              "ERR J: expected J<wheel 0-3>,<target_rpm>,<kp>,<ki>,<center>,<ms>"
              "[,<source>,<window>]\n");
            return true;
        }
        pointer = (*endptr == ',') ? endptr + 1 : endptr;
        const float target_rpm = strtof(pointer, &endptr);
        if (endptr == pointer)
        {
            printf("ERR J: missing target RPM\n");
            return true;
        }
        pointer = (*endptr == ',') ? endptr + 1 : endptr;
        const float kp = strtof(pointer, &endptr);
        if (endptr == pointer)
        {
            printf("ERR J: missing kp\n");
            return true;
        }
        pointer = (*endptr == ',') ? endptr + 1 : endptr;
        const float ki = strtof(pointer, &endptr);
        if (endptr == pointer)
        {
            printf("ERR J: missing ki\n");
            return true;
        }
        pointer = (*endptr == ',') ? endptr + 1 : endptr;
        const long center = strtol(pointer, &endptr, 10);
        if (endptr == pointer)
        {
            printf("ERR J: missing center duty\n");
            return true;
        }
        pointer = (*endptr == ',') ? endptr + 1 : endptr;
        const long duration_ms = strtol(pointer, &endptr, 10);
        if (endptr == pointer)
        {
            printf("ERR J: missing duration\n");
            return true;
        }
        long source = 3;
        long window = 8;
        if (*endptr == ',')
        {
            pointer = endptr + 1;
            source = strtol(pointer, &endptr, 10);
            if (*endptr == ',')
            {
                pointer = endptr + 1;
                window = strtol(pointer, &endptr, 10);
            }
        }
        if (target_rpm == 0.0f || std::fabs(target_rpm) > 150.0f || kp < 0.0f || kp > 200.0f ||
            ki < 0.0f || ki > 500.0f || center < 0 || center > 1023 || duration_ms < 200 ||
            duration_ms > 10000 || source < 0 || source > 3 || window < 2 ||
            window > static_cast<long>(HIGH_RATE_WINDOW_MAX))
        {
            printf(
              "ERR J: bad parameters (target +-0.1..150 RPM, kp 0..200, ki 0..500, "
              "center 0..1023, duration 200..10000 ms, source 0-3, window 2..64)\n");
            return true;
        }
        if (source == 3 && s_i2c_wheel != wheel)
        {
            printf("ERR J: source 3 needs the mux focused on wheel %ld (C%ld)\n", wheel, wheel);
            return true;
        }
        stop_raw_outputs();
        s_pi_wheel = static_cast<uint8_t>(wheel);
        s_pi_target_rpm = target_rpm;
        s_pi_kp = kp;
        s_pi_ki = ki;
        s_pi_center = static_cast<int16_t>(center);
        s_pi_duration_us = static_cast<uint32_t>(duration_ms) * 1000U;
        s_pi_source = static_cast<uint8_t>(source);
        s_pi_window = static_cast<uint8_t>(window);
        s_pi_mode = 0;
        s_pi_done = false;
        s_pi_active = true;
        s_last_command_us = esp_timer_get_time();
        xTaskNotifyGive(s_high_rate_task);
        printf(
          "ACK J: wheel %ld target %.2f RPM kp %.3f ki %.3f center %ld %ld ms source "
          "%ld window %ld\n",
          wheel,
          static_cast<double>(target_rpm),
          static_cast<double>(kp),
          static_cast<double>(ki),
          center,
          duration_ms,
          source,
          window);
        return true;
    }

    // B<wheel>,<high>,<low>,<on_ms>,<period_ms>,<duration_ms>[,<source>,<window>]
    //   — open-loop duty pattern at ~1 kHz: high for on_ms, low for the rest of
    //   the period (manual mode)
    if (*pointer == 'B')
    {
        ++pointer;
        if (!s_manual_mode)
        {
            printf("ERR B: send M first to pause the control task\n");
            return true;
        }
        if (s_timeline_active || s_record_active || s_high_rate_active || s_pi_active)
        {
            printf("ERR B: another harness job is active\n");
            return true;
        }
        char * endptr = nullptr;
        const long wheel = strtol(pointer, &endptr, 10);
        if (endptr == pointer || wheel < 0 || wheel > 3)
        {
            printf(
              "ERR B: expected B<wheel 0-3>,<high>,<low>,<on_ms>,<period_ms>,<ms>"
              "[,<source>,<window>]\n");
            return true;
        }
        pointer = (*endptr == ',') ? endptr + 1 : endptr;
        const long high = strtol(pointer, &endptr, 10);
        if (endptr == pointer)
        {
            printf("ERR B: missing high duty\n");
            return true;
        }
        pointer = (*endptr == ',') ? endptr + 1 : endptr;
        const long low = strtol(pointer, &endptr, 10);
        if (endptr == pointer)
        {
            printf("ERR B: missing low duty\n");
            return true;
        }
        pointer = (*endptr == ',') ? endptr + 1 : endptr;
        const long on_ms = strtol(pointer, &endptr, 10);
        if (endptr == pointer)
        {
            printf("ERR B: missing on time\n");
            return true;
        }
        pointer = (*endptr == ',') ? endptr + 1 : endptr;
        const long period_ms = strtol(pointer, &endptr, 10);
        if (endptr == pointer)
        {
            printf("ERR B: missing period\n");
            return true;
        }
        pointer = (*endptr == ',') ? endptr + 1 : endptr;
        const long duration_ms = strtol(pointer, &endptr, 10);
        if (endptr == pointer)
        {
            printf("ERR B: missing duration\n");
            return true;
        }
        long source = 3;
        long window = 8;
        if (*endptr == ',')
        {
            pointer = endptr + 1;
            source = strtol(pointer, &endptr, 10);
            if (*endptr == ',')
            {
                pointer = endptr + 1;
                window = strtol(pointer, &endptr, 10);
            }
        }
        if (high < 0 || high > 1023 || low < 0 || low > 1023 || on_ms < 1 || period_ms < 2 ||
            on_ms >= period_ms || period_ms > 100 || duration_ms < 200 || duration_ms > 10000 ||
            source < 0 || source > 3 || window < 2 ||
            window > static_cast<long>(HIGH_RATE_WINDOW_MAX))
        {
            printf(
              "ERR B: bad parameters (high/low 0..1023, 1<=on<period<=100 ms, "
              "duration 200..10000 ms, source 0-3, window 2..64)\n");
            return true;
        }
        if (source == 3 && s_i2c_wheel != wheel)
        {
            printf("ERR B: source 3 needs the mux focused on wheel %ld (C%ld)\n", wheel, wheel);
            return true;
        }
        stop_raw_outputs();
        s_pi_wheel = static_cast<uint8_t>(wheel);
        s_pi_target_rpm = 10.0f;
        s_pi_high = static_cast<int16_t>(high);
        s_pi_low = static_cast<int16_t>(low);
        s_pi_on_ms = static_cast<uint16_t>(on_ms);
        s_pi_period_ms = static_cast<uint16_t>(period_ms);
        s_pi_duration_us = static_cast<uint32_t>(duration_ms) * 1000U;
        s_pi_source = static_cast<uint8_t>(source);
        s_pi_window = static_cast<uint8_t>(window);
        s_pi_mode = 1;
        s_pi_done = false;
        s_pi_active = true;
        s_last_command_us = esp_timer_get_time();
        xTaskNotifyGive(s_high_rate_task);
        printf(
          "ACK B: wheel %ld high %ld low %ld on %ld ms period %ld ms %ld ms source "
          "%ld window %ld\n",
          wheel,
          high,
          low,
          on_ms,
          period_ms,
          duration_ms,
          source,
          window);
        return true;
    }

    // V — report estimator speed, FG speed/count, raw ADC and raw duty
    if (*pointer == 'V')
    {
        print_speed_report();
        return true;
    }

    // K<wheel>,<kick_duty>,<kick_ms>,<high>,<low>,<on_ms>,<period_ms>,<duration_ms>
    //   [,<source>,<window>] — start-up kick followed by the 1 kHz duty pattern
    //   without stopping the wheel (manual mode)
    if (*pointer == 'K')
    {
        ++pointer;
        if (!s_manual_mode)
        {
            printf("ERR K: send M first to pause the control task\n");
            return true;
        }
        if (s_timeline_active || s_record_active || s_high_rate_active || s_pi_active)
        {
            printf("ERR K: another harness job is active\n");
            return true;
        }
        long values[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        const int parsed = parse_ints(pointer, values, 10);
        if (parsed < 8)
        {
            printf(
              "ERR K: expected K<wheel>,<kick>,<kick_ms>,<hi>,<lo>,<on_ms>,<period_ms>,"
              "<ms>[,<source>,<window>]\n");
            return true;
        }
        const long wheel = values[0];
        const long kick_duty = values[1];
        const long kick_ms = values[2];
        const long high = values[3];
        const long low = values[4];
        const long on_ms = values[5];
        const long period_ms = values[6];
        const long duration_ms = values[7];
        long source = parsed >= 9 ? values[8] : 3;
        long window = parsed >= 10 ? values[9] : 4;
        if (wheel < 0 || wheel > 3 || kick_duty < -1023 || kick_duty > 1023 || kick_duty == 0 ||
            kick_ms < 20 || kick_ms > 500 || high < 0 || high > 1023 || low < 0 || low > 1023 ||
            on_ms < 1 || period_ms < 2 || on_ms >= period_ms || period_ms > 100 ||
            duration_ms < 200 || duration_ms > 15000 || source < 0 || source > 3 || window < 2 ||
            window > static_cast<long>(HIGH_RATE_WINDOW_MAX))
        {
            printf(
              "ERR K: bad parameters (kick +-1..1023, kick_ms 20..500, hi/lo 0..1023, "
              "1<=on<period<=100 ms, duration 200..15000 ms, source 0-3, window 2..64)\n");
            return true;
        }
        if (source == 3 && s_i2c_wheel != wheel)
        {
            printf("ERR K: source 3 needs the mux focused on wheel %ld (C%ld)\n", wheel, wheel);
            return true;
        }
        stop_raw_outputs();
        s_pi_wheel = static_cast<uint8_t>(wheel);
        s_pi_target_rpm = kick_duty >= 0 ? 10.0f : -10.0f;
        s_pi_kick_duty = static_cast<int16_t>(kick_duty);
        s_pi_kick_ms = static_cast<uint16_t>(kick_ms);
        s_pi_high = static_cast<int16_t>(high);
        s_pi_low = static_cast<int16_t>(low);
        s_pi_on_ms = static_cast<uint16_t>(on_ms);
        s_pi_period_ms = static_cast<uint16_t>(period_ms);
        s_pi_duration_us = static_cast<uint32_t>(duration_ms) * 1000U;
        s_pi_source = static_cast<uint8_t>(source);
        s_pi_window = static_cast<uint8_t>(window);
        s_pi_mode = 2;
        s_pi_done = false;
        s_pi_active = true;
        s_last_command_us = esp_timer_get_time();
        xTaskNotifyGive(s_high_rate_task);
        printf(
          "ACK K: wheel %ld kick %ld/%ld ms then hi %ld lo %ld on %ld/%ld ms for %ld ms "
          "source %ld window %ld\n",
          wheel,
          kick_duty,
          kick_ms,
          high,
          low,
          on_ms,
          period_ms,
          duration_ms,
          source,
          window);
        return true;
    }

    printf("ERR: unknown command '%s'\n", serial_buffer);
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
        SPIdev::init();
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
        motor_cfg.motor_fb_pins = config::test::MOTOR_FB_PINS;
        TEST_ASSERT_EQUAL(ESP_OK, BL48250::get_instance().configure(motor_cfg));

        // WheelController
        WheelController::get_instance().init();

        // Open-loop harness: one task plays the timed duty timelines and one
        // samples the FG counter/AS5600 at the requested interval.
        const esp_timer_create_args_t record_timer_args = {.callback = record_timer_callback,
                                                           .arg = nullptr,
                                                           .dispatch_method = ESP_TIMER_TASK,
                                                           .name = "record",
                                                           .skip_unhandled_events = true};
        TEST_ASSERT_EQUAL(ESP_OK, esp_timer_create(&record_timer_args, &s_record_timer));
        const esp_timer_create_args_t player_timer_args = {.callback = player_timer_callback,
                                                           .arg = nullptr,
                                                           .dispatch_method = ESP_TIMER_TASK,
                                                           .name = "player",
                                                           .skip_unhandled_events = true};
        TEST_ASSERT_EQUAL(ESP_OK, esp_timer_create(&player_timer_args, &s_player_timer));
        TEST_ASSERT_EQUAL(
          pdPASS, xTaskCreate(player_task, "harness_player", 4096, nullptr, 5, &s_player_task));
        TEST_ASSERT_EQUAL(
          pdPASS, xTaskCreate(recorder_task, "harness_record", 4096, nullptr, 4, &s_recorder_task));
        TEST_ASSERT_EQUAL(
          pdPASS, xTaskCreate(high_rate_task, "harness_hr", 4096, nullptr, 6, &s_high_rate_task));

        initialized = true;
    }

    WheelController & wc = WheelController::get_instance();

    printf("\n=======================================================\n");
    printf("    WheelController Interactive PID Test (harness build 6, 1 kHz tick)\n");
    printf("=======================================================\n");
    printf("Features: FastPID (fixed-point), feed-forward,\n");
    printf("  start-up kick, slew-rate limiting, anti-windup,\n");
    printf("  low-speed pulse mode, position control\n");
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
    printf("  W<duty>             - Raw signed duty on all wheels (manual mode)\n");
    printf("  W<fl>,<bl>,<br>,<fr> - Raw signed duty per wheel (manual mode)\n");
    printf("  Q<wheel>,<duty>,<on_ms>,<period_ms>,<cycles> - Pulse train\n");
    printf("  X<wheel>,<duty>,<hold_ms>[,...] - Step sequence, ends stopped\n");
    printf("  L<interval_us>,<duration_ms>,<wheel>,<i2c> - CSV record\n");
    printf("  C<wheel> / C-1      - Focus/release the mux for fast AS5600 reads\n");
    printf("  H<wheel>,<rpm>,<band>,<drive>,<brake>,<ms>[,<source>,<window>]\n");
    printf("                      - 1 kHz speed hysteresis on one wheel (manual mode);\n");
    printf("                        source 0 angle window, 1 EKF, 2 smoothed, 3 I2C window\n");
    printf("  J<wheel>,<rpm>,<kp>,<ki>,<center>,<ms>[,<source>,<window>]\n");
    printf("                      - 1 kHz PI duty around a center duty (manual mode)\n");
    printf("  B<wheel>,<high>,<low>,<on_ms>,<period_ms>,<ms>[,<source>,<window>]\n");
    printf("                      - 1 kHz open-loop duty pattern (manual mode)\n");
    printf("  V                   - Report estimator/FG speed, angle and duty\n");
    printf("  K<wheel>,<kick>,<kick_ms>,<hi>,<lo>,<on_ms>,<period_ms>,<ms>[,...]\n");
    printf("                      - 1 kHz kick then open-loop duty pattern (manual mode)\n");
    printf("=======================================================\n");

    while (true)
    {
        // Service serial commands
        read_serial_data();
        if (process_serial_command(wc))
            s_last_command_us = esp_timer_get_time();
        service_recorder();
        // Manual-mode dead-man: a lost host must not leave the wheels driven.
        if (s_manual_mode && !s_timeline_active && !s_record_active && !s_high_rate_active &&
            !s_pi_active && s_last_command_us != 0 &&
            esp_timer_get_time() - s_last_command_us > MANUAL_TIMEOUT_US)
        {
            s_last_command_us = 0;
            if (s_raw_duty[0] != 0 || s_raw_duty[1] != 0 || s_raw_duty[2] != 0 ||
                s_raw_duty[3] != 0)
            {
                stop_raw_outputs();
                printf("ERR: manual mode dead-man timeout, raw duties zeroed\n");
            }
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
