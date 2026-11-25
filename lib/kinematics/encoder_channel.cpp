#include "encoder_channel.h"

#include <esp_log.h>
#include <esp_timer.h>

#include <algorithm>
#include <cmath>

#include "AdcStream.h"
#include "BL48250.h"
#include "I2Cdev.h"

static const char * TAG = "EncoderChannel";

// Nominal publication rate of one encoder channel from the ADC stream
// configuration: sample rate * channel weight / (block mean * scan slots).
static constexpr double FAST_SAMPLE_RATE_HZ =
  static_cast<double>(CONFIG_ADC_SAMPLE_FREQ_HZ) * static_cast<double>(ADC_STREAM_ENCODER_WEIGHT) /
  (static_cast<double>(CONFIG_ADC_OVERSAMPLE) * static_cast<double>(ADC_STREAM_SCAN_SLOTS));

// Cutoff of the third-order angle filter used before the fast velocity
// regression. The filter reduces encoder quantization noise before the fit.
static constexpr double FAST_FILTER_CUTOFF_HZ = 250.0;

#if CONFIG_ENCODER_DIAG
// Nominal publish period of one encoder channel: the AdcStream scans
// ADC_STREAM_SCAN_SLOTS conversions per round, each encoder channel occupies
// ADC_STREAM_ENCODER_WEIGHT scan slots and publishes one mean per
// CONFIG_ADC_OVERSAMPLE conversions.
static constexpr uint32_t ENCODER_NOMINAL_PUBLISH_PERIOD_US = static_cast<uint32_t>(
  1000000ULL * CONFIG_ADC_OVERSAMPLE * ADC_STREAM_SCAN_SLOTS /
  (static_cast<uint64_t>(ADC_STREAM_SAMPLE_FREQ_HZ) * ADC_STREAM_ENCODER_WEIGHT));
// A publish interval is an overrun when it exceeds this multiple of nominal.
static constexpr uint32_t ENCODER_OVERRUN_FACTOR = 2;
// Console statistics window (one line per wheel per window).
static constexpr int64_t ENCODER_DIAG_PERIOD_US = 10 * 1000 * 1000;
#endif

/** @brief Reads the magnetic field flags of the selected AS5600 over I2C. */
EncoderChannel::MagnetField EncoderChannel::read_magnet_field()
{
    constexpr uint8_t STATUS_REGISTER = 0x0B;
    constexpr uint8_t MAGNET_DETECTED = 1U << 5;
    constexpr uint8_t MAGNET_WEAK = 1U << 4;
    constexpr uint8_t MAGNET_STRONG = 1U << 3;
    constexpr int STATUS_TIMEOUT_MS = 10;

    const i2c_master_dev_handle_t device = I2Cdev::get_device_handle(AS5600::AS5600_ADDR);
    uint8_t status = 0;
    if (device == nullptr ||
        i2c_master_transmit_receive(device, &STATUS_REGISTER, 1, &status, 1, STATUS_TIMEOUT_MS) !=
          ESP_OK)
        return MagnetField::UNREADABLE;
    if ((status & MAGNET_STRONG) != 0)
        return MagnetField::STRONG;
    if ((status & MAGNET_WEAK) != 0 || (status & MAGNET_DETECTED) == 0)
        return MagnetField::WEAK;
    return MagnetField::NORMAL;
}

/** @brief Binds the encoder to an analog input and restarts its estimates. */
void EncoderChannel::bind(adc_channel_t channel, adc_cali_handle_t calibration, adc_unit_t unit)
{
    m_encoder =
      std::make_unique<AS5600>(channel, calibration, calibration != nullptr, unit, ADC_BITWIDTH_12);
    m_filter = WheelKalmanFilter(2.0 * M_PI / config::driver::WHEEL_REDUCTION);
    m_measured_angle_rad = 0.0f;
    m_raw = 0;
    m_smoothed_velocity_rad_s = 0.0f;
    m_smoothed_velocity_started = false;
    m_last_fused_us = 0;
    m_last_pulse_us = 0;
    m_raw_stuck_since_us = 0;
    m_raw_stuck_warned = false;
    reset_fast_velocity();
}

/** @brief Updates the raw angle and both velocity estimates from one sample. */
bool EncoderChannel::process_sample(size_t wheel, uint16_t raw, int64_t now_us)
{
    if (m_encoder == nullptr)
        return false;

    const bool raw_clamped = update_clamp_status(wheel, raw, now_us);

#if CONFIG_ENCODER_DIAG
    m_timing.sample_start_us = esp_timer_get_time();
#endif

    float measured_angle_rad = 0.0f;
    m_encoder->process_new_reading(raw, &measured_angle_rad);
    m_measured_angle_rad = measured_angle_rad;
    m_raw = raw;
    const float wheel_angle_rad = measured_angle_rad / config::driver::WHEEL_REDUCTION;

    if (raw_clamped)
        reset_fast_velocity();
    else
        update_fast_velocity(wheel_angle_rad, now_us);
    update_fused_velocity(wheel, wheel_angle_rad, now_us);
    return true;
}

// Warn once when the encoder input stays at either end of the ADC range.
bool EncoderChannel::update_clamp_status(size_t wheel, uint16_t sample, int64_t now_us)
{
    if (!raw_is_clamped(sample))
    {
        m_raw_stuck_since_us = 0;
        m_raw_stuck_warned = false;
        return false;
    }

    if (m_raw_stuck_since_us == 0)
    {
        m_raw_stuck_since_us = now_us;
    }
    else if (!m_raw_stuck_warned && now_us - m_raw_stuck_since_us >= 1000000)
    {
        m_raw_stuck_warned = true;
        ESP_LOGW(TAG,
                 "M%u analog input stuck at %u for 1 s; check the encoder output/harness.",
                 static_cast<unsigned>(wheel + 1),
                 static_cast<unsigned>(sample));
    }
    return true;
}

// A clamped angle would corrupt the slope across the entire sample window.
void EncoderChannel::reset_fast_velocity()
{
    m_fast_head = 0;
    m_fast_count = 0;
    m_fast_valid = false;
    m_fast_velocity_rad_s = 0.0f;
    m_fast_last_us = 0;
    m_fast_sum_t = 0.0;
    m_fast_sum_tt = 0.0;
    m_fast_sum_a = 0.0;
    m_fast_sum_ta = 0.0;
    m_fast_continuous_angle_rad = 0.0;
    m_fast_last_raw_angle_rad = 0.0f;
    m_fast_filter.reset();
}

float EncoderChannel::filter_fast_angle(float wheel_angle_rad)
{
    if (!m_fast_filter.is_configured())
        m_fast_filter.configure(FAST_SAMPLE_RATE_HZ, FAST_FILTER_CUTOFF_HZ);

    if (!m_fast_filter.is_initialized())
    {
        m_fast_continuous_angle_rad = wheel_angle_rad;
    }
    else
    {
        constexpr float angle_period_rad =
          2.0f * static_cast<float>(M_PI) / config::driver::WHEEL_REDUCTION;
        constexpr float half_period = 0.5f * angle_period_rad;
        float delta = wheel_angle_rad - m_fast_last_raw_angle_rad;
        if (delta > half_period)
            delta -= angle_period_rad;
        else if (delta < -half_period)
            delta += angle_period_rad;
        m_fast_continuous_angle_rad += delta;
    }

    m_fast_last_raw_angle_rad = wheel_angle_rad;
    return m_fast_filter.process(static_cast<float>(m_fast_continuous_angle_rad));
}

// Fit an angle slope over the latest samples for the low-speed loop.
void EncoderChannel::update_fast_velocity(float wheel_angle_rad, int64_t now_us)
{
    const float filtered_angle = filter_fast_angle(wheel_angle_rad);

    // Shift the time origin to the newest sample, which contributes at t = 0.
    if (m_fast_count > 0)
    {
        const double delta_s = static_cast<double>(now_us - m_fast_last_us) * 1e-6;
        const double count = static_cast<double>(m_fast_count);
        m_fast_sum_tt = m_fast_sum_tt - 2.0 * delta_s * m_fast_sum_t + count * delta_s * delta_s;
        m_fast_sum_ta -= delta_s * m_fast_sum_a;
        m_fast_sum_t -= count * delta_s;
    }
    m_fast_last_us = now_us;
    m_fast_sum_a += filtered_angle;

    // Subtract the oldest point before replacing it in the full window.
    if (m_fast_count == FAST_WINDOW)
    {
        const double old_time_s = static_cast<double>(m_fast_times_us[m_fast_head] - now_us) * 1e-6;
        const double old_angle = static_cast<double>(m_fast_angles[m_fast_head]);
        m_fast_sum_t -= old_time_s;
        m_fast_sum_tt -= old_time_s * old_time_s;
        m_fast_sum_a -= old_angle;
        m_fast_sum_ta -= old_time_s * old_angle;
    }
    else
    {
        ++m_fast_count;
    }

    m_fast_angles[m_fast_head] = filtered_angle;
    m_fast_times_us[m_fast_head] = now_us;
    m_fast_head = (m_fast_head + 1) % FAST_WINDOW;
    if (m_fast_count < FAST_WINDOW)
        return;

    // Refresh the rolling sums once per ring revolution so floating-point
    // rounding cannot accumulate into a velocity bias over long runs.
    if (m_fast_head == 0)
    {
        double sum_t = 0.0;
        double sum_tt = 0.0;
        double sum_a = 0.0;
        double sum_ta = 0.0;
        for (size_t i = 0; i < FAST_WINDOW; ++i)
        {
            const double time_s = static_cast<double>(m_fast_times_us[i] - now_us) * 1e-6;
            const double angle = static_cast<double>(m_fast_angles[i]);
            sum_t += time_s;
            sum_tt += time_s * time_s;
            sum_a += angle;
            sum_ta += time_s * angle;
        }
        m_fast_sum_t = sum_t;
        m_fast_sum_tt = sum_tt;
        m_fast_sum_a = sum_a;
        m_fast_sum_ta = sum_ta;
    }

    constexpr double count = static_cast<double>(FAST_WINDOW);
    const double denominator = count * m_fast_sum_tt - m_fast_sum_t * m_fast_sum_t;
    if (denominator > 1e-12)
    {
        m_fast_velocity_rad_s =
          static_cast<float>((count * m_fast_sum_ta - m_fast_sum_t * m_fast_sum_a) / denominator);
        m_fast_valid = true;
    }
}

// Update the fused estimate used by continuous control, tuning and odometry.
void EncoderChannel::update_fused_velocity(size_t wheel, float wheel_angle_rad, int64_t now_us)
{
    constexpr int64_t FUSED_PERIOD_US = 1200;
    constexpr int64_t PULSE_PERIOD_US = 3500;
    constexpr float VELOCITY_SMOOTHING_US = 8000.0f;
    const int64_t fused_elapsed_us = now_us - m_last_fused_us;
    if (fused_elapsed_us < FUSED_PERIOD_US)
        return;

    m_last_fused_us = now_us;
    float motor_velocity = 0.0f;
    bool have_pulse_velocity = false;
    if (now_us - m_last_pulse_us >= PULSE_PERIOD_US)
    {
        m_last_pulse_us = now_us;
        have_pulse_velocity = BL48250::get_instance().pulse_velocity_rad_s(wheel, &motor_velocity, 0);
    }

    // Only fuse motor pulse velocity when it agrees with the encoder's physical
    // direction; an unsigned pulse counter during coasting or back-driving must
    // not fight the magnetic encoder.
    const float signed_pulse_velocity = -motor_velocity;
    const bool pulse_agrees = !m_fast_valid || std::fabs(m_fast_velocity_rad_s) < 1.5f ||
                              (signed_pulse_velocity * m_fast_velocity_rad_s) > 0.0f;
    if (have_pulse_velocity && motor_velocity != 0.0f && pulse_agrees)
        m_filter.update(wheel_angle_rad, signed_pulse_velocity);
    else
        m_filter.update(wheel_angle_rad);

    const float velocity = static_cast<float>(m_filter.get_velocity_rad_s());
    const float elapsed_us = static_cast<float>(fused_elapsed_us);
    const float alpha = elapsed_us / (VELOCITY_SMOOTHING_US + elapsed_us);
    if (m_smoothed_velocity_started)
        m_smoothed_velocity_rad_s += alpha * (velocity - m_smoothed_velocity_rad_s);
    else
    {
        m_smoothed_velocity_rad_s = velocity;
        m_smoothed_velocity_started = true;
    }
}

#if CONFIG_ENCODER_DIAG
void EncoderChannel::record_sample_timing(size_t wheel)
{
    const int64_t now_us = esp_timer_get_time();
    TimingStats & diag = m_timing;
    if (diag.last_publish_us != 0)
    {
        const uint32_t gap_us = static_cast<uint32_t>(now_us - diag.last_publish_us);
        diag.gap_sum_us += gap_us;
        diag.gap_max_us = std::max(diag.gap_max_us, gap_us);
        if (gap_us > ENCODER_OVERRUN_FACTOR * ENCODER_NOMINAL_PUBLISH_PERIOD_US)
            ++diag.overruns;
    }
    diag.last_publish_us = now_us;
    const uint32_t proc_us = static_cast<uint32_t>(now_us - diag.sample_start_us);
    diag.proc_sum_us += proc_us;
    diag.proc_max_us = std::max(diag.proc_max_us, proc_us);
    ++diag.publishes;

    if (diag.window_start_us == 0)
        diag.window_start_us = now_us;
    if (now_us - diag.window_start_us < ENCODER_DIAG_PERIOD_US)
        return;

    const uint32_t gaps = (diag.publishes > 1) ? diag.publishes - 1 : 0;
    const float gap_avg_us =
      (gaps > 0) ? static_cast<float>(diag.gap_sum_us) / static_cast<float>(gaps) : 0.0f;
    const float proc_avg_us = (diag.publishes > 0) ? static_cast<float>(diag.proc_sum_us) /
                                                       static_cast<float>(diag.publishes)
                                                   : 0.0f;
    // Keep the full log line within the 96-byte serial debug text frame.
    ESP_LOGI(TAG,
             "ENC M%u n=%u gap=%u/%u proc=%u/%u ovr=%u",
             static_cast<unsigned>(wheel + 1),
             static_cast<unsigned>(diag.publishes),
             static_cast<unsigned>(std::lround(gap_avg_us)),
             static_cast<unsigned>(diag.gap_max_us),
             static_cast<unsigned>(std::lround(proc_avg_us)),
             static_cast<unsigned>(diag.proc_max_us),
             static_cast<unsigned>(diag.overruns));
    diag.publishes = 0;
    diag.gap_sum_us = 0;
    diag.gap_max_us = 0;
    diag.proc_sum_us = 0;
    diag.proc_max_us = 0;
    diag.overruns = 0;
    diag.window_start_us = now_us;
}
#endif
