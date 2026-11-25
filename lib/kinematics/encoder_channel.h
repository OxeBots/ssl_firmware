#ifndef _ENCODER_CHANNEL_H_
#define _ENCODER_CHANNEL_H_

#include <esp_attr.h>
#include <sdkconfig.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "AS5600.h"
#include "butterworth3.h"
#include "wheel_ekf.h"

class EncoderChannel
{
   public:
    enum class MagnetField : uint8_t
    {
        UNREADABLE,
        NORMAL,
        WEAK,
        STRONG,
    };

    // Reads the AS5600 currently selected on the mux.
    static MagnetField read_magnet_field();

    // The caller serializes binding, sample processing and state reads.
    void bind(adc_channel_t channel, adc_cali_handle_t calibration, adc_unit_t unit);
    bool IRAM_ATTR process_sample(size_t wheel, uint16_t raw, int64_t now_us);

#if CONFIG_ENCODER_DIAG
    // The acquisition task records timing after publishing the sample.
    void IRAM_ATTR record_sample_timing(size_t wheel);
#endif

    AS5600 * get_encoder()
    {
        return m_encoder.get();
    }
    const AS5600 * get_encoder() const
    {
        return m_encoder.get();
    }

    uint16_t get_raw() const
    {
        return m_raw;
    }
    float get_measured_angle_rad() const
    {
        return m_measured_angle_rad;
    }
    float get_filtered_angle_rad() const
    {
        return static_cast<float>(m_filter.get_angle_rad());
    }
    float get_filtered_velocity_rad_s() const
    {
        return static_cast<float>(m_filter.get_velocity_rad_s());
    }
    float get_smoothed_velocity_rad_s() const
    {
        return m_smoothed_velocity_rad_s;
    }
    float get_fast_velocity_rad_s() const
    {
        return m_fast_velocity_rad_s;
    }
    bool has_smoothed_velocity() const
    {
        return m_smoothed_velocity_started;
    }
    bool has_fast_velocity() const
    {
        return m_fast_valid;
    }
    bool is_analog_stuck() const
    {
        return m_raw_stuck_warned;
    }
    bool is_raw_clamped() const
    {
        return raw_is_clamped(m_raw);
    }

   private:
    static constexpr uint16_t ADC_MAX_COUNT = 4095;
    static bool raw_is_clamped(uint16_t sample)
    {
        return sample == 0 || sample >= ADC_MAX_COUNT;
    }
    bool IRAM_ATTR update_clamp_status(size_t wheel, uint16_t sample, int64_t now_us);
    void IRAM_ATTR reset_fast_velocity();
    float IRAM_ATTR filter_fast_angle(float wheel_angle_rad);
    void IRAM_ATTR update_fast_velocity(float wheel_angle_rad, int64_t now_us);
    void IRAM_ATTR update_fused_velocity(size_t wheel, float wheel_angle_rad, int64_t now_us);

    std::unique_ptr<AS5600> m_encoder;
    WheelKalmanFilter m_filter;
    // Latest encoder-shaft angle before filtering (rad, one motor turn).
    float m_measured_angle_rad = 0.0f;
    // Latest raw ADC block mean of the analog input feeding this wheel.
    uint16_t m_raw = 0;
    // Low-pass output of the filter velocity used by the wheel controller.
    float m_smoothed_velocity_rad_s = 0.0f;
    bool m_smoothed_velocity_started = false;
    // Sliding least-squares velocity for the high-rate low-speed loop: the
    // raw wheel angle of the last FAST_WINDOW publications. The window is
    // about 8 ms at the 3.08 kHz acquisition rate.
    static constexpr size_t FAST_WINDOW = 26;
    std::array<float, FAST_WINDOW> m_fast_angles = {};
    std::array<int64_t, FAST_WINDOW> m_fast_times_us = {};
    size_t m_fast_head = 0;
    size_t m_fast_count = 0;
    float m_fast_velocity_rad_s = 0.0f;
    bool m_fast_valid = false;
    // Rolling least-squares sums kept relative to the newest sample, so the
    // per-sample cost is O(1) instead of re-walking the window.
    int64_t m_fast_last_us = 0;
    double m_fast_sum_t = 0.0;
    double m_fast_sum_tt = 0.0;
    double m_fast_sum_a = 0.0;
    double m_fast_sum_ta = 0.0;
    int64_t m_last_fused_us = 0;
    int64_t m_last_pulse_us = 0;
    // Continuous (unwrapped) wheel angle and its fast low-pass, plus the
    // last raw wheel angle for the unwrap step.
    double m_fast_continuous_angle_rad = 0.0;
    float m_fast_last_raw_angle_rad = 0.0f;
    Butterworth3 m_fast_filter;
    // Analog-input stuck diagnostic: timestamp of the first clamped block
    // mean of the current episode (us) and whether it was already logged.
    int64_t m_raw_stuck_since_us = 0;
    bool m_raw_stuck_warned = false;

#if CONFIG_ENCODER_DIAG
    struct TimingStats
    {
        int64_t sample_start_us = 0;
        uint32_t publishes = 0;       // published samples in this window
        uint32_t overruns = 0;        // gaps above 2x the nominal period
        int64_t last_publish_us = 0;  // previous publish timestamp
        uint64_t gap_sum_us = 0;
        uint32_t gap_max_us = 0;
        uint64_t proc_sum_us = 0;  // sample processing time
        uint32_t proc_max_us = 0;
        int64_t window_start_us = 0;
    };
    TimingStats m_timing;
#endif
};

#endif  // _ENCODER_CHANNEL_H_
