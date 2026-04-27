#ifndef RELAY_FEEDBACK_TUNER_H
#define RELAY_FEEDBACK_TUNER_H

#include <cmath>
#include <cstdint>

class RelayFeedbackTuner
{
   public:
    enum class State
    {
        IDLE,
        SETTLING,
        OSCILLATING,
        COMPLETE,
        FAILED
    };

    RelayFeedbackTuner();

    void start(
      float setpoint, float relay_amp, float u0, float hysteresis, uint32_t max_duration_ms,
      uint8_t min_periods);

    bool update(float pv, float & output);

    State get_state() const;
    bool is_success() const;
    void get_tunings(float & kp, float & ki, float & kd) const;
    float get_Ku() const;
    float get_Tu() const;
    float get_amplitude() const;

   private:
    static constexpr uint8_t MAX_PERIODS = 64;
    static constexpr uint32_t SETTLE_MS = 300;

    void compute_gains();
    bool check_convergence();
    static uint32_t millis();

    State m_state;
    float m_setpoint;
    float m_relay_amp;
    float m_u0;
    float m_hysteresis;
    uint32_t m_max_ms;
    uint8_t m_min_periods;

    uint32_t m_start_ms;
    uint32_t m_last_crossing_ms;
    bool m_last_pos;

    uint32_t m_half_periods[MAX_PERIODS];
    uint16_t m_num_half;
    uint32_t m_full_periods[MAX_PERIODS];
    float m_amplitudes[MAX_PERIODS];
    uint16_t m_num_full;

    float m_period_max;
    float m_period_min;

    float m_kp, m_ki, m_kd;
    float m_Ku, m_Tu_s, m_a;
};

#endif
