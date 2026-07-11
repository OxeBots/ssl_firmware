#include "RelayFeedbackTuner.h"

#include <esp_timer.h>

#include <algorithm>
#include <cstring>

RelayFeedbackTuner::RelayFeedbackTuner()
: m_state(State::IDLE),
  m_setpoint(0.0f),
  m_relay_amp(0.0f),
  m_u0(0.0f),
  m_hysteresis(0.0f),
  m_max_ms(0),
  m_min_periods(0),
  m_start_ms(0),
  m_last_crossing_ms(0),
  m_last_pos(false),
  m_num_half(0),
  m_num_full(0),
  m_period_max(0.0f),
  m_period_min(0.0f),
  m_kp(0.0f),
  m_ki(0.0f),
  m_kd(0.0f),
  m_Ku(0.0f),
  m_Tu_s(0.0f),
  m_a(0.0f)
{
    memset(m_half_periods, 0, sizeof(m_half_periods));
    memset(m_full_periods, 0, sizeof(m_full_periods));
    memset(m_amplitudes, 0, sizeof(m_amplitudes));
}

void RelayFeedbackTuner::start(
  float setpoint, float relay_amp, float u0, float hysteresis, uint32_t max_duration_ms,
  uint8_t min_periods)
{
    m_setpoint = setpoint;
    m_relay_amp = relay_amp;
    m_u0 = u0;
    m_hysteresis = hysteresis;
    m_max_ms = max_duration_ms;
    m_min_periods = min_periods;
    m_start_ms = millis();
    m_last_crossing_ms = 0;
    m_last_pos = false;
    m_num_half = 0;
    m_num_full = 0;
    m_period_max = 0.0f;
    m_period_min = 0.0f;
    m_kp = m_ki = m_kd = 0.0f;
    m_Ku = m_Tu_s = m_a = 0.0f;
    m_state = State::SETTLING;
}

bool RelayFeedbackTuner::update(float pv, float & output)
{
    uint32_t now = millis();

    switch (m_state)
    {
        case State::SETTLING:
            output = m_u0;
            if (now - m_start_ms >= SETTLE_MS)
            {
                m_state = State::OSCILLATING;
                m_period_max = pv;
                m_period_min = pv;
                // Initialize crossing timestamp now so the first half-period
                // isn't measured from time 0.
                m_last_crossing_ms = now;
            }
            return true;

        case State::OSCILLATING:
        {
            // Hysteresis-based relay switching (matches reference implementations
            // from pidautotuner.cpp, ziegler_nichols.c, astrom_hagglund.c).
            // Relay output = u0 + relay_amp (HIGH) or u0 - relay_amp (LOW).
            bool relay_high = (m_last_pos);  // true = output HIGH, false = output LOW

            // Switch from HIGH to LOW when pv exceeds setpoint + hysteresis.
            // Switch from LOW to HIGH when pv drops below setpoint - hysteresis.
            bool should_switch = false;
            bool new_state = relay_high;

            if (relay_high && pv > m_setpoint + m_hysteresis)
            {
                should_switch = true;
                new_state = false;
            }
            else if (!relay_high && pv < m_setpoint - m_hysteresis)
            {
                should_switch = true;
                new_state = true;
            }

            if (should_switch)
            {
                uint32_t half = now - m_last_crossing_ms;
                if (half >= 5)
                {
                    m_half_periods[m_num_half % MAX_PERIODS] = half;
                    m_num_half++;

                    if ((m_num_half % 2) == 0 && m_num_half >= 2)
                    {
                        uint32_t idx_prev = (m_num_half - 2) % MAX_PERIODS;
                        uint32_t full = half + m_half_periods[idx_prev];
                        m_full_periods[m_num_full % MAX_PERIODS] = full;
                        m_amplitudes[m_num_full % MAX_PERIODS] =
                          (m_period_max - m_period_min) * 0.5f;
                        m_num_full++;

                        m_period_max = pv;
                        m_period_min = pv;
                    }
                }
                m_last_crossing_ms = now;
                m_last_pos = new_state;
            }
            else
            {
                if (pv > m_period_max)
                    m_period_max = pv;
                if (pv < m_period_min)
                    m_period_min = pv;
            }

            if (check_convergence())
            {
                output = 0.0f;
                return true;
            }

            if (now - m_start_ms >= m_max_ms)
            {
                m_state = State::FAILED;
                output = 0.0f;
                return true;
            }

            output = m_u0 + (m_last_pos ? m_relay_amp : -m_relay_amp);
            return true;
        }

        default:
            output = 0.0f;
            return false;
    }
}

void RelayFeedbackTuner::compute_gains()
{
    if (m_Tu_s <= 0.0f || m_Ku <= 0.0f)
    {
        m_kp = m_ki = m_kd = 0.0f;
        return;
    }
    // Damped Astrom-Hagglund: design parameter 0.5 -> Kp = 0.5 Ku
    // Ti = 0.8 Tu, Td = 0.125 Tu
    float Ti = 0.8f * m_Tu_s;
    float Td = 0.125f * m_Tu_s;
    m_kp = 0.5f * m_Ku;
    m_ki = m_kp / Ti;
    m_kd = m_kp * Td;

    m_kp = std::max(0.01f, std::min(500.0f, m_kp));
    m_ki = std::max(0.01f, std::min(50.0f, m_ki));
    m_kd = std::max(0.001f, std::min(50.0f, m_kd));
}

bool RelayFeedbackTuner::check_convergence()
{
    if (m_num_full < m_min_periods + 1)
        return false;

    uint16_t i0 = (m_num_full - 1) % MAX_PERIODS;
    uint16_t i1 = (m_num_full - 2) % MAX_PERIODS;
    uint16_t i2 = (m_num_full - 3) % MAX_PERIODS;

    float avg_p = (static_cast<float>(m_full_periods[i0]) +
                   static_cast<float>(m_full_periods[i1]) +
                   static_cast<float>(m_full_periods[i2])) /
                  3.0f;
    float avg_a = (m_amplitudes[i0] + m_amplitudes[i1] + m_amplitudes[i2]) / 3.0f;

    if (avg_p <= 0.0f || avg_a <= 0.0f)
        return false;

    bool p_ok = std::fabs(static_cast<float>(m_full_periods[i0]) - avg_p) < 0.25f * avg_p &&
                std::fabs(static_cast<float>(m_full_periods[i1]) - avg_p) < 0.25f * avg_p &&
                std::fabs(static_cast<float>(m_full_periods[i2]) - avg_p) < 0.25f * avg_p;
    bool a_ok = std::fabs(m_amplitudes[i0] - avg_a) < 0.25f * avg_a &&
                std::fabs(m_amplitudes[i1] - avg_a) < 0.25f * avg_a &&
                std::fabs(m_amplitudes[i2] - avg_a) < 0.25f * avg_a;

    if (p_ok && a_ok && avg_a > 0.2f)
    {
        m_Tu_s = avg_p / 1000.0f;
        m_a = avg_a;
        m_Ku = (4.0f * m_relay_amp) / (static_cast<float>(M_PI) * m_a);
        compute_gains();
        m_state = State::COMPLETE;
        return true;
    }

    return false;
}

RelayFeedbackTuner::State RelayFeedbackTuner::get_state() const
{
    return m_state;
}

bool RelayFeedbackTuner::is_success() const
{
    return m_state == State::COMPLETE;
}

void RelayFeedbackTuner::get_tunings(float & kp, float & ki, float & kd) const
{
    kp = m_kp;
    ki = m_ki;
    kd = m_kd;
}

float RelayFeedbackTuner::get_Ku() const
{
    return m_Ku;
}

float RelayFeedbackTuner::get_Tu() const
{
    return m_Tu_s;
}

float RelayFeedbackTuner::get_amplitude() const
{
    return m_a;
}

uint32_t RelayFeedbackTuner::millis()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}
