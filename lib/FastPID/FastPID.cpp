#include "FastPID.h"

#include <algorithm>

void FastPID::clear()
{
    m_last_sp = 0;
    m_last_out = 0;
    m_sum = 0;
    m_last_err = 0;
}

bool FastPID::setCoefficients(float kp, float ki, float kd, float hz)
{
    m_p = float_to_param(kp);
    m_i = float_to_param(ki / hz);
    m_d = float_to_param(kd * hz);
    return !m_cfg_err;
}

bool FastPID::setOutputConfig(int bits, bool sign)
{
    if (bits > 16 || bits < 1)
    {
        set_cfg_err();
    }
    else
    {
        if (bits == 16)
            m_outmax = static_cast<int64_t>((0xFFFFULL >> (17 - bits)) * FAST_PID_PARAM_MULT);
        else
            m_outmax = static_cast<int64_t>((0xFFFFULL >> (16 - bits)) * FAST_PID_PARAM_MULT);

        if (sign)
            m_outmin =
              -static_cast<int64_t>(((0xFFFFULL >> (17 - bits)) + 1) * FAST_PID_PARAM_MULT);
        else
            m_outmin = 0;
    }
    return !m_cfg_err;
}

bool FastPID::setOutputRange(int16_t min, int16_t max)
{
    if (min >= max)
    {
        set_cfg_err();
        return !m_cfg_err;
    }
    m_outmin = static_cast<int64_t>(min) * static_cast<int64_t>(FAST_PID_PARAM_MULT);
    m_outmax = static_cast<int64_t>(max) * static_cast<int64_t>(FAST_PID_PARAM_MULT);
    return !m_cfg_err;
}

bool FastPID::configure(float kp, float ki, float kd, float hz, int bits, bool sign)
{
    clear();
    m_cfg_err = false;
    setCoefficients(kp, ki, kd, hz);
    setOutputConfig(bits, sign);
    return !m_cfg_err;
}

uint32_t FastPID::float_to_param(float in)
{
    if (in > FAST_PID_PARAM_MAX || in < 0)
    {
        m_cfg_err = true;
        return 0;
    }

    uint32_t param = static_cast<uint32_t>(in * FAST_PID_PARAM_MULT);

    if (in != 0 && param == 0)
    {
        m_cfg_err = true;
        return 0;
    }

    return param;
}

/**
 * @brief Execute one PID step.
 *
 * All coefficients are pre-scaled to fixed-point during configure().
 *
 * @param sp Setpoint (scaled integer input)
 * @param fb Feedback (scaled integer input)
 * @return PID output (clamped to configured output range)
 */
int16_t FastPID::step(int16_t sp, int16_t fb)
{
    int32_t err = static_cast<int32_t>(sp) - static_cast<int32_t>(fb);
    int32_t P = 0;
    int32_t I = 0;
    int32_t D = 0;

    if (m_p)
    {
        P = static_cast<int32_t>(m_p) * static_cast<int32_t>(err);
    }

    if (m_i)
    {
        m_sum += static_cast<int64_t>(err) * static_cast<int64_t>(m_i);

        if (m_sum > FAST_PID_INTEG_MAX)
            m_sum = FAST_PID_INTEG_MAX;
        else if (m_sum < FAST_PID_INTEG_MIN)
            m_sum = FAST_PID_INTEG_MIN;

        I = static_cast<int32_t>(m_sum);
    }

    if (m_d)
    {
        int32_t deriv = (err - m_last_err) - static_cast<int32_t>(sp - m_last_sp);
        m_last_sp = sp;
        m_last_err = err;

        if (deriv > FAST_PID_DERIV_MAX)
            deriv = FAST_PID_DERIV_MAX;
        else if (deriv < FAST_PID_DERIV_MIN)
            deriv = FAST_PID_DERIV_MIN;

        D = static_cast<int32_t>(m_d) * static_cast<int32_t>(deriv);
    }

    int64_t out = static_cast<int64_t>(P) + static_cast<int64_t>(I) + static_cast<int64_t>(D);

    if (out > m_outmax)
        out = m_outmax;
    else if (out < m_outmin)
        out = m_outmin;

    int16_t rval = static_cast<int16_t>(out >> FAST_PID_PARAM_SHIFT);

    if (out & (0x1ULL << (FAST_PID_PARAM_SHIFT - 1)))
        rval++;

    return rval;
}

void FastPID::set_cfg_err()
{
    m_cfg_err = true;
    m_p = m_i = m_d = 0;
}