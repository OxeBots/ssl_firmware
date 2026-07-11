#ifndef FAST_PID_H_
#define FAST_PID_H_

#include <stdint.h>

#define FAST_PID_INTEG_MAX (INT32_MAX)
#define FAST_PID_INTEG_MIN (INT32_MIN)
#define FAST_PID_DERIV_MAX (INT16_MAX)
#define FAST_PID_DERIV_MIN (INT16_MIN)

#define FAST_PID_PARAM_SHIFT 8
#define FAST_PID_PARAM_BITS 16
#define FAST_PID_PARAM_MAX (((0x1ULL << FAST_PID_PARAM_BITS) - 1) >> FAST_PID_PARAM_SHIFT)
#define FAST_PID_PARAM_MULT \
    (((0x1ULL << FAST_PID_PARAM_BITS)) >> (FAST_PID_PARAM_BITS - FAST_PID_PARAM_SHIFT))

class FastPID
{
   public:
    FastPID() { clear(); }

    FastPID(float kp, float ki, float kd, float hz, int bits = 16, bool sign = false)
    {
        configure(kp, ki, kd, hz, bits, sign);
    }

    ~FastPID() = default;

    bool setCoefficients(float kp, float ki, float kd, float hz);

    bool setOutputConfig(int bits, bool sign);

    bool setOutputRange(int16_t min, int16_t max);

    void clear();

    bool configure(float kp, float ki, float kd, float hz, int bits = 16, bool sign = false);

    int16_t step(int16_t sp, int16_t fb);

    bool err() const { return m_cfg_err; }

   private:
    uint32_t float_to_param(float in);
    void set_cfg_err();

    uint32_t m_p = 0;
    uint32_t m_i = 0;
    uint32_t m_d = 0;
    int64_t m_outmax = 0;
    int64_t m_outmin = 0;
    bool m_cfg_err = false;

    int16_t m_last_sp = 0;
    int16_t m_last_out = 0;
    int64_t m_sum = 0;
    int32_t m_last_err = 0;
};

#endif  // FAST_PID_H_