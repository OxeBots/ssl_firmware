#ifndef _BUTTERWORTH3_H_
#define _BUTTERWORTH3_H_

// Third-order Butterworth low-pass filter for a fixed sample rate.
class Butterworth3
{
   public:
    void configure(double sample_rate_hz, double cutoff_hz);
    void reset();
    float process(float value);

    bool is_configured() const { return m_configured; }

    bool is_initialized() const { return m_initialized; }

   private:
    struct Section
    {
        float b0 = 0.0F;
        float b1 = 0.0F;
        float b2 = 0.0F;
        float a1 = 0.0F;
        float a2 = 0.0F;
        float state1 = 0.0F;
        float state2 = 0.0F;
    };

    Section m_sections[2] = {};
    bool m_configured = false;
    bool m_initialized = false;
};

#endif  // _BUTTERWORTH3_H_
