#include "butterworth3.h"

#include <cmath>

/**
 * @brief Designs the third-order Butterworth sections for a sample rate.
 *
 * The bilinear transform uses a prewarped cutoff k = tan(pi * fc / fs).
 * A real pole and a conjugate pair form the two unity-gain sections.
 */
void Butterworth3::configure(double sample_rate_hz, double cutoff_hz)
{
    if (!(sample_rate_hz > 0.0) || !(cutoff_hz > 0.0) || !(cutoff_hz < 0.5 * sample_rate_hz))
        return;
    const double k = std::tan(M_PI * cutoff_hz / sample_rate_hz);

    Section & first = m_sections[0];
    const double first_b0 = k / (1.0 + k);
    first.b0 = static_cast<float>(first_b0);
    first.b1 = first.b0;
    first.b2 = 0.0F;
    first.a1 = static_cast<float>((k - 1.0) / (k + 1.0));
    first.a2 = 0.0F;

    Section & pair = m_sections[1];
    const double damping = 1.0;  // 2*sin(pi/(2*3)) for the single Butterworth pair
    const double denominator = 1.0 + damping * k + k * k;
    const double pair_b0 = k * k / denominator;
    pair.b0 = static_cast<float>(pair_b0);
    pair.b1 = static_cast<float>(2.0 * pair_b0);
    pair.b2 = pair.b0;
    pair.a1 = static_cast<float>(2.0 * (k * k - 1.0) / denominator);
    pair.a2 = static_cast<float>((1.0 - damping * k + k * k) / denominator);

    m_configured = true;
    reset();
}

void Butterworth3::reset()
{
    for (Section & section : m_sections)
    {
        section.state1 = 0.0F;
        section.state2 = 0.0F;
    }
    m_initialized = false;
}

/**
 * @brief Processes one sample using transposed direct form II per section.
 */
float Butterworth3::process(float value)
{
    if (!m_configured)
        return value;
    if (!m_initialized)
    {
        // Prime each section at the first value to avoid a startup transient.
        for (Section & section : m_sections)
        {
            section.state1 =
              (section.b0 + section.b1 + section.b2 - section.a1 - section.a2) * value;
            section.state2 =
              (section.b0 - section.b1 + section.b2 + section.a1 - section.a2) * value;
        }
        m_initialized = true;
        return value;
    }

    float output = value;
    for (Section & section : m_sections)
    {
        const float y = section.b0 * output + section.state1;
        section.state1 = section.b1 * output - section.a1 * y + section.state2;
        section.state2 = section.b2 * output - section.a2 * y;
        output = y;
    }
    return output;
}
