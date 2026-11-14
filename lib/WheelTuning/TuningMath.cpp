#include "TuningMath.h"

#include <algorithm>
#include <cmath>

namespace tuning
{

namespace
{

// Movement thresholds for the identification (wheel shaft).
constexpr float MIN_MOVING_VELOCITY_RAD_S = 1.0f;
constexpr float MIN_STEP_DELTA_RAD_S = 0.5f;
constexpr float MIN_PLANT_GAIN = 1.0e-4f;
constexpr float MAX_PLANT_GAIN = 100.0f;
constexpr float BASELINE_FRACTION = 0.1f;
constexpr float FINAL_FRACTION = 0.2f;
constexpr float DEAD_TIME_FRACTION = 0.02f;
constexpr float TAU_FRACTION = 0.632f;
constexpr float VALIDATION_MAX_RAD_S = 30.0f;
constexpr float VALIDATION_MIN_RAD_S = 5.0f;
// The validation target leaves two fifths of the duty range free for the P
// term, so a step never starts saturated.
constexpr float VALIDATION_DUTY_FRACTION = 0.65f;
// The P gain is capped so the first step keeps u_ff plus the initial P term
// close to the duty the feedforward already needs: a small kick, no saturation.
constexpr float PROPORTIONAL_DUTY_FRACTION = 0.72f;
// At the step start the P term lifts the equilibrium speed by a*Kp (relative
// overshoot). Keeping a*Kp below this bound keeps that anticipation small.
constexpr float LOOP_GAIN_NOMINAL = 0.06f;
constexpr float LOOP_GAIN_MIN = 0.02f;
constexpr float LOOP_GAIN_REFINE_MAX = 0.15f;
// The measured step response first goes through a 3-sample median filter
// (encoder spikes) and then a 50 ms moving average (quantization ripple).
constexpr size_t SMOOTHING_HALF_WINDOW = 2;
// Steady-state window of a validation step (samples at 100 Hz).
constexpr size_t FINAL_WINDOW_SAMPLES = 40;
constexpr float DUTY_MAX = 1023.0f;
constexpr float DUTY_RESERVE = 60.0f;
constexpr float MIN_CLOSED_LOOP_TAU_S = 0.12f;

// Mean of a range of samples.
float mean_of(const float * values, size_t begin, size_t end)
{
    if (end <= begin)
        return 0.0f;
    float sum = 0.0f;
    for (size_t i = begin; i < end; ++i) sum += values[i];
    return sum / static_cast<float>(end - begin);
}

// Median of three samples around one index (spike rejection for the measured
// step response; the encoder velocity occasionally reports a single outlier).
float median_of_three(const float * values, size_t count, size_t index)
{
    const size_t first = (index > 0) ? index - 1 : 0;
    const size_t last = std::min(count - 1, index + 1);
    float window[3] = {values[first], values[index], values[last]};
    if (window[0] > window[1])
        std::swap(window[0], window[1]);
    if (window[1] > window[2])
        std::swap(window[1], window[2]);
    if (window[0] > window[1])
        std::swap(window[0], window[1]);
    return window[1];
}

// First sample index whose value crossed the fraction of the total delta.
bool first_crossing(const float * times_s,
                    const float * values,
                    size_t count,
                    float baseline,
                    float delta,
                    float fraction,
                    float * crossing_s)
{
    for (size_t i = 0; i < count; ++i)
    {
        if (std::fabs(values[i] - baseline) >= fraction * std::fabs(delta))
        {
            *crossing_s = times_s[i];
            return true;
        }
    }
    return false;
}

}  // namespace

bool fit_static_curve(
  const float * duties, const float * velocities, size_t count, float * gain, float * friction_duty)
{
    if (duties == nullptr || velocities == nullptr || gain == nullptr || friction_duty == nullptr)
        return false;
    if (count < 2)
        return false;

    float sum_duty = 0.0f;
    float sum_velocity = 0.0f;
    for (size_t i = 0; i < count; ++i)
    {
        if (std::fabs(velocities[i]) < MIN_MOVING_VELOCITY_RAD_S)
            return false;
        sum_duty += duties[i];
        sum_velocity += velocities[i];
    }
    const float mean_duty = sum_duty / static_cast<float>(count);
    const float mean_velocity = sum_velocity / static_cast<float>(count);

    float variance = 0.0f;
    float covariance = 0.0f;
    for (size_t i = 0; i < count; ++i)
    {
        variance += (duties[i] - mean_duty) * (duties[i] - mean_duty);
        covariance += (duties[i] - mean_duty) * (velocities[i] - mean_velocity);
    }
    if (variance <= 0.0f)
        return false;

    const float slope = covariance / variance;
    if (slope < MIN_PLANT_GAIN || slope > MAX_PLANT_GAIN)
        return false;

    const float offset = mean_velocity - slope * mean_duty;
    // Duty of the fitted static curve at zero velocity. It is signed: the
    // curve can cross the axis on either side, and the sign tells the
    // feedforward whether to add or remove duty at the operating point.
    *gain = slope;
    *friction_duty = std::clamp(-offset / slope, -FEEDFORWARD_MAX, FEEDFORWARD_MAX);
    return true;
}

bool fit_step_dynamics(
  const float * times_s, const float * velocities, size_t count, float * tau_s, float * dead_time_s)
{
    if (times_s == nullptr || velocities == nullptr || tau_s == nullptr || dead_time_s == nullptr)
        return false;
    if (count < 8)
        return false;

    const size_t baseline_end = std::max<size_t>(2, static_cast<size_t>(BASELINE_FRACTION * count));
    const size_t final_begin =
      count - std::max<size_t>(3, static_cast<size_t>(FINAL_FRACTION * count));
    const float baseline = mean_of(velocities, 0, baseline_end);
    const float final_value = mean_of(velocities, final_begin, count);
    const float delta = final_value - baseline;
    if (std::fabs(delta) < MIN_STEP_DELTA_RAD_S)
        return false;

    float dead = 0.0f;
    float tau_crossing = 0.0f;
    if (!first_crossing(times_s, velocities, count, baseline, delta, DEAD_TIME_FRACTION, &dead))
        return false;
    if (!first_crossing(times_s, velocities, count, baseline, delta, TAU_FRACTION, &tau_crossing))
        return false;

    const float tau = tau_crossing - dead;
    if (tau <= 0.0f || tau > 2.0f)
        return false;

    *tau_s = tau;
    *dead_time_s = std::clamp(dead, 0.0f, 1.0f);
    return true;
}

StepMetrics analyze_step(const float * times_s,
                         const float * velocities,
                         size_t count,
                         float target_rad_s)
{
    StepMetrics metrics;
    if (times_s == nullptr || velocities == nullptr || count < 8 || target_rad_s <= 0.0f)
        return metrics;

    // The encoder velocity carries spikes and quantization ripple at low
    // speeds; reject spikes with a median filter, then average before locating
    // the peak and the rise crossings.
    const auto smoothed = [&](size_t index) {
        const size_t begin = (index > SMOOTHING_HALF_WINDOW) ? index - SMOOTHING_HALF_WINDOW : 0;
        const size_t end = std::min(count, index + SMOOTHING_HALF_WINDOW + 1);
        float sum = 0.0f;
        for (size_t i = begin; i < end; ++i) sum += median_of_three(velocities, count, i);
        return sum / static_cast<float>(end - begin);
    };

    const size_t final_samples = std::min(FINAL_WINDOW_SAMPLES, std::max<size_t>(5, count / 4));
    float final_value = 0.0f;
    for (size_t i = count - final_samples; i < count; ++i) final_value += smoothed(i);
    final_value /= static_cast<float>(final_samples);

    float peak = smoothed(0);
    for (size_t i = 1; i < count; ++i) peak = std::max(peak, smoothed(i));

    metrics.final_rad_s = final_value;
    metrics.target_rad_s = target_rad_s;
    metrics.overshoot_pct = std::max(0.0f, (peak - target_rad_s) / target_rad_s * 100.0f);
    metrics.ss_error_pct = std::fabs(final_value - target_rad_s) / target_rad_s * 100.0f;

    float rise10 = 0.0f;
    float rise90 = 0.0f;
    bool has10 = false;
    bool has90 = false;
    for (size_t i = 0; i < count; ++i)
    {
        const float value = smoothed(i);
        if (!has10 && value >= 0.10f * target_rad_s)
        {
            rise10 = times_s[i];
            has10 = true;
        }
        if (!has90 && value >= 0.90f * target_rad_s)
        {
            rise90 = times_s[i];
            has90 = true;
            break;
        }
    }
    if (!has10 || !has90 || rise90 < rise10)
        return metrics;

    metrics.rise_ms = (rise90 - rise10) * 1000.0f;
    metrics.valid = true;
    return metrics;
}

float choose_validation_target(const PlantModel & plant)
{
    if (!plant.valid || plant.gain_rad_s_per_duty <= 0.0f)
        return VALIDATION_MIN_RAD_S;

    const float velocity_ff = 1.0f / plant.gain_rad_s_per_duty;
    const float duty_available =
      VALIDATION_DUTY_FRACTION * DUTY_MAX - DUTY_RESERVE - plant.friction_duty;
    float target = duty_available / velocity_ff;
    // The two-point model is only trustworthy inside the measured range; the
    // validation step stays below the highest identified operating speed.
    if (plant.max_velocity_rad_s > 1.0f)
        target = std::min(target, 0.80f * plant.max_velocity_rad_s);
    // A saturated curve (both steps land close together) has almost no slope:
    // validating near the lower operating point keeps the feedforward honest.
    if (plant.max_velocity_rad_s > 1.0f && plant.min_velocity_rad_s > 0.5f &&
        (plant.max_velocity_rad_s - plant.min_velocity_rad_s) < 0.25f * plant.max_velocity_rad_s)
    {
        target = std::min(target, 0.55f * plant.min_velocity_rad_s);
    }
    return std::clamp(target, VALIDATION_MIN_RAD_S, VALIDATION_MAX_RAD_S);
}

Gains synthesize(const PlantModel & plant, float validation_target_rad_s)
{
    Gains gains;
    if (!plant.valid || plant.gain_rad_s_per_duty <= MIN_PLANT_GAIN)
        return gains;

    const float velocity_ff = 1.0f / plant.gain_rad_s_per_duty;
    gains.velocity_ff = std::clamp(velocity_ff, 0.5f, VELOCITY_FF_MAX);
    // The signed static-curve intercept: the feedforward of the operating point.
    gains.feedforward = std::clamp(plant.friction_duty, -FEEDFORWARD_MAX, FEEDFORWARD_MAX);
    gains.static_friction = std::clamp(plant.breakaway_duty, 0.0f, FEEDFORWARD_MAX);

    const float tau = std::clamp(plant.tau_s, 0.01f, 2.0f);
    (void)tau;
    // The feedforward carries the setpoint, so P only trims residual errors.
    // The dimensionless loop gain Kp * plant_gain stays small: the measured
    // velocity lags the real one, and a larger gain makes the step overshoot.
    float kp = LOOP_GAIN_NOMINAL * gains.velocity_ff;
    if (validation_target_rad_s > 0.1f)
    {
        // Do not demand more proportional duty than the actuator has left at
        // the start of the step: a saturated start would overshoot.
        const float feedforward_duty =
          gains.feedforward + gains.velocity_ff * validation_target_rad_s;
        const float headroom = PROPORTIONAL_DUTY_FRACTION * DUTY_MAX - std::fabs(feedforward_duty);
        const float kp_at_saturation = headroom / validation_target_rad_s;
        if (kp_at_saturation > 0.0f)
            kp = std::min(kp, kp_at_saturation);
    }
    gains.kp = std::clamp(kp,
                          LOOP_GAIN_MIN * gains.velocity_ff,
                          std::min(KP_MAX, LOOP_GAIN_REFINE_MAX * gains.velocity_ff));
    // Start without integral action: the velocity feedforward carries the
    // setpoint, so P alone produces the monotone first-order response.
    gains.ki = 0.0f;
    gains.kd = 0.0f;
    return gains;
}

Gains refine(const PlantModel & plant, const Gains & current, const StepMetrics & metrics)
{
    Gains next = current;
    (void)plant;
    if (!metrics.valid || metrics.overshoot_pct > OVERSHOOT_PCT_LIMIT)
    {
        // Too aggressive: trade speed for damping. When P is already at its
        // floor the overshoot comes from the feedforward being too high for
        // this operating point, so trim the velocity feedforward instead.
        const float kp_floor = LOOP_GAIN_MIN * std::max(next.velocity_ff, 0.1f);
        if (next.kp > 1.05f * kp_floor)
            next.kp *= 0.6f;
        else
            next.velocity_ff *= 0.92f;
    }
    else if (metrics.ss_error_pct > SS_ERROR_PCT_LIMIT)
    {
        // The steady-state error is a feedforward error: trim the velocity
        // feedforward, not the integral term (integral action winds up during
        // the measurement lag and adds overshoot).
        if (metrics.target_rad_s > 0.1f)
        {
            const float relative_error =
              (metrics.final_rad_s - metrics.target_rad_s) / metrics.target_rad_s;
            const float loop_gain = next.kp / std::max(next.velocity_ff, 0.1f);
            const float trim = std::clamp((1.0f + loop_gain) * relative_error, -0.2f, 0.2f);
            next.velocity_ff *= (1.0f - trim);
        }
    }
    else if (metrics.rise_ms > RISE_MS_LIMIT)
    {
        next.kp *= 1.3f;
    }

    const float feedforward_min = LOOP_GAIN_MIN * std::max(next.velocity_ff, 0.1f);
    const float feedforward_max = LOOP_GAIN_REFINE_MAX * std::max(next.velocity_ff, 0.1f);
    next.kp = std::clamp(next.kp, feedforward_min, std::min(KP_MAX, feedforward_max));
    next.velocity_ff = std::clamp(next.velocity_ff, 0.5f, VELOCITY_FF_MAX);
    next.ki = 0.0f;
    next.kd = 0.0f;
    return next;
}

bool accepts(const StepMetrics & metrics)
{
    return metrics.valid && metrics.rise_ms > 0.0f && metrics.rise_ms <= RISE_MS_LIMIT &&
           metrics.overshoot_pct <= OVERSHOOT_PCT_LIMIT &&
           metrics.ss_error_pct <= SS_ERROR_PCT_LIMIT;
}

bool accepts_best_effort(const StepMetrics & metrics)
{
    return metrics.valid && metrics.rise_ms > 0.0f && metrics.rise_ms <= FALLBACK_RISE_MS_LIMIT &&
           metrics.overshoot_pct <= FALLBACK_OVERSHOOT_PCT_LIMIT &&
           metrics.ss_error_pct <= FALLBACK_SS_ERROR_PCT_LIMIT;
}

float step_cost(const StepMetrics & metrics)
{
    if (!metrics.valid)
        return 1.0e6f;
    return 2.0f * metrics.overshoot_pct + metrics.ss_error_pct + metrics.rise_ms / 100.0f;
}

}  // namespace tuning
