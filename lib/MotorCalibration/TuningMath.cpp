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
// The validation step must sit in the linear part of the speed/duty curve.
// Near the lower branch knee the two-point plant model is poor, the integral
// is faded out at those targets and the steady-state error grows; the step is
// therefore placed close to the identified operating point instead.
constexpr float VALIDATION_DUTY_FRACTION = 0.90f;
// The P gain is capped so u_ff plus the initial P term stays inside the
// validation duty budget (0.90 * DUTY_MAX) without starting saturated.
constexpr float PROPORTIONAL_DUTY_FRACTION = 0.90f;
// At the step start the P term lifts the equilibrium speed by a*Kp (relative
// overshoot). With the lower-lag angle estimator the loop supports stronger
// proportional rejection before overshooting.
constexpr float LOOP_GAIN_NOMINAL = 0.14f;
constexpr float LOOP_GAIN_MIN = 0.02f;
constexpr float LOOP_GAIN_REFINE_MAX = 0.35f;
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

bool fit_plant_two_points(float velocity_high,
                          float duty_high,
                          float velocity_low,
                          float duty_low,
                          float * gain,
                          float * friction_duty)
{
    if (gain == nullptr || friction_duty == nullptr)
        return false;
    const float duty_span = duty_high - duty_low;
    if (std::fabs(duty_span) < 1.0f)
        return false;
    const float slope = (velocity_high - velocity_low) / duty_span;
    if (slope < MIN_PLANT_GAIN || slope > MAX_PLANT_GAIN)
        return false;
    // Duty of the static curve at zero velocity, signed by the branch.
    *gain = slope;
    *friction_duty = std::clamp(duty_low - velocity_low / slope, -FEEDFORWARD_MAX, FEEDFORWARD_MAX);
    return true;
}

void median_filter_step(const float * input, float * output, size_t count)
{
    for (size_t i = 0; i < count; ++i) output[i] = median_of_three(input, count, i);
}

bool fit_step_dynamics(
  const float * times_s, const float * velocities, size_t count, float * tau_s, float * dead_time_s)
{
    if (times_s == nullptr || velocities == nullptr || tau_s == nullptr || dead_time_s == nullptr)
        return false;
    if (count < 8)
        return false;

    const float sample_count = static_cast<float>(count);
    const size_t baseline_end =
      std::max<size_t>(2, static_cast<size_t>(BASELINE_FRACTION * sample_count));
    const size_t final_begin =
      count - std::max<size_t>(3, static_cast<size_t>(FINAL_FRACTION * sample_count));
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

SettledSpeed measure_settled_speed(const float * velocities, size_t count)
{
    constexpr size_t WINDOW_SAMPLES = 10;
    SettledSpeed result;
    const size_t windows = count / WINDOW_SAMPLES;
    if (velocities == nullptr || windows < 3)
        return result;

    double sum = 0.0;
    double squared_sum = 0.0;
    for (size_t window = 0; window < windows; ++window)
    {
        const size_t first = count - (windows - window) * WINDOW_SAMPLES;
        const double mean = mean_of(velocities, first, first + WINDOW_SAMPLES);
        if (!std::isfinite(mean))
            return result;
        if (window == 0 || mean < result.min_rad_s)
            result.min_rad_s = static_cast<float>(mean);
        if (window == 0 || mean > result.max_rad_s)
            result.max_rad_s = static_cast<float>(mean);
        sum += mean;
        squared_sum += mean * mean;
    }
    const double average = sum / static_cast<double>(windows);
    result.mean_rad_s = static_cast<float>(average);
    result.ripple_rad_s = static_cast<float>(
      std::sqrt(std::max(0.0, squared_sum / static_cast<double>(windows) - average * average)));
    result.valid = true;
    return result;
}

bool tracks_speed(const SettledSpeed & steady, float target_rad_s, float mean_error_fraction)
{
    const float magnitude = std::fabs(target_rad_s);
    const float slowest = target_rad_s > 0.0f ? steady.min_rad_s : -steady.max_rad_s;
    return steady.valid && magnitude > 0.0f &&
           std::fabs(steady.mean_rad_s - target_rad_s) <= mean_error_fraction * magnitude &&
           slowest >= 0.10f * magnitude;
}

SettledSpeed measure_settled_travel(const float * times_s,
                                    const float * angles_rad,
                                    size_t count,
                                    float window_s)
{
    SettledSpeed result;
    if (!times_s || !angles_rad || count < 2 || !(window_s > 0.0f) || !std::isfinite(times_s[0]) ||
        !std::isfinite(angles_rad[0]))
        return result;

    size_t first = 0;
    size_t windows = 0;
    double sum = 0.0;
    double square_sum = 0.0;
    for (size_t last = 1; last < count; ++last)
    {
        if (!std::isfinite(times_s[last]) || !std::isfinite(angles_rad[last]) ||
            times_s[last] <= times_s[last - 1])
            return {};
        if (times_s[last] - times_s[0] < window_s)
            continue;
        while (times_s[last] - times_s[first + 1] >= window_s) ++first;
        // Interpolate the angle at the exact window boundary so that scheduler
        // jitter cannot hide or create a peak.
        const float boundary_s = times_s[last] - window_s;
        const float fraction =
          (boundary_s - times_s[first]) / (times_s[first + 1] - times_s[first]);
        const float start_angle =
          angles_rad[first] + fraction * (angles_rad[first + 1] - angles_rad[first]);
        const float velocity = (angles_rad[last] - start_angle) / window_s;
        if (windows == 0 || velocity < result.min_rad_s)
            result.min_rad_s = velocity;
        if (windows == 0 || velocity > result.max_rad_s)
            result.max_rad_s = velocity;
        sum += velocity;
        square_sum += static_cast<double>(velocity) * velocity;
        ++windows;
    }
    if (windows == 0)
        return {};

    const double window_mean = sum / static_cast<double>(windows);
    result.mean_rad_s = (angles_rad[count - 1] - angles_rad[0]) / (times_s[count - 1] - times_s[0]);
    result.ripple_rad_s = static_cast<float>(std::sqrt(
      std::max(0.0, square_sum / static_cast<double>(windows) - window_mean * window_mean)));
    result.valid = true;
    return result;
}

float peak_speed_error(const SettledSpeed & steady, float target_rad_s)
{
    return std::max(std::fabs(steady.min_rad_s - target_rad_s),
                    std::fabs(steady.max_rad_s - target_rad_s));
}

uint8_t unmatched_speed_mask(const std::array<float, 8> & mean_speeds, float target_rad_s)
{
    const float tolerance = MATCHED_SPEED_FRACTION * target_rad_s;
    uint8_t unmatched = 0;
    size_t lowest = 0;
    size_t highest = 0;
    for (size_t i = 0; i < mean_speeds.size(); ++i)
    {
        const float mean = mean_speeds[i];
        if (!std::isfinite(mean) || std::fabs(mean - target_rad_s) > tolerance)
            unmatched |= static_cast<uint8_t>(1U << i);
        if (mean < mean_speeds[lowest])
            lowest = i;
        if (mean > mean_speeds[highest])
            highest = i;
    }
    if (unmatched == 0 && mean_speeds[highest] - mean_speeds[lowest] > tolerance)
    {
        const float below = target_rad_s - mean_speeds[lowest];
        const float above = mean_speeds[highest] - target_rad_s;
        unmatched = static_cast<uint8_t>(1U << (below > above ? lowest : highest));
    }
    return unmatched;
}

StepMetrics analyze_step(const float * times_s,
                         const float * velocities,
                         size_t count,
                         float target_rad_s)
{
    StepMetrics metrics;
    if (times_s == nullptr || velocities == nullptr || count < 8 || target_rad_s == 0.0f)
        return metrics;

    // The response to a negative target is the mirror of the positive one, so
    // measure in the direction frame: rise, overshoot and steady-state error
    // keep the same definitions in both directions. The final velocity stays
    // signed.
    const float direction = (target_rad_s > 0.0f) ? 1.0f : -1.0f;
    const float magnitude = std::fabs(target_rad_s);

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

    float peak = direction * smoothed(0);
    for (size_t i = 1; i < count; ++i) peak = std::max(peak, direction * smoothed(i));

    metrics.final_rad_s = final_value;
    metrics.target_rad_s = target_rad_s;
    metrics.overshoot_pct = std::max(0.0f, (peak - magnitude) / magnitude * 100.0f);
    metrics.ss_error_pct = std::fabs(direction * final_value - magnitude) / magnitude * 100.0f;
    const size_t ripple_samples = std::min<size_t>(100, count / 2);
    const auto steady = measure_settled_speed(velocities + count - ripple_samples, ripple_samples);
    if (steady.valid)
        metrics.settled_ripple_pct = 100.0f * steady.ripple_rad_s / magnitude;

    float rise10 = 0.0f;
    float rise90 = 0.0f;
    bool has10 = false;
    bool has90 = false;
    for (size_t i = 0; i < count; ++i)
    {
        const float value = direction * smoothed(i);
        if (!has10 && value >= 0.10f * magnitude)
        {
            rise10 = times_s[i];
            has10 = true;
        }
        if (!has90 && value >= 0.90f * magnitude)
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
    // The intercept is signed by the branch (positive forward, negative
    // reverse): its magnitude consumes the same duty budget either way.
    const float duty_available =
      VALIDATION_DUTY_FRACTION * DUTY_MAX - DUTY_RESERVE - std::fabs(plant.friction_duty);
    float target = duty_available / velocity_ff;
    // The two-point model is only trustworthy inside the measured range; the
    // validation step stays below the highest identified operating speed.
    if (plant.max_velocity_rad_s > 1.0f)
        target = std::min(target, 0.95f * plant.max_velocity_rad_s);
    // A saturated curve (both steps land close together) has almost no slope:
    // validating near the lower operating point keeps the feedforward honest.
    if (plant.max_velocity_rad_s > 1.0f && plant.min_velocity_rad_s > 0.5f &&
        (plant.max_velocity_rad_s - plant.min_velocity_rad_s) < 0.25f * plant.max_velocity_rad_s)
    {
        target = std::min(target, 0.55f * plant.min_velocity_rad_s);
    }
    return std::clamp(target, VALIDATION_MIN_RAD_S, VALIDATION_MAX_RAD_S);
}

Gains synthesize(const PlantModel & forward_plant,
                 const PlantModel & reverse_plant,
                 float validation_target_rad_s)
{
    Gains gains;
    if (!forward_plant.valid || !reverse_plant.valid ||
        forward_plant.gain_rad_s_per_duty <= MIN_PLANT_GAIN ||
        reverse_plant.gain_rad_s_per_duty <= MIN_PLANT_GAIN)
        return gains;

    // One feedforward branch per direction: the velocity feedforward carries
    // the setpoint, the signed static-curve intercept the kinetic friction.
    gains.velocity_ff = std::clamp(1.0f / forward_plant.gain_rad_s_per_duty, 0.5f, VELOCITY_FF_MAX);
    gains.feedforward = std::clamp(forward_plant.friction_duty, -FEEDFORWARD_MAX, FEEDFORWARD_MAX);
    gains.static_friction = std::clamp(forward_plant.breakaway_duty, 0.0f, FEEDFORWARD_MAX);
    gains.velocity_ff_rev =
      std::clamp(1.0f / reverse_plant.gain_rad_s_per_duty, 0.5f, VELOCITY_FF_MAX);
    gains.feedforward_rev =
      std::clamp(reverse_plant.friction_duty, -FEEDFORWARD_MAX, FEEDFORWARD_MAX);
    gains.static_friction_rev = std::clamp(reverse_plant.breakaway_duty, 0.0f, FEEDFORWARD_MAX);

    // The common Kp serves both directions: bound it by the larger velocity
    // feedforward, because the dimensionless loop gain Kp * plant_gain stays
    // small (the measured velocity lags the real one and a larger gain makes
    // the step overshoot).
    const float velocity_ff_max = std::max(gains.velocity_ff, gains.velocity_ff_rev);
    float kp = LOOP_GAIN_NOMINAL * velocity_ff_max;
    if (validation_target_rad_s > 0.1f)
    {
        // Do not demand more proportional duty than the actuator has left at
        // the start of the step in either direction: a saturated start would
        // overshoot. The reverse step runs at the negative target, so its
        // feedforward duty is the intercept minus the velocity term.
        const float forward_duty = gains.feedforward + gains.velocity_ff * validation_target_rad_s;
        const float reverse_duty =
          gains.feedforward_rev - gains.velocity_ff_rev * validation_target_rad_s;
        const float headroom = PROPORTIONAL_DUTY_FRACTION * DUTY_MAX -
                               std::max(std::fabs(forward_duty), std::fabs(reverse_duty));
        const float kp_at_saturation = headroom / validation_target_rad_s;
        if (kp_at_saturation > 0.0f)
            kp = std::min(kp, kp_at_saturation);
    }
    gains.kp = std::clamp(kp,
                          LOOP_GAIN_MIN * velocity_ff_max,
                          std::min(KP_MAX, LOOP_GAIN_REFINE_MAX * velocity_ff_max));
    // The integral gain starts proportional to P: it closes the low-speed
    // steady-state error the feedforward cannot trim. The controller applies it
    // outside FastPID, which runs with ki = 0.
    gains.ki = std::clamp(KI_RATIO * gains.kp, KI_MIN_CAL, KI_MAX_CAL);
    gains.kd = 0.0f;
    return gains;
}

Gains refine(const Gains & current,
             const StepMetrics & forward_metrics,
             const StepMetrics & reverse_metrics)
{
    Gains next = current;
    const bool forward_valid = forward_metrics.valid;
    const bool reverse_valid = reverse_metrics.valid;
    const bool forward_overshoot =
      !forward_valid || forward_metrics.overshoot_pct > OVERSHOOT_PCT_LIMIT;
    const bool reverse_overshoot =
      !reverse_valid || reverse_metrics.overshoot_pct > OVERSHOOT_PCT_LIMIT;

    if (forward_overshoot || reverse_overshoot)
    {
        // Too aggressive in at least one direction: trade speed for damping.
        // The Kp is common, so damping it helps both; when it is already at
        // its floor the overshoot comes from that direction's feedforward
        // being too high for its operating point, so trim that branch too.
        const float velocity_ff_max = std::max(next.velocity_ff, next.velocity_ff_rev);
        const float kp_floor = LOOP_GAIN_MIN * std::max(velocity_ff_max, 0.1f);
        if (next.kp > 1.05f * kp_floor)
        {
            next.kp *= 0.6f;
        }
        else
        {
            // P is already at its floor: the overshoot comes from too much
            // integral action for the operating point, so back it off.
            next.ki = std::max(KI_MIN_CAL, next.ki * 0.6f);
        }
    }
    else if (forward_metrics.ss_error_pct > SS_ERROR_PCT_LIMIT ||
             reverse_metrics.ss_error_pct > SS_ERROR_PCT_LIMIT)
    {
        if (next.ki < KI_MAX_CAL - 1.0e-3f)
        {
            // The steady-state error is the integral's job: raise the common
            // integral gain before touching the per-direction feedforward.
            next.ki = std::min(KI_MAX_CAL, next.ki * 1.7f);
        }
        else
        {
            // The integral is already at its cap: trim the velocity
            // feedforward of the direction that misses the target. The signed
            // error and target keep the trim direction correct in both
            // rotation directions.
            if (forward_metrics.ss_error_pct > SS_ERROR_PCT_LIMIT &&
                forward_metrics.target_rad_s > 0.1f)
            {
                const float relative_error =
                  (forward_metrics.final_rad_s - forward_metrics.target_rad_s) /
                  forward_metrics.target_rad_s;
                const float loop_gain = next.kp / std::max(next.velocity_ff, 0.1f);
                const float trim = std::clamp((1.0f + loop_gain) * relative_error, -0.2f, 0.2f);
                next.velocity_ff *= (1.0f - trim);
            }
            if (reverse_metrics.ss_error_pct > SS_ERROR_PCT_LIMIT &&
                std::fabs(reverse_metrics.target_rad_s) > 0.1f)
            {
                const float relative_error =
                  (reverse_metrics.final_rad_s - reverse_metrics.target_rad_s) /
                  reverse_metrics.target_rad_s;
                const float loop_gain = next.kp / std::max(next.velocity_ff_rev, 0.1f);
                const float trim = std::clamp((1.0f + loop_gain) * relative_error, -0.2f, 0.2f);
                next.velocity_ff_rev *= (1.0f - trim);
            }
        }
    }
    else if ((forward_valid && forward_metrics.rise_ms > RISE_MS_LIMIT) ||
             (reverse_valid && reverse_metrics.rise_ms > RISE_MS_LIMIT))
    {
        next.kp *= 1.3f;
    }

    const float velocity_ff_max = std::max(next.velocity_ff, next.velocity_ff_rev);
    const float kp_min = LOOP_GAIN_MIN * std::max(velocity_ff_max, 0.1f);
    const float kp_max = LOOP_GAIN_REFINE_MAX * std::max(velocity_ff_max, 0.1f);
    next.kp = std::clamp(next.kp, kp_min, std::min(KP_MAX, kp_max));
    next.velocity_ff = std::clamp(next.velocity_ff, 0.5f, VELOCITY_FF_MAX);
    next.velocity_ff_rev = std::clamp(next.velocity_ff_rev, 0.5f, VELOCITY_FF_MAX);
    next.ki = std::clamp(next.ki, KI_MIN_CAL, KI_MAX_CAL);
    next.kd = 0.0f;
    return next;
}

bool accepts(const StepMetrics & metrics)
{
    return metrics.valid && metrics.rise_ms > 0.0f && metrics.rise_ms <= RISE_MS_LIMIT &&
           metrics.overshoot_pct <= OVERSHOOT_PCT_LIMIT &&
           metrics.ss_error_pct <= SS_ERROR_PCT_LIMIT &&
           metrics.settled_ripple_pct <= SETTLED_RIPPLE_PCT_LIMIT;
}

bool accepts_best_effort(const StepMetrics & metrics)
{
    return metrics.valid && metrics.rise_ms > 0.0f && metrics.rise_ms <= FALLBACK_RISE_MS_LIMIT &&
           metrics.overshoot_pct <= FALLBACK_OVERSHOOT_PCT_LIMIT &&
           metrics.ss_error_pct <= FALLBACK_SS_ERROR_PCT_LIMIT &&
           metrics.settled_ripple_pct <= SETTLED_RIPPLE_PCT_LIMIT;
}

bool accepts_pair(const StepMetrics & forward_metrics, const StepMetrics & reverse_metrics)
{
    return accepts(forward_metrics) && accepts(reverse_metrics);
}

bool accepts_best_effort_pair(const StepMetrics & forward_metrics,
                              const StepMetrics & reverse_metrics)
{
    return accepts_best_effort(forward_metrics) && accepts_best_effort(reverse_metrics);
}

bool accepts_low_speed(const StepMetrics & metrics)
{
    return metrics.valid && metrics.rise_ms > 0.0f && metrics.rise_ms <= LOW_SPEED_RISE_MS_LIMIT &&
           metrics.overshoot_pct <= LOW_SPEED_OVERSHOOT_PCT_LIMIT &&
           metrics.ss_error_pct <= LOW_SPEED_SS_ERROR_PCT_LIMIT;
}

Gains adjust_low_speed_integral(const Gains & current,
                                const StepMetrics & forward_metrics,
                                const StepMetrics & reverse_metrics)
{
    Gains next = current;
    const bool valid = forward_metrics.valid && reverse_metrics.valid;
    const float overshoot = std::max(forward_metrics.valid ? forward_metrics.overshoot_pct : 0.0f,
                                     reverse_metrics.valid ? reverse_metrics.overshoot_pct : 0.0f);
    if (!valid || overshoot > LOW_SPEED_OVERSHOOT_PCT_LIMIT)
    {
        // Too much integral action for a low-speed step: back it off.
        next.ki = std::max(KI_MIN_CAL, next.ki * 0.6f);
    }
    else
    {
        // Steady-state error (or a slow approach) needs more integral: it
        // shortens the remaining error without touching the validated P gain.
        next.ki = std::min(KI_MAX_CAL, next.ki * 1.7f);
    }
    next.ki = std::clamp(next.ki, KI_MIN_CAL, KI_MAX_CAL);
    next.kd = 0.0f;
    return next;
}

float step_cost(const StepMetrics & metrics)
{
    if (!metrics.valid)
        return 1.0e6f;
    return 2.0f * metrics.overshoot_pct + metrics.ss_error_pct + metrics.rise_ms / 100.0f +
           2.0f * metrics.settled_ripple_pct;
}

uint16_t calibrated_max_rpm(const uint16_t * wheel_max_rpm, size_t count)
{
    if (wheel_max_rpm == nullptr || count == 0)
        return 0;
    uint16_t slowest = wheel_max_rpm[0];
    for (size_t i = 1; i < count; ++i) slowest = std::min(slowest, wheel_max_rpm[i]);
    // A missing measurement keeps the stored limit: no common limit can be
    // derived from an incomplete run.
    if (slowest == 0)
        return 0;
    return slowest;
}

}  // namespace tuning
