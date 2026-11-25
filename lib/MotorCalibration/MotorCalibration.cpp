#include "MotorCalibration.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/task.h>

#include <algorithm>
#include <cmath>

#include "AS5600.h"
#include "BL48250.h"
#include "WheelController.h"
#include "wheel_state_estimator.h"

static const char * TAG = "MotorCalibration";

static const char * WHEEL_NAMES[4] = {"M1", "M2", "M3", "M4"};
static constexpr uint8_t ALL_WHEELS_MASK = (1U << NUM_ENC_CHANNELS) - 1U;
static constexpr std::array<uint32_t, 4> STOP_DUTIES = {};
static constexpr std::array<uint8_t, 4> CLOCKWISE_DIRECTIONS = {config::driver::MOTOR_CW,
                                                                config::driver::MOTOR_CW,
                                                                config::driver::MOTOR_CW,
                                                                config::driver::MOTOR_CW};

static const char * error_name(MotorCalibration::EncoderError error)
{
    switch (error)
    {
        case MotorCalibration::EncoderError::NONE:
            return "none";
        case MotorCalibration::EncoderError::NO_MOVEMENT:
            return "no movement";
        case MotorCalibration::EncoderError::AMBIGUOUS_ANALOG:
            return "ambiguous analog input";
        case MotorCalibration::EncoderError::AMBIGUOUS_I2C:
            return "ambiguous mux channel";
        case MotorCalibration::EncoderError::BAD_MAPPING:
            return "invalid wiring map";
        case MotorCalibration::EncoderError::ABORTED:
            return "aborted";
        case MotorCalibration::EncoderError::I2C:
            return "I2C read failures";
        case MotorCalibration::EncoderError::MAGNET_TOO_CLOSE:
            return "magnet too close";
        case MotorCalibration::EncoderError::MAGNET_TOO_FAR:
            return "magnet too far or missing";
    }
    return "unknown";
}

/**
 * @brief Per-input movement score: the spin spread above the resting spread.
 *
 * A wheel that coasts on from the previous spin keeps its input at the full
 * range in both windows, so its score collapses to zero while the input of the
 * wheel being spun goes from its noise floor to the full range.
 */
static std::array<uint16_t, 4> movement_scores(const std::array<uint16_t, 4> & spin,
                                               const std::array<uint16_t, 4> & baseline)
{
    std::array<uint16_t, 4> scores = {};
    for (size_t i = 0; i < 4; ++i)
    {
        if (spin[i] > baseline[i])
            scores[i] = static_cast<uint16_t>(spin[i] - baseline[i]);
    }
    return scores;
}

MotorCalibration & MotorCalibration::get_instance()
{
    static MotorCalibration instance;
    return instance;
}

MotorCalibration::MotorCalibration() : m_status_mutex(xSemaphoreCreateMutex())
{
}

bool MotorCalibration::reserve_routine(Routine routine)
{
    portENTER_CRITICAL(&m_run_mux);
    const bool available = m_routine == Routine::NONE;
    if (available)
    {
        m_routine = routine;
        m_abort_requested = false;
    }
    portEXIT_CRITICAL(&m_run_mux);
    return available;
}

void MotorCalibration::release_routine()
{
    portENTER_CRITICAL(&m_run_mux);
    m_routine = Routine::NONE;
    m_abort_requested = false;
    portEXIT_CRITICAL(&m_run_mux);
}

void MotorCalibration::abort(Routine routine)
{
    portENTER_CRITICAL(&m_run_mux);
    if (m_routine != Routine::NONE && (routine == Routine::NONE || m_routine == routine))
        m_abort_requested = true;
    portEXIT_CRITICAL(&m_run_mux);
}

bool MotorCalibration::abort_requested() const
{
    portENTER_CRITICAL(&m_run_mux);
    const bool aborted = m_abort_requested;
    portEXIT_CRITICAL(&m_run_mux);
    return aborted;
}

bool MotorCalibration::is_running() const
{
    portENTER_CRITICAL(&m_run_mux);
    const bool running = m_routine != Routine::NONE;
    portEXIT_CRITICAL(&m_run_mux);
    return running;
}

esp_err_t MotorCalibration::start_encoder_alignment()
{
    if (!reserve_routine(Routine::ENCODER_ALIGNMENT))
        return ESP_ERR_INVALID_STATE;

    EncoderStatus status;
    status.state = State::RUNNING;
    publish_encoder_status(status);
    if (xTaskCreate(encoder_task_wrapper, "motor_cal", 4096, this, 4, nullptr) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create the encoder alignment task.");
        status.state = State::IDLE;
        publish_encoder_status(status);
        release_routine();
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/**
 * @brief Returns a thread-safe copy of the latest status.
 */
MotorCalibration::EncoderStatus MotorCalibration::get_encoder_status() const
{
    EncoderStatus status;
    if (m_status_mutex == nullptr)
        return status;

    xSemaphoreTake(m_status_mutex, portMAX_DELAY);
    status = m_encoder_status;
    xSemaphoreGive(m_status_mutex);
    return status;
}

/**
 * @brief Replaces the published status snapshot.
 */
void MotorCalibration::publish_encoder_status(const EncoderStatus & status)
{
    if (m_status_mutex == nullptr)
        return;

    xSemaphoreTake(m_status_mutex, portMAX_DELAY);
    m_encoder_status = status;
    xSemaphoreGive(m_status_mutex);
}

void MotorCalibration::encoder_task_wrapper(void * arg)
{
    static_cast<MotorCalibration *>(arg)->run_encoder_alignment();
}

/**
 * @brief Resets a sample window so the next sample seeds every extreme.
 */
void MotorCalibration::reset_window(WindowStats * window)
{
    window->analog_min.fill(UINT16_MAX);
    window->analog_max.fill(0);
    window->i2c_min.fill(UINT16_MAX);
    window->i2c_max.fill(0);
}

/**
 * @brief Samples every analog input and every mux channel once.
 *
 * Failed I2C reads are counted and do not update the I2C extremes: a broken
 * read reports zero and would otherwise fake movement.
 */
void MotorCalibration::sample_once(WindowStats * window,
                                   std::array<EncoderError, 4> * magnet_warning)
{
    WheelStateEstimator & estimator = WheelStateEstimator::get_instance();

    const std::array<uint16_t, 4> analog = estimator.get_raw_values();
    for (size_t i = 0; i < 4; ++i)
    {
        window->analog_min[i] = std::min(window->analog_min[i], analog[i]);
        window->analog_max[i] = std::max(window->analog_max[i], analog[i]);
    }

    std::array<uint16_t, 4> angles = {};
    std::array<EncoderChannel::MagnetField, 4> magnet_fields = {};
    ++m_i2c_attempts;
    const esp_err_t angle_result =
      estimator.read_i2c_raw_angles(&angles, magnet_warning != nullptr ? &magnet_fields : nullptr);
    if (magnet_warning != nullptr)
    {
        // Keep a warning once observed so a brief change is still visible.
        for (size_t mux = 0; mux < magnet_fields.size(); ++mux)
        {
            if (magnet_fields[mux] == EncoderChannel::MagnetField::STRONG)
                (*magnet_warning)[mux] = EncoderError::MAGNET_TOO_CLOSE;
            else if (magnet_fields[mux] == EncoderChannel::MagnetField::WEAK)
                (*magnet_warning)[mux] = EncoderError::MAGNET_TOO_FAR;
        }
    }
    if (angle_result != ESP_OK)
    {
        ++m_i2c_failures;
        return;
    }

    for (size_t i = 0; i < 4; ++i)
    {
        window->i2c_min[i] = std::min(window->i2c_min[i], angles[i]);
        window->i2c_max[i] = std::max(window->i2c_max[i], angles[i]);
    }
}

/**
 * @brief Computes the minimum-to-maximum spread of every input in a window.
 *
 * The spread is used instead of sample-to-sample deltas because the analog
 * output and the raw angle both wrap, which defeats unwrapping.
 */
MotorCalibration::WindowSpans MotorCalibration::window_spans(const WindowStats & window) const
{
    WindowSpans spans = {};
    for (size_t i = 0; i < 4; ++i)
    {
        const uint16_t analog_span =
          window.analog_max[i] > window.analog_min[i]
            ? static_cast<uint16_t>(window.analog_max[i] - window.analog_min[i])
            : 0;
        const uint16_t i2c_span = window.i2c_max[i] > window.i2c_min[i]
                                    ? static_cast<uint16_t>(window.i2c_max[i] - window.i2c_min[i])
                                    : 0;
        spans.analog[i] = analog_span;
        spans.i2c[i] = i2c_span;
    }
    return spans;
}

/**
 * @brief Samples every input until the window elapses.
 *
 * Publishes the live per-input spreads (raw minimum-to-maximum movement, not
 * baseline-corrected) while a status is supplied so the dashboard can watch
 * the current spin. One tick (10 ms) is the shortest delay available at
 * CONFIG_FREERTOS_HZ=100.
 *
 * @return false when the routine must abort.
 */
bool MotorCalibration::run_window(WindowStats * window,
                                  uint32_t duration_ms,
                                  EncoderStatus * live_status)
{
    const int64_t start_us = esp_timer_get_time();
    const int64_t duration_us = static_cast<int64_t>(duration_ms) * 1000;

    while (esp_timer_get_time() - start_us < duration_us)
    {
        if (should_stop())
            return false;

        sample_once(window, live_status != nullptr ? &live_status->magnet_warning : nullptr);

        if (live_status != nullptr)
        {
            const WindowSpans spans = window_spans(*window);
            for (size_t i = 0; i < 4; ++i)
            {
                live_status->analog_level[i] =
                  static_cast<int16_t>(std::min<int>(spans.analog[i], INT16_MAX));
                live_status->i2c_level[i] =
                  static_cast<int16_t>(std::min<int>(spans.i2c[i], INT16_MAX));
            }
            publish_encoder_status(*live_status);
        }

        vTaskDelay(1);
    }
    return true;
}

/**
 * @brief Waits for the stopped wheel to settle, checking for abort.
 *
 * @return false when the routine must abort.
 */
bool MotorCalibration::run_settle(uint32_t duration_ms)
{
    const int64_t start_us = esp_timer_get_time();
    const int64_t duration_us = static_cast<int64_t>(duration_ms) * 1000;

    while (esp_timer_get_time() - start_us < duration_us)
    {
        if (should_stop())
            return false;
        vTaskDelay(1);
    }
    return true;
}

/**
 * @brief Picks the only input that moved clearly more than its baseline.
 *
 * An input counts as moving when its baseline-corrected score reaches
 * max(MOVEMENT_MIN_SPAN, 8 * baseline + 100). A second input above that
 * threshold with more than a third of the winner's score makes the result
 * ambiguous.
 *
 * @param spans Baseline-corrected movement score per input.
 * @param baseline Baseline-window spread per input, used for the thresholds.
 * @param ambiguous_error EncoderError reported when two inputs moved.
 * @param best_input Output for the winning input index.
 * @return EncoderError::NONE when exactly one input moved.
 */
MotorCalibration::EncoderError MotorCalibration::pick_moving_input(
  const std::array<uint16_t, 4> & spans,
  const std::array<uint16_t, 4> & baseline,
  EncoderError ambiguous_error,
  uint8_t * best_input) const
{
    uint8_t best = 0;
    for (uint8_t i = 1; i < 4; ++i)
    {
        if (spans[i] > spans[best])
            best = i;
    }

    const uint32_t best_threshold =
      std::max<uint32_t>(MOVEMENT_MIN_SPAN, 8u * baseline[best] + 100u);
    if (spans[best] < best_threshold)
        return EncoderError::NO_MOVEMENT;

    for (uint8_t i = 0; i < 4; ++i)
    {
        if (i == best)
            continue;
        const uint32_t threshold = std::max<uint32_t>(MOVEMENT_MIN_SPAN, 8u * baseline[i] + 100u);
        if (spans[i] >= threshold && spans[i] * 3u > spans[best])
            return ambiguous_error;
    }

    *best_input = best;
    return EncoderError::NONE;
}

/**
 * @brief Checks that a wheel -> input map visits every input exactly once.
 */
bool MotorCalibration::is_permutation(const std::array<uint8_t, 4> & map) const
{
    uint8_t seen = 0;
    for (const uint8_t value : map)
    {
        if (value >= 4)
            return false;
        const uint8_t bit = static_cast<uint8_t>(1U << value);
        if ((seen & bit) != 0U)
            return false;
        seen |= bit;
    }
    return seen == 0x0FU;
}

bool MotorCalibration::should_stop() const
{
    if (abort_requested())
        return true;
    return esp_timer_get_time() - m_encoder_start_us > ENCODER_RUN_BUDGET_US;
}

/**
 * @brief Ordinary least-squares fit adc = slope*i2c + intercept.
 *
 * @param i2c I2C raw angle samples (0..4095).
 * @param adc ADC raw samples of the same wheel.
 * @param count Number of samples.
 * @param slope Output slope.
 * @param intercept Output intercept.
 * @param rms Output residual RMS in counts.
 * @return false when the paired data has no usable spread.
 */
bool MotorCalibration::fit_line(const std::array<uint16_t, ALIGN_MAX_CYCLES> & i2c,
                                const std::array<uint16_t, ALIGN_MAX_CYCLES> & adc,
                                size_t count,
                                double * slope,
                                double * intercept,
                                double * rms)
{
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_xx = 0.0;
    double sum_xy = 0.0;
    for (size_t i = 0; i < count; ++i)
    {
        const double x = static_cast<double>(i2c[i]);
        const double y = static_cast<double>(adc[i]);
        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
    }

    const double n = static_cast<double>(count);
    const double denominator = n * sum_xx - sum_x * sum_x;
    if (std::fabs(denominator) < 1.0)
        return false;

    const double fitted_slope = (n * sum_xy - sum_x * sum_y) / denominator;
    const double fitted_intercept = (sum_y - fitted_slope * sum_x) / n;
    double squared_error = 0.0;
    for (size_t i = 0; i < count; ++i)
    {
        const double residual =
          static_cast<double>(adc[i]) - (fitted_slope * i2c[i] + fitted_intercept);
        squared_error += residual * residual;
    }

    *slope = fitted_slope;
    *intercept = fitted_intercept;
    *rms = std::sqrt(squared_error / n);
    return true;
}

/**
 * @brief Robust least-squares fit adc = slope*i2c + intercept.
 *
 * A first pass fits the retained standstill pairs; pairs whose residual exceeds
 * ALIGN_OUTLIER_COUNTS are dropped and the line is refitted when at least
 * ALIGN_MIN_FIT_SAMPLES remain. The final fit is rejected when the sample
 * count, slope, residual RMS or anchored endpoints are outside the accepted
 * band.
 *
 * @param i2c Retained I2C raw angle samples (0..4095, wrap guard already applied).
 * @param adc ADC raw samples of the same wheel.
 * @param count Number of retained samples.
 * @return Fit result; valid is false when the data does not qualify.
 */
MotorCalibration::LinearFit MotorCalibration::fit_adc_vs_i2c(
  const std::array<uint16_t, ALIGN_MAX_CYCLES> & i2c,
  const std::array<uint16_t, ALIGN_MAX_CYCLES> & adc,
  size_t count)
{
    LinearFit fit;
    if (count < ALIGN_MIN_FIT_SAMPLES || count > ALIGN_MAX_CYCLES)
        return fit;

    double slope = 0.0;
    double intercept = 0.0;
    double rms = 0.0;
    if (!fit_line(i2c, adc, count, &slope, &intercept, &rms))
        return fit;

    // Robust refit: drop the pairs whose residual exceeds the outlier band.
    std::array<uint16_t, ALIGN_MAX_CYCLES> kept_i2c = {};
    std::array<uint16_t, ALIGN_MAX_CYCLES> kept_adc = {};
    size_t kept = 0;
    for (size_t i = 0; i < count; ++i)
    {
        const double residual = static_cast<double>(adc[i]) - (slope * i2c[i] + intercept);
        if (std::fabs(residual) <= static_cast<double>(ALIGN_OUTLIER_COUNTS))
        {
            kept_i2c[kept] = i2c[i];
            kept_adc[kept] = adc[i];
            ++kept;
        }
    }
    fit.outliers_dropped = count - kept;

    if (kept < ALIGN_MIN_FIT_SAMPLES)
        return fit;
    if (kept < count)
    {
        if (!fit_line(kept_i2c, kept_adc, kept, &slope, &intercept, &rms))
            return fit;
        count = kept;
    }

    fit.slope = static_cast<float>(slope);
    fit.intercept = static_cast<float>(intercept);
    fit.residual_rms = static_cast<float>(rms);
    fit.samples_used = count;

    const double max_adc = slope * 4095.0 + intercept;
    if (slope < ALIGN_MIN_SLOPE || slope > ALIGN_MAX_SLOPE)
        return fit;
    if (rms > ALIGN_MAX_RESIDUAL_RMS)
        return fit;
    if (intercept < 0.0 || max_adc > 4095.0)
        return fit;

    fit.valid = true;
    return fit;
}

/**
 * @brief Waits until one wheel is actually at rest after an alignment pulse.
 *
 * The fixed settle window (ALIGN_SETTLE_MS) is the floor; a wheel that is still
 * coasting after it would make the ADC and I2C samples describe different
 * angles and inflate the fit residual. The smoothed wheel velocity is polled
 * every ALIGN_REST_POLL_MS until it drops below ALIGN_REST_RAD_S, with
 * ALIGN_REST_TIMEOUT_MS as the cap.
 *
 * @param wheel Wheel index (0..3).
 * @return false only when the routine must abort (user abort or budget).
 */
bool MotorCalibration::wait_for_rest(uint8_t wheel)
{
    if (!run_settle(ALIGN_SETTLE_MS))
        return false;

    WheelStateEstimator & estimator = WheelStateEstimator::get_instance();
    const int64_t timeout_us = static_cast<int64_t>(ALIGN_REST_TIMEOUT_MS) * 1000;
    const int64_t start_us = esp_timer_get_time();
    while (esp_timer_get_time() - start_us < timeout_us)
    {
        if (should_stop())
            return false;
        const std::array<float, NUM_ENC_CHANNELS> velocities =
          estimator.get_smoothed_velocity_rad_s();
        if (std::fabs(velocities[wheel]) < ALIGN_REST_RAD_S)
            return true;
        vTaskDelay(pdMS_TO_TICKS(ALIGN_REST_POLL_MS));
    }

    ESP_LOGW(TAG,
             "Wheel %s alignment: still moving after %lu ms; sampling anyway.",
             WHEEL_NAMES[wheel],
             static_cast<unsigned long>(ALIGN_REST_TIMEOUT_MS));
    return true;
}

bool MotorCalibration::pulse_alignment_wheel(uint8_t wheel, uint32_t duration_ms)
{
    auto & motors = BL48250::get_instance();
    std::array<uint32_t, 4> duties = {};
    duties[wheel] = ALIGN_DUTY;
    motors.set_duties(duties, CLOCKWISE_DIRECTIONS);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    motors.set_duties(STOP_DUTIES, CLOCKWISE_DIRECTIONS);
    return wait_for_rest(wheel);
}

MotorCalibration::EncoderError MotorCalibration::collect_alignment_samples(
  uint8_t wheel, uint8_t analog_slot, AlignmentSamples & samples)
{
    auto & estimator = WheelStateEstimator::get_instance();
    int32_t previous_angle = 0;
    int32_t minimum = INT32_MAX;
    int32_t maximum = INT32_MIN;
    int32_t fit_minimum = INT32_MAX;
    int32_t fit_maximum = INT32_MIN;
    bool kick_done = false;

    for (size_t cycle = 0; cycle < ALIGN_MAX_CYCLES; ++cycle)
    {
        if (should_stop())
            return EncoderError::ABORTED;

        // Vary the pulse duration so the samples cover different shaft positions.
        const uint32_t pulse_ms =
          ALIGN_PULSE_MIN_MS + (static_cast<uint32_t>(cycle) * ALIGN_PULSE_STEP_MS) %
                                 (ALIGN_PULSE_MAX_MS - ALIGN_PULSE_MIN_MS + 1U);
        if (!pulse_alignment_wheel(wheel, pulse_ms))
            return EncoderError::ABORTED;

        uint16_t i2c_raw = 0;
        if (estimator.read_wheel_i2c_raw_angle(wheel, &i2c_raw) != ESP_OK)
        {
            ESP_LOGE(TAG, "Wheel %s alignment: I2C read failed.", WHEEL_NAMES[wheel]);
            return EncoderError::I2C;
        }
        const auto analog_values = estimator.get_raw_values();

        // Unwrap for coverage. The fit uses raw counts on both paths.
        int32_t angle = i2c_raw;
        if (samples.count > 0)
        {
            while (angle - previous_angle < -2048) angle += 4096;
            while (angle - previous_angle > 2048) angle -= 4096;
        }
        previous_angle = angle;
        minimum = std::min(minimum, angle);
        maximum = std::max(maximum, angle);
        samples.spread = maximum - minimum;
        ++samples.count;

        // The analog output is nonlinear near the angle wrap.
        if (i2c_raw >= ALIGN_WRAP_GUARD && i2c_raw <= 4095 - ALIGN_WRAP_GUARD)
        {
            samples.i2c[samples.fit_count] = i2c_raw;
            samples.adc[samples.fit_count] = analog_values[analog_slot];
            ++samples.fit_count;
            fit_minimum = std::min(fit_minimum, angle);
            fit_maximum = std::max(fit_maximum, angle);
            samples.fit_spread = fit_maximum - fit_minimum;
        }

        if (!kick_done && samples.count >= ALIGN_KICK_AFTER_CYCLES &&
            samples.spread < ALIGN_KICK_SPREAD)
        {
            kick_done = true;
            ESP_LOGW(TAG,
                     "Wheel %s alignment: only %d I2C counts after %u cycles; driving a %lu ms "
                     "kick.",
                     WHEEL_NAMES[wheel],
                     static_cast<int>(samples.spread),
                     static_cast<unsigned>(samples.count),
                     static_cast<unsigned long>(ALIGN_KICK_MS));
            if (!pulse_alignment_wheel(wheel, ALIGN_KICK_MS))
                return EncoderError::ABORTED;
        }

        if (samples.count >= ALIGN_MIN_CYCLES && samples.spread >= ALIGN_MIN_I2C_SPREAD &&
            samples.fit_count >= ACQUIRE_MIN_MIDRANGE_SAMPLES &&
            samples.fit_spread >= ALIGN_MIN_FIT_SPREAD)
            break;
    }
    return EncoderError::NONE;
}

bool MotorCalibration::save_alignment(uint8_t wheel, const AlignmentSamples & samples)
{
    if (samples.spread < ALIGN_MIN_I2C_SPREAD)
    {
        ESP_LOGE(TAG,
                 "Wheel %s alignment: I2C spread %d < %d counts after %u cycles; keeping the "
                 "stored calibration.",
                 WHEEL_NAMES[wheel],
                 static_cast<int>(samples.spread),
                 static_cast<int>(ALIGN_MIN_I2C_SPREAD),
                 static_cast<unsigned>(samples.count));
        return false;
    }

    const size_t wrap_excluded = samples.count - samples.fit_count;
    if (wrap_excluded > 0)
        ESP_LOGI(TAG,
                 "Wheel %s alignment: excluded %u wrap-zone samples (guard %d counts).",
                 WHEEL_NAMES[wheel],
                 static_cast<unsigned>(wrap_excluded),
                 static_cast<int>(ALIGN_WRAP_GUARD));

    if (samples.fit_count < ACQUIRE_MIN_MIDRANGE_SAMPLES ||
        samples.fit_spread < ALIGN_MIN_FIT_SPREAD)
    {
        ESP_LOGE(TAG,
                 "Wheel %s alignment: only %u mid-range samples covering %d counts after %u "
                 "cycles (need >= %u and >= %d); keeping the stored calibration.",
                 WHEEL_NAMES[wheel],
                 static_cast<unsigned>(samples.fit_count),
                 static_cast<int>(samples.fit_spread),
                 static_cast<unsigned>(samples.count),
                 static_cast<unsigned>(ACQUIRE_MIN_MIDRANGE_SAMPLES),
                 static_cast<int>(ALIGN_MIN_FIT_SPREAD));
        return false;
    }

    const LinearFit fit = fit_adc_vs_i2c(samples.i2c, samples.adc, samples.fit_count);
    if (!fit.valid)
    {
        ESP_LOGE(TAG,
                 "Wheel %s alignment rejected (n=%u used=%u dropped=%u spread=%d slope=%.4f "
                 "rms=%.2f); keeping the stored calibration.",
                 WHEEL_NAMES[wheel],
                 static_cast<unsigned>(samples.count),
                 static_cast<unsigned>(fit.samples_used),
                 static_cast<unsigned>(fit.outliers_dropped),
                 static_cast<int>(samples.spread),
                 static_cast<double>(fit.slope),
                 static_cast<double>(fit.residual_rms));
        return false;
    }

    const long min_raw = std::lround(fit.intercept);
    const long max_raw = std::lround(fit.slope * 4095.0f + fit.intercept);
    ESP_LOGI(TAG,
             "Wheel %s ADC-I2C alignment: n=%u used=%u dropped=%u spread=%d slope=%.4f "
             "rms=%.2f endpoints [%ld..%ld].",
             WHEEL_NAMES[wheel],
             static_cast<unsigned>(samples.count),
             static_cast<unsigned>(fit.samples_used),
             static_cast<unsigned>(fit.outliers_dropped),
             static_cast<int>(samples.spread),
             static_cast<double>(fit.slope),
             static_cast<double>(fit.residual_rms),
             min_raw,
             max_raw);
    if (WheelStateEstimator::get_instance().set_wheel_raw_range(
          wheel, static_cast<uint16_t>(min_raw), static_cast<uint16_t>(max_raw)) != ESP_OK)
    {
        ESP_LOGE(TAG,
                 "Wheel %s alignment: anchored endpoints rejected or not persisted; keeping "
                 "the stored calibration.",
                 WHEEL_NAMES[wheel]);
        return false;
    }
    return true;
}

/**
 * @brief Fits and stores each wheel's raw range using stationary angle pairs.
 * @return false when the routine was aborted or exhausted its time budget.
 */
bool MotorCalibration::align_adc_to_i2c(const std::array<uint8_t, 4> & analog_map,
                                        uint8_t & anchored_mask)
{
    anchored_mask = 0;
    for (uint8_t wheel = 0; wheel < NUM_ENC_CHANNELS; ++wheel)
    {
        if (should_stop())
            return false;

        AlignmentSamples samples;
        const EncoderError capture = collect_alignment_samples(wheel, analog_map[wheel], samples);
        if (capture == EncoderError::ABORTED)
            return false;
        if (capture == EncoderError::NONE && save_alignment(wheel, samples))
            anchored_mask |= static_cast<uint8_t>(1U << wheel);
    }

    if (anchored_mask == 0)
        ESP_LOGW(TAG, "ADC-I2C alignment produced no usable wheel; the stored ranges were kept.");
    return true;
}

void MotorCalibration::log_spans(uint8_t wheel, const char * label, const WindowSpans & spans)
{
    ESP_LOGI(TAG,
             "Wheel %s %s: ENC1=%u ENC2=%u ENC3=%u ENC4=%u | mux1=%u mux2=%u mux3=%u mux4=%u",
             WHEEL_NAMES[wheel],
             label,
             static_cast<unsigned>(spans.analog[0]),
             static_cast<unsigned>(spans.analog[1]),
             static_cast<unsigned>(spans.analog[2]),
             static_cast<unsigned>(spans.analog[3]),
             static_cast<unsigned>(spans.i2c[0]),
             static_cast<unsigned>(spans.i2c[1]),
             static_cast<unsigned>(spans.i2c[2]),
             static_cast<unsigned>(spans.i2c[3]));
}

bool MotorCalibration::measure_wheel_motion(EncoderStatus & status,
                                            WindowSpans & baseline,
                                            WindowSpans & spin)
{
    const uint8_t wheel = status.wheel;
    status.progress = static_cast<uint8_t>(wheel * 20);
    status.analog_level = {};
    status.i2c_level = {};
    publish_encoder_status(status);
    ESP_LOGI(TAG, "Wheel %s: sampling the resting inputs.", WHEEL_NAMES[wheel]);
    if (should_stop())
        return false;

    WindowStats window;
    reset_window(&window);
    if (!run_window(&window, BASELINE_MS, nullptr))
        return false;
    baseline = window_spans(window);
    log_spans(wheel, "baseline spans", baseline);

    auto & motors = BL48250::get_instance();
    std::array<uint32_t, 4> duties = {};
    duties[wheel] = CAL_DUTY;
    motors.set_duties(duties, CLOCKWISE_DIRECTIONS);
    ESP_LOGI(TAG,
             "Wheel %s: spinning at duty %lu.",
             WHEEL_NAMES[wheel],
             static_cast<unsigned long>(CAL_DUTY));

    reset_window(&window);
    const bool spin_complete = run_window(&window, SPIN_MS, &status);
    motors.set_duties(STOP_DUTIES, CLOCKWISE_DIRECTIONS);
    if (!spin_complete || !run_settle(SETTLE_MS))
        return false;

    spin = window_spans(window);
    log_spans(wheel, "spin spans", spin);
    return true;
}

MotorCalibration::EncoderError MotorCalibration::match_wheel_inputs(EncoderStatus & status,
                                                                    const WindowSpans & baseline,
                                                                    const WindowSpans & spin)
{
    const uint8_t wheel = status.wheel;
    const WindowSpans scores = {movement_scores(spin.analog, baseline.analog),
                                movement_scores(spin.i2c, baseline.i2c)};
    log_spans(wheel, "movement scores", scores);

    // Report communication failures before classifying missing movement.
    if (m_i2c_attempts > 0 && m_i2c_failures * 2 > m_i2c_attempts)
    {
        ESP_LOGE(TAG,
                 "Wheel %s: %lu of %lu I2C encoder reads failed.",
                 WHEEL_NAMES[wheel],
                 static_cast<unsigned long>(m_i2c_failures),
                 static_cast<unsigned long>(m_i2c_attempts));
        status.wheel_error[wheel] = EncoderError::I2C;
        publish_encoder_status(status);
        return EncoderError::I2C;
    }

    uint8_t best_analog = 0;
    uint8_t best_mux = 0;
    const EncoderError analog_error = pick_moving_input(
      scores.analog, baseline.analog, EncoderError::AMBIGUOUS_ANALOG, &best_analog);
    const EncoderError mux_error =
      pick_moving_input(scores.i2c, baseline.i2c, EncoderError::AMBIGUOUS_I2C, &best_mux);
    const uint8_t bit = static_cast<uint8_t>(1U << wheel);
    status.progress = static_cast<uint8_t>((wheel + 1) * 20);
    if (analog_error == EncoderError::NONE)
        status.analog_ready |= bit;
    if (mux_error == EncoderError::NONE)
    {
        status.mux_map[wheel] = best_mux;
        status.i2c_ready |= bit;
    }

    if (analog_error != EncoderError::NONE || mux_error != EncoderError::NONE)
    {
        // The analog path feeds the controller and takes precedence.
        status.wheel_error[wheel] = analog_error != EncoderError::NONE ? analog_error : mux_error;
        publish_encoder_status(status);
        ESP_LOGE(TAG,
                 "Wheel %s: detection failed (analog %s, mux %s); continuing with the next wheel.",
                 WHEEL_NAMES[wheel],
                 error_name(analog_error),
                 error_name(mux_error));
        return EncoderError::NONE;
    }

    status.analog_map[wheel] = best_analog;
    status.result_mask |= bit;
    publish_encoder_status(status);
    ESP_LOGI(TAG,
             "Wheel %s matched: ENC%u (span %u) and mux%u (span %u).",
             WHEEL_NAMES[wheel],
             static_cast<unsigned>(best_analog + 1),
             static_cast<unsigned>(spin.analog[best_analog]),
             static_cast<unsigned>(best_mux + 1),
             static_cast<unsigned>(spin.i2c[best_mux]));
    return EncoderError::NONE;
}

MotorCalibration::EncoderError MotorCalibration::detect_wiring(EncoderStatus & status)
{
    for (uint8_t wheel = 0; wheel < NUM_ENC_CHANNELS; ++wheel)
    {
        status.wheel = wheel;
        WindowSpans baseline;
        WindowSpans spin;
        if (!measure_wheel_motion(status, baseline, spin))
            return EncoderError::ABORTED;
        const EncoderError error = match_wheel_inputs(status, baseline, spin);
        if (error != EncoderError::NONE)
            return error;
    }

    if (status.result_mask == ALL_WHEELS_MASK &&
        (!is_permutation(status.analog_map) || !is_permutation(status.mux_map)))
    {
        ESP_LOGE(TAG, "Detected wiring map is not a permutation of the four encoder inputs.");
        return EncoderError::BAD_MAPPING;
    }
    return EncoderError::NONE;
}

void MotorCalibration::restore_unaligned_ranges(uint8_t anchored_mask,
                                                const RawRanges & previous_ranges)
{
    auto & estimator = WheelStateEstimator::get_instance();
    for (uint8_t wheel = 0; wheel < NUM_ENC_CHANNELS; ++wheel)
    {
        const uint16_t min_raw = previous_ranges.minimum[wheel];
        const uint16_t max_raw = previous_ranges.maximum[wheel];
        const int span = static_cast<int>(max_raw) - static_cast<int>(min_raw);
        if ((anchored_mask & (1U << wheel)) != 0 ||
            span < static_cast<int>(AS5600::MIN_VALID_RAW_RANGE) ||
            span > static_cast<int>(AS5600::MAX_VALID_RAW_RANGE))
            continue;

        const esp_err_t restored = estimator.set_wheel_raw_range(wheel, min_raw, max_raw);
        if (restored == ESP_OK)
            ESP_LOGW(TAG,
                     "Wheel %s alignment failed; restored previous anchored range [%u..%u].",
                     WHEEL_NAMES[wheel],
                     static_cast<unsigned>(min_raw),
                     static_cast<unsigned>(max_raw));
        else
            ESP_LOGE(TAG,
                     "Wheel %s alignment failed; previous anchored range [%u..%u] could "
                     "not be restored (%s); keeping the observed range.",
                     WHEEL_NAMES[wheel],
                     static_cast<unsigned>(min_raw),
                     static_cast<unsigned>(max_raw),
                     esp_err_to_name(restored));
    }

    const auto applied_min = estimator.get_range_raw_min();
    const auto applied_max = estimator.get_range_raw_max();
    for (size_t wheel = 0; wheel < applied_min.size(); ++wheel)
        ESP_LOGI(TAG,
                 "Wheel M%u applied raw range: [%u..%u].",
                 static_cast<unsigned>(wheel + 1),
                 static_cast<unsigned>(applied_min[wheel]),
                 static_cast<unsigned>(applied_max[wheel]));
}

MotorCalibration::EncoderError MotorCalibration::apply_calibration(
  EncoderStatus & status, const RawRanges & previous_ranges)
{
    auto & estimator = WheelStateEstimator::get_instance();
    const uint8_t range_slots = estimator.finish_range_tracking();
    status.range_mask = 0;
    for (uint8_t wheel = 0; wheel < NUM_ENC_CHANNELS; ++wheel)
    {
        if ((range_slots & (1U << status.analog_map[wheel])) != 0)
            status.range_mask |= static_cast<uint8_t>(1U << wheel);
    }
    ESP_LOGI(TAG,
             "Encoder ranges: valid slots 0x%02X, wheel mask 0x%02X.",
             static_cast<unsigned>(range_slots),
             static_cast<unsigned>(status.range_mask));
    if (status.range_mask != ALL_WHEELS_MASK)
        ESP_LOGW(TAG, "Some encoder ranges are invalid; closed-loop control may need a rerun.");

    const auto raw_min = estimator.get_range_raw_min();
    const auto raw_max = estimator.get_range_raw_max();
    for (size_t wheel = 0; wheel < raw_min.size(); ++wheel)
    {
        ESP_LOGI(TAG,
                 "Wheel M%u encoder raw range: [%u..%u] (%u counts).",
                 static_cast<unsigned>(wheel + 1),
                 static_cast<unsigned>(raw_min[wheel]),
                 static_cast<unsigned>(raw_max[wheel]),
                 static_cast<unsigned>(raw_max[wheel] - raw_min[wheel]));
    }

    const auto & analog_map = status.analog_map;
    const auto & mux_map = status.mux_map;
    const esp_err_t applied = estimator.apply_mapping(analog_map, mux_map);
    if (applied != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to apply the wiring map: %s.", esp_err_to_name(applied));
        return EncoderError::BAD_MAPPING;
    }

    // NOLINTNEXTLINE(bugprone-branch-clone)
    ESP_LOGI(TAG,
             "Wiring map applied and persisted: M1 ENC%u/mux%u | M2 ENC%u/mux%u | "
             "M3 ENC%u/mux%u | M4 ENC%u/mux%u",
             static_cast<unsigned>(analog_map[0] + 1),
             static_cast<unsigned>(mux_map[0] + 1),
             static_cast<unsigned>(analog_map[1] + 1),
             static_cast<unsigned>(mux_map[1] + 1),
             static_cast<unsigned>(analog_map[2] + 1),
             static_cast<unsigned>(mux_map[2] + 1),
             static_cast<unsigned>(analog_map[3] + 1),
             static_cast<unsigned>(mux_map[3] + 1));

    status.progress = 90;
    status.wheel = NUM_ENC_CHANNELS;
    publish_encoder_status(status);
    uint8_t anchored_mask = 0;
    const bool alignment_complete = align_adc_to_i2c(analog_map, anchored_mask);
    const bool aborted = !alignment_complete && abort_requested();
    if (!alignment_complete && !aborted)
        ESP_LOGW(
          TAG,
          "ADC-I2C alignment stopped by the runtime budget; keeping the ranges stored so far.");

    // Restore valid prior anchors for wheels whose new fit was rejected.
    restore_unaligned_ranges(anchored_mask, previous_ranges);
    return aborted ? EncoderError::ABORTED : EncoderError::NONE;
}

void MotorCalibration::publish_encoder_completion(EncoderStatus & status, EncoderError error)
{
    status.analog_level = {};
    status.i2c_level = {};
    status.error = error;
    if (error == EncoderError::NONE)
    {
        status.state = status.result_mask == ALL_WHEELS_MASK ? State::DONE : State::PARTIAL;
        status.progress = 100;
        status.wheel = NUM_ENC_CHANNELS;
        if (status.state == State::DONE)
            ESP_LOGI(TAG, "Calibration done.");
    }
    else
    {
        if (error == EncoderError::ABORTED || error == EncoderError::BAD_MAPPING)
            status.wheel = NUM_ENC_CHANNELS;
        status.state = State::FAILED;
        ESP_LOGE(TAG, "Calibration failed: %s.", error_name(error));
    }
    publish_encoder_status(status);
}

void MotorCalibration::run_encoder_alignment()
{
    m_encoder_start_us = esp_timer_get_time();
    m_i2c_attempts = 0;
    m_i2c_failures = 0;

    auto & wheel_controller = WheelController::get_instance();
    auto & estimator = WheelStateEstimator::get_instance();
    auto & motors = BL48250::get_instance();
    wheel_controller.set_open_loop(true);
    motors.set_duties(STOP_DUTIES, CLOCKWISE_DIRECTIONS);
    ESP_LOGI(TAG, "Calibration started: open loop, one wheel at a time; the wheels spin.");

    if (estimator.configure_encoder_output_stage(AS5600::OutputStage::ANALOG_REDUCED) == ESP_OK)
        ESP_LOGI(TAG, "Encoder output stage: all wheels REDUCED.");
    else
        ESP_LOGE(TAG,
                 "Encoder output stage: not all wheels are REDUCED; ranges may be unreliable.");

    const RawRanges previous_ranges = {estimator.get_range_raw_min(),
                                       estimator.get_range_raw_max()};
    estimator.start_range_tracking();

    EncoderStatus status = get_encoder_status();
    EncoderError error = detect_wiring(status);
    if (error == EncoderError::NONE && status.result_mask == ALL_WHEELS_MASK)
        error = apply_calibration(status, previous_ranges);
    else
    {
        estimator.stop_range_tracking();
        if (error == EncoderError::NONE)
            ESP_LOGE(TAG, "Calibration incomplete; the stored wiring map was kept.");
    }

    motors.set_duties(STOP_DUTIES, CLOCKWISE_DIRECTIONS);
    wheel_controller.set_open_loop(false);
    publish_encoder_completion(status, error);

    ESP_LOGI(TAG, "Encoder alignment task finished.");
    release_routine();
    vTaskDelete(nullptr);
}
