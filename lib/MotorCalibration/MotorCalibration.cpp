#include "MotorCalibration.h"

#include <esp_log.h>
#include <esp_timer.h>

#include <algorithm>

#include "BL48250.h"
#include "WheelController.h"
#include "wheel_state_estimator.h"

static const char * TAG = "MotorCalibration";

static const char * WHEEL_NAMES[4] = {"M1", "M2", "M3", "M4"};

/**
 * @brief Human-readable calibration failure reason.
 */
static const char * error_name(MotorCalibration::Error error)
{
    switch (error)
    {
        case MotorCalibration::Error::NONE:
            return "none";
        case MotorCalibration::Error::NO_MOVEMENT:
            return "no movement";
        case MotorCalibration::Error::AMBIGUOUS_ANALOG:
            return "ambiguous analog input";
        case MotorCalibration::Error::AMBIGUOUS_I2C:
            return "ambiguous mux channel";
        case MotorCalibration::Error::BAD_MAPPING:
            return "invalid wiring map";
        case MotorCalibration::Error::ABORTED:
            return "aborted";
        case MotorCalibration::Error::I2C:
            return "I2C read failures";
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

MotorCalibration::MotorCalibration()
: m_status_mutex(xSemaphoreCreateMutex()),
  m_status(),
  m_running(false),
  m_abort(false),
  m_task_handle(nullptr),
  m_run_start_us(0),
  m_i2c_attempts(0),
  m_i2c_failures(0)
{
}

/**
 * @brief Starts a calibration run in a background task.
 *
 * Rejects a second run while one is active. The status is reset to RUNNING
 * before the task exists, so the dashboard sees the routine start immediately.
 *
 * @return ESP_OK when the task was created, ESP_ERR_INVALID_STATE when a run is
 *         already active, ESP_ERR_NO_MEM when the task could not be created.
 */
esp_err_t MotorCalibration::start()
{
    if (m_running.exchange(true))
        return ESP_ERR_INVALID_STATE;

    m_abort.store(false);

    Status status;
    status.state = State::RUNNING;
    status.wheel = 4;
    status.progress = 0;
    status.result_mask = 0;
    status.i2c_ready = 0;
    status.analog_map = {0, 1, 2, 3};
    status.mux_map = {0, 1, 2, 3};
    status.analog_level = {};
    status.i2c_level = {};
    status.range_mask = 0;
    status.error = Error::NONE;
    publish(status);

    if (xTaskCreate(task_wrapper, "motor_cal", 4096, this, 4, &m_task_handle) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create the calibration task.");
        m_running.store(false);
        m_task_handle = nullptr;
        status.state = State::IDLE;
        publish(status);
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

/**
 * @brief Requests the running routine to stop.
 *
 * Only raises the abort flag: the routine checks it between samples, stops the
 * motors and publishes the final status from its own task. Idempotent.
 */
void MotorCalibration::abort()
{
    m_abort.store(true);
}

bool MotorCalibration::is_running() const
{
    return m_running.load();
}

/**
 * @brief Returns a thread-safe copy of the latest status.
 */
MotorCalibration::Status MotorCalibration::get_status() const
{
    Status status;
    if (m_status_mutex == nullptr)
        return status;

    xSemaphoreTake(m_status_mutex, portMAX_DELAY);
    status = m_status;
    xSemaphoreGive(m_status_mutex);
    return status;
}

/**
 * @brief Replaces the published status snapshot.
 */
void MotorCalibration::publish(const Status & status)
{
    if (m_status_mutex == nullptr)
        return;

    xSemaphoreTake(m_status_mutex, portMAX_DELAY);
    m_status = status;
    xSemaphoreGive(m_status_mutex);
}

void MotorCalibration::task_wrapper(void * arg)
{
    static_cast<MotorCalibration *>(arg)->task_loop();
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
void MotorCalibration::sample_once(WindowStats * window)
{
    WheelStateEstimator & estimator = WheelStateEstimator::get_instance();

    const std::array<uint16_t, 4> analog = estimator.get_raw_values();
    for (size_t i = 0; i < 4; ++i)
    {
        window->analog_min[i] = std::min(window->analog_min[i], analog[i]);
        window->analog_max[i] = std::max(window->analog_max[i], analog[i]);
    }

    std::array<uint16_t, 4> angles = {};
    ++m_i2c_attempts;
    if (estimator.read_i2c_raw_angles(&angles) != ESP_OK)
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
bool MotorCalibration::run_window(WindowStats * window, uint32_t duration_ms, Status * live_status)
{
    const int64_t start_us = esp_timer_get_time();
    const int64_t duration_us = static_cast<int64_t>(duration_ms) * 1000;

    while (esp_timer_get_time() - start_us < duration_us)
    {
        if (should_stop())
            return false;

        sample_once(window);

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
            publish(*live_status);
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
 * @param ambiguous_error Error reported when two inputs moved.
 * @param best_input Output for the winning input index.
 * @return Error::NONE when exactly one input moved.
 */
MotorCalibration::Error MotorCalibration::pick_moving_input(
  const std::array<uint16_t, 4> & spans,
  const std::array<uint16_t, 4> & baseline,
  Error ambiguous_error,
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
        return Error::NO_MOVEMENT;

    for (uint8_t i = 0; i < 4; ++i)
    {
        if (i == best)
            continue;
        const uint32_t threshold = std::max<uint32_t>(MOVEMENT_MIN_SPAN, 8u * baseline[i] + 100u);
        if (spans[i] >= threshold && spans[i] * 3u > spans[best])
            return ambiguous_error;
    }

    *best_input = best;
    return Error::NONE;
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
    if (m_abort.load())
        return true;
    return esp_timer_get_time() - m_run_start_us > RUN_BUDGET_US;
}

/**
 * @brief Runs the complete wiring calibration sequence.
 *
 * Drives one wheel at a time in open loop while every analog input and every
 * mux channel is sampled, derives the wheel -> encoder maps from the input
 * that moved, stores the encoder analog ranges, and applies the maps through
 * WheelStateEstimator. Every exit path stops the motors, leaves open-loop mode
 * and publishes a final status.
 */
void MotorCalibration::task_loop()
{
    m_run_start_us = esp_timer_get_time();
    m_i2c_attempts = 0;
    m_i2c_failures = 0;

    WheelController & wheel_controller = WheelController::get_instance();
    WheelStateEstimator & estimator = WheelStateEstimator::get_instance();
    BL48250 & motors = BL48250::get_instance();

    const std::array<uint32_t, 4> stop_duties = {0, 0, 0, 0};
    const std::array<uint8_t, 4> direction_cw = {config::driver::MOTOR_CW,
                                                 config::driver::MOTOR_CW,
                                                 config::driver::MOTOR_CW,
                                                 config::driver::MOTOR_CW};

    // Open loop: the PID controller must not touch the motor outputs while the
    // routine drives one wheel at a time.
    wheel_controller.set_open_loop(true);
    motors.set_duties(stop_duties, direction_cw);
    ESP_LOGI(TAG, "Calibration started: open loop, one wheel at a time; the wheels spin.");
    estimator.start_range_tracking();

    Status status = get_status();
    std::array<uint8_t, 4> analog_map = {0, 1, 2, 3};
    std::array<uint8_t, 4> mux_map = {0, 1, 2, 3};
    uint8_t result_mask = 0;
    uint8_t i2c_ready = 0;
    Error error = Error::NONE;

    for (uint8_t wheel = 0; wheel < 4 && error == Error::NONE; ++wheel)
    {
        status.wheel = wheel;
        status.progress = static_cast<uint8_t>(wheel * 20);
        status.analog_level = {};
        status.i2c_level = {};
        publish(status);
        ESP_LOGI(TAG, "Wheel %s: sampling the resting inputs.", WHEEL_NAMES[wheel]);

        if (should_stop())
        {
            error = Error::ABORTED;
            break;
        }

        // Baseline: every motor is stopped, so any movement is measurement noise.
        WindowStats baseline = {};
        reset_window(&baseline);
        if (!run_window(&baseline, BASELINE_MS, nullptr))
        {
            error = Error::ABORTED;
            break;
        }
        const WindowSpans baseline_spans = window_spans(baseline);
        ESP_LOGI(TAG,
                 "Wheel %s baseline spans: ENC1=%u ENC2=%u ENC3=%u ENC4=%u | "
                 "mux1=%u mux2=%u mux3=%u mux4=%u",
                 WHEEL_NAMES[wheel],
                 static_cast<unsigned>(baseline_spans.analog[0]),
                 static_cast<unsigned>(baseline_spans.analog[1]),
                 static_cast<unsigned>(baseline_spans.analog[2]),
                 static_cast<unsigned>(baseline_spans.analog[3]),
                 static_cast<unsigned>(baseline_spans.i2c[0]),
                 static_cast<unsigned>(baseline_spans.i2c[1]),
                 static_cast<unsigned>(baseline_spans.i2c[2]),
                 static_cast<unsigned>(baseline_spans.i2c[3]));

        // Spin: only this wheel driven, all of them forward. The duty is high
        // enough that its encoder sweeps past one turn during the window.
        std::array<uint32_t, 4> spin_duties = {0, 0, 0, 0};
        spin_duties[wheel] = CAL_DUTY;
        motors.set_duties(spin_duties, direction_cw);
        ESP_LOGI(TAG,
                 "Wheel %s: spinning at duty %lu.",
                 WHEEL_NAMES[wheel],
                 static_cast<unsigned long>(CAL_DUTY));

        WindowStats spin = {};
        reset_window(&spin);
        const bool spin_complete = run_window(&spin, SPIN_MS, &status);
        motors.set_duties(stop_duties, direction_cw);
        if (!spin_complete || !run_settle(SETTLE_MS))
        {
            error = Error::ABORTED;
            break;
        }

        const WindowSpans spin_spans = window_spans(spin);
        ESP_LOGI(TAG,
                 "Wheel %s spin spans: ENC1=%u ENC2=%u ENC3=%u ENC4=%u | "
                 "mux1=%u mux2=%u mux3=%u mux4=%u",
                 WHEEL_NAMES[wheel],
                 static_cast<unsigned>(spin_spans.analog[0]),
                 static_cast<unsigned>(spin_spans.analog[1]),
                 static_cast<unsigned>(spin_spans.analog[2]),
                 static_cast<unsigned>(spin_spans.analog[3]),
                 static_cast<unsigned>(spin_spans.i2c[0]),
                 static_cast<unsigned>(spin_spans.i2c[1]),
                 static_cast<unsigned>(spin_spans.i2c[2]),
                 static_cast<unsigned>(spin_spans.i2c[3]));

        const std::array<uint16_t, 4> analog_scores =
          movement_scores(spin_spans.analog, baseline_spans.analog);
        const std::array<uint16_t, 4> i2c_scores =
          movement_scores(spin_spans.i2c, baseline_spans.i2c);
        ESP_LOGI(TAG,
                 "Wheel %s movement scores: ENC1=%u ENC2=%u ENC3=%u ENC4=%u | "
                 "mux1=%u mux2=%u mux3=%u mux4=%u",
                 WHEEL_NAMES[wheel],
                 static_cast<unsigned>(analog_scores[0]),
                 static_cast<unsigned>(analog_scores[1]),
                 static_cast<unsigned>(analog_scores[2]),
                 static_cast<unsigned>(analog_scores[3]),
                 static_cast<unsigned>(i2c_scores[0]),
                 static_cast<unsigned>(i2c_scores[1]),
                 static_cast<unsigned>(i2c_scores[2]),
                 static_cast<unsigned>(i2c_scores[3]));

        // A dying I2C path would look like missing movement: report it first.
        if (m_i2c_attempts > 0 && m_i2c_failures * 2 > m_i2c_attempts)
        {
            ESP_LOGE(TAG,
                     "Wheel %s: %lu of %lu I2C encoder reads failed.",
                     WHEEL_NAMES[wheel],
                     static_cast<unsigned long>(m_i2c_failures),
                     static_cast<unsigned long>(m_i2c_attempts));
            error = Error::I2C;
            break;
        }

        uint8_t best_analog = 0;
        uint8_t best_mux = 0;
        error = pick_moving_input(
          analog_scores, baseline_spans.analog, Error::AMBIGUOUS_ANALOG, &best_analog);
        if (error == Error::NONE)
            error =
              pick_moving_input(i2c_scores, baseline_spans.i2c, Error::AMBIGUOUS_I2C, &best_mux);
        if (error != Error::NONE)
        {
            ESP_LOGE(
              TAG, "Wheel %s: detection failed (%s).", WHEEL_NAMES[wheel], error_name(error));
            break;
        }

        analog_map[wheel] = best_analog;
        mux_map[wheel] = best_mux;
        result_mask |= static_cast<uint8_t>(1U << wheel);
        i2c_ready |= static_cast<uint8_t>(1U << wheel);
        status.analog_map = analog_map;
        status.mux_map = mux_map;
        status.result_mask = result_mask;
        status.i2c_ready = i2c_ready;
        status.progress = static_cast<uint8_t>((wheel + 1) * 20);
        publish(status);
        ESP_LOGI(TAG,
                 "Wheel %s matched: ENC%u (span %u) and mux%u (span %u).",
                 WHEEL_NAMES[wheel],
                 static_cast<unsigned>(best_analog + 1),
                 static_cast<unsigned>(spin_spans.analog[best_analog]),
                 static_cast<unsigned>(best_mux + 1),
                 static_cast<unsigned>(spin_spans.i2c[best_mux]));
    }

    // Verify the detected maps cover every input exactly once.
    if (error == Error::NONE && (!is_permutation(analog_map) || !is_permutation(mux_map)))
    {
        ESP_LOGE(TAG, "Detected wiring map is not a permutation of the four encoder inputs.");
        error = Error::BAD_MAPPING;
    }

    if (error == Error::NONE)
    {
        // Every wheel was spun, so the tracked ranges cover every encoder.
        const uint8_t range_slots = estimator.finish_range_tracking();
        uint8_t range_mask = 0;
        for (uint8_t wheel = 0; wheel < 4; ++wheel)
        {
            if ((range_slots & (1U << analog_map[wheel])) != 0)
                range_mask |= static_cast<uint8_t>(1U << wheel);
        }
        status.range_mask = range_mask;
        ESP_LOGI(TAG,
                 "Encoder ranges: valid slots 0x%02X, wheel mask 0x%02X.",
                 static_cast<unsigned>(range_slots),
                 static_cast<unsigned>(range_mask));
        if (range_mask != 0x0F)
            ESP_LOGW(TAG, "Some encoder ranges are invalid; closed-loop control may need a rerun.");

        const esp_err_t applied = estimator.apply_mapping(analog_map, mux_map);
        if (applied == ESP_OK)
        {
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
        }
        else
        {
            ESP_LOGE(TAG, "Failed to apply the wiring map: %s.", esp_err_to_name(applied));
            error = Error::BAD_MAPPING;
        }
    }
    else
    {
        // Never persist ranges tracked by an incomplete run.
        estimator.stop_range_tracking();
    }

    // Always leave the motors stopped and the PID loop in charge again.
    motors.set_duties(stop_duties, direction_cw);
    wheel_controller.set_open_loop(false);

    status.analog_level = {};
    status.i2c_level = {};
    if (error == Error::NONE)
    {
        status.state = State::DONE;
        status.progress = 100;
        status.wheel = 4;
        status.error = Error::NONE;
        ESP_LOGI(TAG, "Calibration done.");
    }
    else
    {
        if (error == Error::ABORTED || error == Error::BAD_MAPPING)
            status.wheel = 4;
        status.state = State::FAILED;
        status.error = error;
        ESP_LOGE(TAG, "Calibration failed: %s.", error_name(error));
    }
    publish(status);

    m_running.store(false);
    m_task_handle = nullptr;
    ESP_LOGI(TAG, "Calibration task finished.");
    vTaskDelete(nullptr);
}
