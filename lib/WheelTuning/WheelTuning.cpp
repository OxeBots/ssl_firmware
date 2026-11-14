/**
 * @file WheelTuning.cpp
 * @brief Per-wheel velocity-control calibration: identification, gain
 *        synthesis, closed-loop validation and NVS persistence.
 */

#include "WheelTuning.h"

#include <esp_log.h>
#include <esp_timer.h>

#include <algorithm>
#include <cmath>

#include "BL48250.h"
#include "MotorCalibration.h"
#include "WheelController.h"
#include "wheel_state_estimator.h"

static const char * TAG = "WheelTuning";

static const char * const WHEEL_NAMES[4] = {"M1", "M2", "M3", "M4"};

// Median of a window of samples: robust to the encoder spikes and the pulse
// velocity quantization at low speeds.
static float median_of(const float * values, size_t count, size_t window)
{
    const size_t n = std::min(window, count);
    std::array<float, 64> buffer = {};
    if (n == 0 || n > buffer.size())
        return 0.0f;
    for (size_t i = 0; i < n; ++i) buffer[i] = values[count - n + i];
    for (size_t i = 1; i < n; ++i)
    {
        const float value = buffer[i];
        size_t j = i;
        while (j > 0 && buffer[j - 1] > value)
        {
            buffer[j] = buffer[j - 1];
            --j;
        }
        buffer[j] = value;
    }
    return buffer[n / 2];
}

// Three-sample median filter over a step response (spike rejection).
static void median_filter_step(const float * input, float * output, size_t count)
{
    for (size_t i = 0; i < count; ++i)
    {
        const size_t first = (i > 0) ? i - 1 : 0;
        const size_t last = std::min(count - 1, i + 1);
        float window[3] = {input[first], input[i], input[last]};
        if (window[0] > window[1])
            std::swap(window[0], window[1]);
        if (window[1] > window[2])
            std::swap(window[1], window[2]);
        if (window[0] > window[1])
            std::swap(window[0], window[1]);
        output[i] = window[1];
    }
}

// Copies the calibration gains into the controller parameter struct.
static WheelController::Parameters to_parameters(const tuning::Gains & gains)
{
    WheelController::Parameters parameters;
    parameters.kp = gains.kp;
    parameters.ki = gains.ki;
    parameters.kd = gains.kd;
    parameters.feedforward = gains.feedforward;
    parameters.velocity_ff = gains.velocity_ff;
    parameters.static_friction = gains.static_friction;
    return parameters;
}

// Per-wheel progress split of the whole routine.
static constexpr uint8_t WHEEL_PROGRESS_SPAN = 100 / 4;
static constexpr uint8_t PHASE_FRICTION_END = 15;
static constexpr uint8_t PHASE_IDENTIFY_END = 45;
static constexpr uint8_t PHASE_VALIDATE_END = 90;

WheelTuning & WheelTuning::get_instance()
{
    static WheelTuning instance;
    return instance;
}

WheelTuning::WheelTuning()
: m_status_mutex(xSemaphoreCreateMutex()), m_running(false), m_abort(false), m_task_handle(nullptr)
{
}

esp_err_t WheelTuning::start()
{
    if (m_running.load())
        return ESP_ERR_INVALID_STATE;
    if (MotorCalibration::get_instance().is_running())
        return ESP_ERR_INVALID_STATE;

    m_abort.store(false);
    m_working = Status();
    m_working.state = State::RUNNING;
    publish();

    m_running.store(true);
    if (xTaskCreatePinnedToCore(task_wrapper, "wheel_tune", 8192, this, 5, &m_task_handle, 0) !=
        pdPASS)
    {
        m_running.store(false);
        m_working.state = State::FAILED;
        m_working.error = Error::ABORTED;
        publish();
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Control calibration started: the wheels spin one at a time.");
    return ESP_OK;
}

void WheelTuning::abort()
{
    m_abort.store(true);
}

bool WheelTuning::is_running() const
{
    return m_running.load();
}

WheelTuning::Status WheelTuning::get_status() const
{
    WheelTuning & self = const_cast<WheelTuning &>(*this);
    Status status;
    if (self.m_status_mutex != nullptr &&
        xSemaphoreTake(self.m_status_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        status = self.m_status;
        xSemaphoreGive(self.m_status_mutex);
    }
    return status;
}

void WheelTuning::publish()
{
    if (m_status_mutex == nullptr)
        return;
    if (xSemaphoreTake(m_status_mutex, pdMS_TO_TICKS(20)) != pdTRUE)
        return;
    m_status = m_working;
    xSemaphoreGive(m_status_mutex);
}

void WheelTuning::set_progress(uint8_t wheel, Phase phase, uint8_t wheel_progress)
{
    m_working.wheel = wheel;
    m_working.phase = phase;
    const uint8_t clamped = std::min<uint8_t>(wheel_progress, 100);
    m_working.progress = static_cast<uint8_t>(
      std::min(100, wheel * WHEEL_PROGRESS_SPAN + clamped * WHEEL_PROGRESS_SPAN / 100));
    publish();
}

void WheelTuning::task_wrapper(void * arg)
{
    static_cast<WheelTuning *>(arg)->task_loop();
}

int WheelTuning::check_stop(uint32_t start_us)
{
    if (m_abort.load())
        return 1;
    const uint32_t elapsed_us = static_cast<uint32_t>(esp_timer_get_time()) - start_us;
    if (elapsed_us > RUN_BUDGET_US)
        return 2;
    return 0;
}

bool WheelTuning::wait_ms(uint32_t duration_ms, uint32_t start_us)
{
    const uint32_t end_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL) + duration_ms;
    while (static_cast<uint32_t>(esp_timer_get_time() / 1000ULL) < end_ms)
    {
        vTaskDelay(1);
        if (check_stop(start_us) != 0)
            return false;
    }
    return true;
}

void WheelTuning::drive_wheel(int wheel, int duty)
{
    std::array<uint32_t, 4> duties = {0, 0, 0, 0};
    std::array<uint8_t, 4> dirs = {config::driver::MOTOR_CW,
                                   config::driver::MOTOR_CW,
                                   config::driver::MOTOR_CW,
                                   config::driver::MOTOR_CW};
    if (wheel >= 0 && wheel < 4)
    {
        const int clamped = std::clamp(duty, -1023, 1023);
        duties[wheel] = static_cast<uint32_t>(std::abs(clamped));
        dirs[wheel] = (clamped >= 0) ? config::driver::MOTOR_CW : config::driver::MOTOR_CCW;
    }
    BL48250::get_instance().set_duties(duties, dirs);
}

bool WheelTuning::sample_velocity(int wheel,
                                  uint32_t duration_ms,
                                  uint32_t start_us,
                                  float * velocity)
{
    auto & estimator = WheelStateEstimator::get_instance();
    const uint32_t end_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL) + duration_ms;
    std::array<float, 64> samples = {};
    size_t count = 0;
    while (static_cast<uint32_t>(esp_timer_get_time() / 1000ULL) < end_ms && count < samples.size())
    {
        vTaskDelay(1);
        if (check_stop(start_us) != 0)
            return false;
        const auto velocities = estimator.get_smoothed_velocity_rad_s();
        // The estimator reports the encoder frame; the motor frame is its negation.
        samples[count++] = -velocities[wheel];
    }
    if (count == 0)
        return false;
    // Median: a single encoder spike must not fake movement.
    *velocity = median_of(samples.data(), count, count);
    return true;
}

size_t WheelTuning::sample_step(int wheel,
                                uint32_t duration_ms,
                                uint32_t start_us,
                                float * times_s,
                                float * velocities,
                                size_t capacity)
{
    auto & estimator = WheelStateEstimator::get_instance();
    const int64_t start = esp_timer_get_time();
    const int64_t end = start + static_cast<int64_t>(duration_ms) * 1000;
    size_t count = 0;
    while (esp_timer_get_time() < end && count < capacity)
    {
        vTaskDelay(1);
        if (check_stop(start_us) != 0)
            break;
        const int64_t now = esp_timer_get_time();
        times_s[count] = static_cast<float>(now - start) / 1e6f;
        const auto measured = estimator.get_smoothed_velocity_rad_s();
        velocities[count] = -measured[wheel];
        count++;
    }
    return count;
}

bool WheelTuning::measure_breakaway(int wheel,
                                    uint32_t start_us,
                                    int * breakaway_duty,
                                    float * angle_sign)
{
    *breakaway_duty = 0;
    *angle_sign = -1.0f;
    auto & estimator = WheelStateEstimator::get_instance();
    constexpr float MOTOR_SHAFT_PERIOD_RAD = 2.0f * static_cast<float>(M_PI);

    for (int duty = BREAKAWAY_START_DUTY; duty <= BREAKAWAY_MAX_DUTY; duty += BREAKAWAY_STEP_DUTY)
    {
        const float raw_before = estimator.get_measured_angle_rad()[wheel];
        drive_wheel(wheel, duty);
        if (!wait_ms(BREAKAWAY_HOLD_MS, start_us))
            return false;
        float velocity = 0.0f;
        if (!sample_velocity(wheel, BREAKAWAY_HOLD_MS / 2, start_us, &velocity))
            return false;
        if (std::fabs(velocity) > MOVEMENT_THRESHOLD_RAD_S)
        {
            *breakaway_duty = duty;
            // Let the wheel spin fast enough that the raw angle clearly moves,
            // then accumulate its deltas: the orientation cannot be read at the
            // breakaway duty, where the wheel barely turns.
            const int sign_duty = std::clamp(duty + 300, 450, 900);
            drive_wheel(wheel, sign_duty);
            float travelled = 0.0f;
            float previous = estimator.get_measured_angle_rad()[wheel];
            const uint32_t sign_end_ms =
              static_cast<uint32_t>(esp_timer_get_time() / 1000ULL) + 700;
            while (static_cast<uint32_t>(esp_timer_get_time() / 1000ULL) < sign_end_ms)
            {
                vTaskDelay(1);
                if (check_stop(start_us) != 0)
                    return false;
                const float current = estimator.get_measured_angle_rad()[wheel];
                travelled += std::remainder(current - previous, MOTOR_SHAFT_PERIOD_RAD);
                previous = current;
            }
            // Positive duty is a positive motor rotation: the raw angle may
            // increase or decrease depending on the magnet orientation.
            if (std::fabs(travelled) > 2.0f)
                *angle_sign = (travelled > 0.0f) ? 1.0f : -1.0f;
            (void)raw_before;
            drive_wheel(wheel, 0);
            if (!wait_ms(IDENTIFY_STOP_MS, start_us))
                return false;
            ESP_LOGI(TAG,
                     "%s breakaway duty: %d raw angle sign: %+.0f",
                     WHEEL_NAMES[wheel],
                     duty,
                     static_cast<double>(*angle_sign));
            return true;
        }
    }
    drive_wheel(wheel, 0);
    return true;
}

bool WheelTuning::identify_plant(int wheel,
                                 int breakaway_duty,
                                 uint32_t start_us,
                                 tuning::PlantModel * plant)
{
    static float times[STEP_SAMPLE_CAPACITY];
    static float velocities[STEP_SAMPLE_CAPACITY];

    // Two duty steps above the breakaway point. The first one is also used for
    // the time constant, so it starts from rest.
    int duty_low = std::clamp(breakaway_duty + 150, 250, 700);
    int duty_high = std::clamp(duty_low + 250, duty_low + 100, 950);

    for (int attempt = 0; attempt < 2; ++attempt)
    {
        drive_wheel(wheel, duty_low);
        const size_t count_low =
          sample_step(wheel, IDENTIFY_HOLD_MS, start_us, times, velocities, STEP_SAMPLE_CAPACITY);
        if (count_low < IDENTIFY_STEADY_SAMPLES + 5)
            return false;
        const float velocity_low = median_of(velocities, count_low, IDENTIFY_STEADY_SAMPLES);

        static float filtered[STEP_SAMPLE_CAPACITY];
        median_filter_step(velocities, filtered, count_low);
        float tau = 0.0f;
        float dead_time = 0.0f;
        tuning::fit_step_dynamics(times, filtered, count_low, &tau, &dead_time);

        // Second step lands on the higher duty and is sampled once settled.
        drive_wheel(wheel, duty_high);
        const size_t count_high =
          sample_step(wheel, IDENTIFY_HOLD_MS, start_us, times, velocities, STEP_SAMPLE_CAPACITY);
        if (count_high < IDENTIFY_STEADY_SAMPLES + 5)
            return false;
        const float velocity_high = median_of(velocities, count_high, IDENTIFY_STEADY_SAMPLES);

        drive_wheel(wheel, 0);
        if (!wait_ms(IDENTIFY_STOP_MS, start_us))
            return false;

        const float duties[2] = {static_cast<float>(duty_low), static_cast<float>(duty_high)};
        const float steady[2] = {velocity_low, velocity_high};
        float gain = 0.0f;
        float fitted_friction = 0.0f;
        const bool fit_ok = tuning::fit_static_curve(duties, steady, 2, &gain, &fitted_friction);
        ESP_LOGI(TAG,
                 "%s identify attempt %d: duty %d/%d -> %.2f/%.2f rad/s, gain=%.5f tau=%.3f "
                 "dead=%.3f",
                 WHEEL_NAMES[wheel],
                 attempt,
                 duty_low,
                 duty_high,
                 static_cast<double>(velocity_low),
                 static_cast<double>(velocity_high),
                 static_cast<double>(gain),
                 static_cast<double>(tau),
                 static_cast<double>(dead_time));

        if (fit_ok && tau > 0.0f && velocity_high > velocity_low + 0.5f)
        {
            plant->gain_rad_s_per_duty = gain;
            // The static-curve intercept is the operating-point feedforward:
            // while the wheel turns, kinetic friction is much lower than the
            // breakaway duty measured from rest.
            plant->friction_duty = fitted_friction;
            plant->breakaway_duty = static_cast<float>(breakaway_duty);
            plant->tau_s = tau;
            plant->dead_time_s = dead_time;
            plant->max_velocity_rad_s = std::max(velocity_low, velocity_high);
            plant->min_velocity_rad_s = std::min(velocity_low, velocity_high);
            plant->valid = true;
            return true;
        }

        // Retry once with wider steps.
        duty_low = std::clamp(breakaway_duty + 250, 350, 850);
        duty_high = std::clamp(duty_low + 300, duty_low + 100, 980);
    }
    return true;
}

bool WheelTuning::validate_step(int wheel,
                                float target_rad_s,
                                uint32_t start_us,
                                tuning::StepMetrics * metrics)
{
    static float times[STEP_SAMPLE_CAPACITY];
    static float velocities[STEP_SAMPLE_CAPACITY];

    WheelController & controller = WheelController::get_instance();
    controller.set_target_velocities({0.0f, 0.0f, 0.0f, 0.0f});
    controller.set_open_loop(false);
    if (!wait_ms(VALIDATE_SETTLE_MS, start_us))
        return false;

    std::array<float, 4> targets = {0.0f, 0.0f, 0.0f, 0.0f};
    targets[wheel] = target_rad_s;
    controller.set_target_velocities(targets);

    const size_t count =
      sample_step(wheel, VALIDATE_HOLD_MS, start_us, times, velocities, STEP_SAMPLE_CAPACITY);
    controller.set_target_velocities({0.0f, 0.0f, 0.0f, 0.0f});

    if (count < VALIDATE_MIN_SAMPLES)
        return false;
    *metrics = tuning::analyze_step(times, velocities, count, target_rad_s);
    ESP_LOGI(TAG,
             "%s validate target=%.2f rad/s: rise=%.0f ms overshoot=%.1f%% ss=%.1f%% valid=%d",
             WHEEL_NAMES[wheel],
             static_cast<double>(target_rad_s),
             static_cast<double>(metrics->rise_ms),
             static_cast<double>(metrics->overshoot_pct),
             static_cast<double>(metrics->ss_error_pct),
             static_cast<int>(metrics->valid));
    return metrics->valid;
}

bool WheelTuning::tune_wheel(int wheel, uint32_t start_us)
{
    WheelController & controller = WheelController::get_instance();
    const WheelController::Parameters original = controller.get_parameters()[wheel];

    set_progress(wheel, Phase::FRICTION, 0);
    controller.set_open_loop(true);
    if (!wait_ms(300, start_us))
        return false;

    int breakaway_duty = 0;
    float angle_sign = -1.0f;
    if (!measure_breakaway(wheel, start_us, &breakaway_duty, &angle_sign))
        return false;
    if (breakaway_duty <= 0)
    {
        ESP_LOGW(TAG, "%s did not move in open loop; wheel failed.", WHEEL_NAMES[wheel]);
        m_working.error = Error::NO_MOVEMENT;
        controller.set_open_loop(false);
        return false;
    }
    // Store the raw-angle orientation used by the position control.
    {
        WheelController::Parameters parameters = controller.get_parameters()[wheel];
        parameters.angle_sign = angle_sign;
        controller.apply_parameters(wheel, parameters);
    }

    set_progress(wheel, Phase::IDENTIFY, 20);
    tuning::PlantModel plant;
    if (!identify_plant(wheel, breakaway_duty, start_us, &plant))
        return false;
    if (!plant.valid)
    {
        ESP_LOGW(TAG, "%s plant identification failed; wheel failed.", WHEEL_NAMES[wheel]);
        m_working.error = Error::UNSTABLE;
        controller.set_open_loop(false);
        return false;
    }

    // Leave open loop: validation runs through the real control loop.
    controller.set_open_loop(false);

    const float target_rad_s = tuning::choose_validation_target(plant);
    tuning::Gains gains = tuning::synthesize(plant, target_rad_s);
    ESP_LOGI(TAG,
             "%s plant: gain=%.5f ff=%.0f vff=%.2f tau=%.3f dead=%.3f target=%.2f rad/s "
             "Kp=%.2f Ki=%.2f",
             WHEEL_NAMES[wheel],
             static_cast<double>(plant.gain_rad_s_per_duty),
             static_cast<double>(gains.feedforward),
             static_cast<double>(gains.velocity_ff),
             static_cast<double>(plant.tau_s),
             static_cast<double>(plant.dead_time_s),
             static_cast<double>(target_rad_s),
             static_cast<double>(gains.kp),
             static_cast<double>(gains.ki));

    tuning::StepMetrics best_metrics;
    tuning::Gains best_gains;
    // Best candidate that satisfies at least the relaxed acceptance criteria.
    // It is tracked apart from the cheapest capture: the cheapest one can be
    // outside the relaxed limits (for example a fast step with a large
    // steady-state error), while a slightly more expensive candidate is fine.
    tuning::StepMetrics fallback_metrics;
    tuning::Gains fallback_gains;
    bool accepted = false;

    for (uint8_t iteration = 0; iteration <= MAX_REFINEMENTS; ++iteration)
    {
        set_progress(wheel,
                     iteration == 0 ? Phase::VALIDATE : Phase::REFINE,
                     static_cast<uint8_t>(50 + iteration * 10));
        controller.apply_parameters(wheel, to_parameters(gains));
        if (check_stop(start_us) != 0)
        {
            controller.apply_parameters(wheel, original);
            return false;
        }

        tuning::StepMetrics metrics;
        bool measured = validate_step(wheel, target_rad_s, start_us, &metrics);
        // A single bad capture (encoder glitch or mechanical jerk) must not
        // discard a good candidate: retry once when the capture is invalid or
        // implausible for any sane controller.
        if ((!measured || metrics.overshoot_pct > 60.0f || metrics.ss_error_pct > 50.0f) &&
            iteration < MAX_REFINEMENTS && check_stop(start_us) == 0)
        {
            tuning::StepMetrics retry;
            const bool retry_measured = validate_step(wheel, target_rad_s, start_us, &retry);
            if (retry_measured &&
                (!measured || tuning::step_cost(retry) < tuning::step_cost(metrics)))
            {
                metrics = retry;
                measured = true;
            }
        }
        if (!measured && check_stop(start_us) != 0)
        {
            controller.apply_parameters(wheel, original);
            return false;
        }

        if (!best_metrics.valid || tuning::step_cost(metrics) < tuning::step_cost(best_metrics))
        {
            best_metrics = metrics;
            best_gains = gains;
            m_working.rise_ms[wheel] =
              static_cast<uint16_t>(std::clamp(metrics.rise_ms, 0.0f, 65535.0f));
            m_working.overshoot_pct[wheel] =
              static_cast<uint8_t>(std::clamp(metrics.overshoot_pct, 0.0f, 255.0f));
            m_working.ss_error_pct[wheel] =
              static_cast<uint8_t>(std::clamp(metrics.ss_error_pct, 0.0f, 255.0f));
            publish();
        }

        if (tuning::accepts_best_effort(metrics) &&
            (!fallback_metrics.valid ||
             tuning::step_cost(metrics) < tuning::step_cost(fallback_metrics)))
        {
            fallback_metrics = metrics;
            fallback_gains = gains;
        }

        if (tuning::accepts(metrics))
        {
            accepted = true;
            break;
        }
        gains = tuning::refine(plant, gains, metrics);
    }

    if (!accepted && fallback_metrics.valid)
    {
        ESP_LOGW(
          TAG, "%s: accepting the best acceptable candidate after refinement.", WHEEL_NAMES[wheel]);
        accepted = true;
        best_metrics = fallback_metrics;
        best_gains = fallback_gains;
    }
    if (!accepted)
    {
        ESP_LOGW(TAG, "%s: no candidate met the step limits; wheel failed.", WHEEL_NAMES[wheel]);
        m_working.error = Error::UNSTABLE;
        controller.apply_parameters(wheel, original);
        return false;
    }

    set_progress(wheel, Phase::STORE, 95);
    controller.apply_parameters(wheel, to_parameters(best_gains));
    controller.save_parameters(wheel);
    m_working.rise_ms[wheel] =
      static_cast<uint16_t>(std::clamp(best_metrics.rise_ms, 0.0f, 65535.0f));
    m_working.overshoot_pct[wheel] =
      static_cast<uint8_t>(std::clamp(best_metrics.overshoot_pct, 0.0f, 255.0f));
    m_working.ss_error_pct[wheel] =
      static_cast<uint8_t>(std::clamp(best_metrics.ss_error_pct, 0.0f, 255.0f));
    m_working.result_mask |= static_cast<uint8_t>(1U << wheel);
    publish();
    return true;
}

void WheelTuning::task_loop()
{
    const uint32_t start_us = static_cast<uint32_t>(esp_timer_get_time());
    bool stopped = false;

    for (int wheel = 0; wheel < 4; ++wheel)
    {
        if (check_stop(start_us) != 0)
        {
            stopped = true;
            break;
        }
        if (!tune_wheel(wheel, start_us))
        {
            if (check_stop(start_us) != 0)
            {
                stopped = true;
                break;
            }
            // Keep going with the remaining wheels.
        }
        set_progress(static_cast<uint8_t>(wheel), Phase::STORE, 100);
    }

    // Every exit path leaves the motors stopped and the control loop running.
    drive_wheel(-1, 0);
    WheelController & controller = WheelController::get_instance();
    controller.stop_position_moves();
    controller.set_open_loop(false);
    controller.set_target_velocities({0.0f, 0.0f, 0.0f, 0.0f});

    m_working.wheel = 4;
    m_working.progress = 100;
    const int stop_reason = check_stop(start_us);
    if (stopped || stop_reason == 1)
    {
        m_working.state = State::FAILED;
        m_working.phase = Phase::DONE;
        m_working.error = Error::ABORTED;
    }
    else if (stop_reason == 2)
    {
        m_working.state = State::FAILED;
        m_working.phase = Phase::DONE;
        m_working.error = Error::TIMEOUT;
    }
    else
    {
        // The routine itself completed: every wheel was attempted and at least
        // one produced valid parameters. Per-wheel failures stay visible in
        // result_mask (the dashboard lists them), and the last failure reason
        // is kept for the operator.
        m_working.state = (m_working.result_mask != 0) ? State::DONE : State::FAILED;
        m_working.phase = Phase::DONE;
        if (m_working.state == State::DONE)
            m_working.error = Error::NONE;
        else if (m_working.error == Error::NONE)
            m_working.error = Error::UNSTABLE;
    }
    publish();

    ESP_LOGI(TAG,
             "Control calibration finished: state=%d error=%d result=0x%X Kp=%.2f,%.2f,%.2f,%.2f",
             static_cast<int>(m_working.state),
             static_cast<int>(m_working.error),
             static_cast<unsigned>(m_working.result_mask),
             static_cast<double>(controller.get_parameters()[0].kp),
             static_cast<double>(controller.get_parameters()[1].kp),
             static_cast<double>(controller.get_parameters()[2].kp),
             static_cast<double>(controller.get_parameters()[3].kp));

    m_running.store(false);
    m_task_handle = nullptr;
    vTaskDelete(nullptr);
}
