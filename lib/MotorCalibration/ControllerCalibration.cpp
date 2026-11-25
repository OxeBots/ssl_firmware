#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <algorithm>
#include <cmath>

#include "BL48250.h"
#include "MotorCalibration.h"
#include "TuningMath.h"
#include "WheelController.h"
#include "wheel_state_estimator.h"

static const char * TAG = "MotorCalibration";

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
    std::nth_element(buffer.begin(), buffer.begin() + n / 2, buffer.begin() + n);
    return buffer[n / 2];
}

// Copies the calibration gains into the controller parameter struct, keeping
// the raw-angle orientation learned by the breakaway measurement.
static WheelController::Parameters to_parameters(const tuning::Gains & gains,
                                                 float angle_sign,
                                                 const tuning::PlantModel & forward,
                                                 const tuning::PlantModel & reverse,
                                                 WheelController::Parameters parameters)
{
    parameters.kp = gains.kp;
    parameters.ki = gains.ki;
    parameters.kd = gains.kd;
    parameters.feedforward = gains.feedforward;
    parameters.velocity_ff = gains.velocity_ff;
    parameters.static_friction = gains.static_friction;
    parameters.feedforward_rev = gains.feedforward_rev;
    parameters.velocity_ff_rev = gains.velocity_ff_rev;
    parameters.static_friction_rev = gains.static_friction_rev;
    parameters.angle_sign = angle_sign;
    parameters.continuous_min_rad_s = {forward.continuous_min_rad_s, reverse.continuous_min_rad_s};
    if (parameters.pulse_duty[0] == 0.0f)
    {
        parameters.pulse_duty[0] = forward.continuous_min_duty;
        parameters.pulse_interleaved[0] = true;
        parameters.pulse_coast_delay_s[0] = forward.dead_time_s;
    }
    if (parameters.pulse_duty[1] == 0.0f)
    {
        parameters.pulse_duty[1] = reverse.continuous_min_duty;
        parameters.pulse_interleaved[1] = true;
        parameters.pulse_coast_delay_s[1] = reverse.dead_time_s;
    }
    parameters.pulse_response_s = {forward.tau_s + forward.dead_time_s,
                                   reverse.tau_s + reverse.dead_time_s};
    return parameters;
}

// Per-wheel progress split of the whole routine.
static constexpr uint8_t WHEEL_PROGRESS_SPAN = 24;
static constexpr float RANGE_MEAN_ERROR_FRACTION = 0.12f;
static constexpr float ENTER_PULSE_FACTOR =
  1.0f - WheelController::PULSE_HANDOFF_HYSTERESIS_FRACTION;
static constexpr float SPEED_OBSERVATION_S = config::driver::BL48250_FB_WINDOW_MS * 1e-3f;
static constexpr float CONTINUOUS_RIPPLE_FRACTION = 0.12f;

esp_err_t MotorCalibration::start_controller_tuning()
{
    if (!reserve_routine(Routine::CONTROLLER_TUNING))
        return ESP_ERR_INVALID_STATE;

    m_controller_working = ControllerStatus();
    m_controller_working.state = State::RUNNING;
    publish_controller_status();

    auto & controller = WheelController::get_instance();
    // Validate against the drivetrain limit while measuring the shared limit.
    m_previous_max_velocity_rad_s = controller.get_max_velocity_rad_s();
    controller.set_max_velocity_rad_s(config::driver::WHEEL_MAX_VEL_RAD);
    // Tune the continuous controller with low-speed pulse mode disabled.
    controller.set_low_speed_mode_enabled(false);

    if (xTaskCreatePinnedToCore(controller_task_wrapper, "wheel_tune", 8192, this, 5, nullptr, 0) !=
        pdPASS)
    {
        controller.set_max_velocity_rad_s(m_previous_max_velocity_rad_s);
        controller.set_low_speed_mode_enabled(true);
        m_controller_working.state = State::FAILED;
        m_controller_working.error = ControllerError::ABORTED;
        publish_controller_status();
        release_routine();
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Control calibration started: the wheels spin one at a time.");
    return ESP_OK;
}

MotorCalibration::ControllerStatus MotorCalibration::get_controller_status() const
{
    ControllerStatus status;
    if (m_status_mutex != nullptr && xSemaphoreTake(m_status_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        status = m_controller_status;
        xSemaphoreGive(m_status_mutex);
    }
    return status;
}

void MotorCalibration::publish_controller_status()
{
    if (m_status_mutex == nullptr)
        return;
    if (xSemaphoreTake(m_status_mutex, pdMS_TO_TICKS(20)) != pdTRUE)
        return;
    m_controller_status = m_controller_working;
    xSemaphoreGive(m_status_mutex);
}

void MotorCalibration::set_controller_progress(uint8_t wheel,
                                               ControllerPhase phase,
                                               uint8_t wheel_progress)
{
    m_controller_working.wheel = wheel;
    m_controller_working.phase = phase;
    const uint8_t clamped = std::min<uint8_t>(wheel_progress, 100);
    m_controller_working.progress = static_cast<uint8_t>(
      std::min(100, wheel * WHEEL_PROGRESS_SPAN + clamped * WHEEL_PROGRESS_SPAN / 100));
    publish_controller_status();
}

void MotorCalibration::controller_task_wrapper(void * arg)
{
    static_cast<MotorCalibration *>(arg)->run_controller_tuning();
}

int MotorCalibration::check_stop(uint32_t start_us)
{
    if (abort_requested())
        return 1;
    const uint32_t elapsed_us = static_cast<uint32_t>(esp_timer_get_time()) - start_us;
    if (elapsed_us > CONTROLLER_RUN_BUDGET_US)
        return 2;
    return 0;
}

bool MotorCalibration::wait_ms(uint32_t duration_ms, uint32_t start_us)
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

void MotorCalibration::drive_wheel(int wheel, int duty)
{
    auto & driver = BL48250::get_instance();
    std::array<uint32_t, 4> duties = {};
    auto dirs = driver.get_directions();
    if (wheel >= 0 && wheel < 4 && duty != 0)
    {
        const int clamped = std::clamp(duty, -1023, 1023);
        duties[wheel] = static_cast<uint32_t>(std::abs(clamped));
        dirs[wheel] = (clamped >= 0) ? config::driver::MOTOR_CW : config::driver::MOTOR_CCW;
    }
    driver.set_duties(duties, dirs);
}

bool MotorCalibration::sample_velocity(int wheel,
                                       uint32_t duration_ms,
                                       uint32_t start_us,
                                       float * velocity)
{
    auto & estimator = WheelStateEstimator::get_instance();
    const int64_t start = esp_timer_get_time();
    const int64_t end = start + static_cast<int64_t>(duration_ms) * 1000;
    constexpr float ANGLE_PERIOD_RAD = 2.0f * static_cast<float>(M_PI);
    float previous = estimator.get_measured_angle_rad()[wheel];
    float angle_delta = 0.0f;
    while (esp_timer_get_time() < end)
    {
        vTaskDelay(1);
        if (check_stop(start_us) != 0)
            return false;
        const float angle = estimator.get_measured_angle_rad()[wheel];
        angle_delta += std::remainder(angle - previous, ANGLE_PERIOD_RAD);
        previous = angle;
    }
    const float elapsed_s = static_cast<float>(esp_timer_get_time() - start) * 1e-6f;
    *velocity = angle_delta / (elapsed_s * config::driver::WHEEL_REDUCTION);
    return true;
}

size_t MotorCalibration::sample_step(int wheel, uint32_t duration_ms, uint32_t start_us)
{
    auto & estimator = WheelStateEstimator::get_instance();
    const int64_t start = esp_timer_get_time();
    const int64_t end = start + static_cast<int64_t>(duration_ms) * 1000;
    size_t count = 0;
    while (esp_timer_get_time() < end && count < m_step_times_s.size())
    {
        // 100 Hz sampling: the 320-sample buffer covers 3.2 s. At one sample
        // per millisecond (FreeRTOS tick 1 kHz) it would cover only 320 ms and
        // the step would be measured while still accelerating.
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
        if (check_stop(start_us) != 0)
            break;
        const int64_t now = esp_timer_get_time();
        m_step_times_s[count] = static_cast<float>(now - start) / 1e6f;
        const auto measured = estimator.get_smoothed_velocity_rad_s();
        m_step_velocities[count] = -measured[wheel];
        count++;
    }
    return count;
}

bool MotorCalibration::measure_breakaway(
  int wheel, int direction, uint32_t start_us, int * breakaway_duty, float * angle_sign)
{
    *breakaway_duty = 0;
    const int sign_direction = (direction >= 0) ? 1 : -1;

    for (int duty = BREAKAWAY_START_DUTY; duty <= BREAKAWAY_MAX_DUTY; duty += BREAKAWAY_STEP_DUTY)
    {
        drive_wheel(wheel, sign_direction * duty);
        if (!wait_ms(BREAKAWAY_HOLD_MS, start_us))
            return false;
        float velocity = 0.0f;
        if (!sample_velocity(wheel, BREAKAWAY_HOLD_MS / 2, start_us, &velocity))
            return false;
        const float movement = sign_direction > 0 ? std::fabs(velocity) : -*angle_sign * velocity;
        if (movement > MOVEMENT_THRESHOLD_RAD_S)
        {
            *breakaway_duty = duty;
            // The forward run also learns the raw-angle orientation used by the
            // position control: let the wheel spin fast enough that the raw
            // angle clearly moves, then accumulate its deltas. The orientation
            // cannot be read at the breakaway duty, where the wheel barely
            // turns.
            if (sign_direction > 0)
            {
                const int sign_duty = std::clamp(duty + 300, 450, 900);
                drive_wheel(wheel, sign_duty);
                if (!sample_velocity(wheel, ANGLE_SIGN_SAMPLE_MS, start_us, &velocity))
                    return false;
                // Positive duty is a positive motor rotation: the raw angle may
                // increase or decrease depending on the magnet orientation.
                if (std::fabs(velocity) > MOVEMENT_THRESHOLD_RAD_S)
                    *angle_sign = velocity < 0.0f ? -1.0f : 1.0f;
            }
            drive_wheel(wheel, 0);
            if (!wait_for_stop(wheel, start_us))
                return false;
            ESP_LOGI(TAG,
                     "%s breakaway duty %s%d raw angle sign: %+.0f",
                     WHEEL_NAMES[wheel],
                     (sign_direction > 0) ? "+" : "-",
                     duty,
                     static_cast<double>(*angle_sign));
            return true;
        }
    }
    drive_wheel(wheel, 0);
    return true;
}

bool MotorCalibration::capture_plant_response(int wheel,
                                              int direction,
                                              uint32_t start_us,
                                              PlantMeasurement & measurement)
{
    static float filtered[STEP_SAMPLE_CAPACITY];
    drive_wheel(wheel, direction * measurement.duty_low);
    const size_t count_low = sample_step(wheel, IDENTIFY_HOLD_MS, start_us);
    if (count_low < IDENTIFY_STEADY_SAMPLES + 5)
        return false;
    measurement.velocity_low =
      median_of(m_step_velocities.data(), count_low, IDENTIFY_STEADY_SAMPLES);
    tuning::median_filter_step(m_step_velocities.data(), filtered, count_low);
    tuning::fit_step_dynamics(
      m_step_times_s.data(), filtered, count_low, &measurement.tau_s, &measurement.dead_time_s);

    drive_wheel(wheel, direction * measurement.duty_high);
    const size_t count_high = sample_step(wheel, IDENTIFY_HOLD_MS, start_us);
    if (count_high < IDENTIFY_STEADY_SAMPLES + 5)
        return false;
    measurement.velocity_high =
      median_of(m_step_velocities.data(), count_high, IDENTIFY_STEADY_SAMPLES);
    const auto high = tuning::measure_settled_speed(
      m_step_velocities.data() + count_high - IDENTIFY_STEADY_SAMPLES, IDENTIFY_STEADY_SAMPLES);
    measure_continuous_floor(wheel, direction, start_us, high, measurement);

    drive_wheel(wheel, 0);
    return wait_for_stop(wheel, start_us);
}

void MotorCalibration::measure_continuous_floor(int wheel,
                                                int direction,
                                                uint32_t start_us,
                                                const tuning::SettledSpeed & high,
                                                PlantMeasurement & measurement)
{
    const float sign = static_cast<float>(direction);
    const auto stable = [sign](const tuning::SettledSpeed & speed) {
        const float magnitude = sign * speed.mean_rad_s;
        return speed.valid && magnitude > RAMP_STOP_RAD_S &&
               speed.ripple_rad_s <= CONTINUOUS_RIPPLE_FRACTION * magnitude;
    };
    measurement.floor_speed = stable(high) ? sign * high.mean_rad_s : 0.0f;
    measurement.floor_duty = measurement.duty_high;
    if (measurement.floor_speed <= 0.0f)
        return;

    // Start near the lower measured duty, on the ramp's duty grid.
    const int ramp_start = measurement.duty_high - (measurement.duty_high - measurement.duty_low) /
                                                     RAMP_STEP_DUTY * RAMP_STEP_DUTY;
    for (int duty = ramp_start; duty >= RAMP_MIN_DUTY; duty -= RAMP_STEP_DUTY)
    {
        const auto speed = measure_open_loop_speed(wheel, direction * duty, start_us);
        if (!stable(speed))
            break;
        measurement.floor_speed = sign * speed.mean_rad_s;
        measurement.floor_duty = duty;
    }
}

bool MotorCalibration::fit_plant_model(int wheel,
                                       int attempt,
                                       int direction,
                                       int breakaway_duty,
                                       const PlantMeasurement & measurement,
                                       tuning::PlantModel & plant)
{
    // Signed samples give both directions a positive gain and a signed intercept.
    const float duties[2] = {static_cast<float>(direction * measurement.duty_low),
                             static_cast<float>(direction * measurement.duty_high)};
    const float sign = static_cast<float>(direction);
    const float steady[2] = {measurement.velocity_low, measurement.velocity_high};
    float gain = 0.0f;
    float friction = 0.0f;
    const bool fit_ok = tuning::fit_static_curve(duties, steady, 2, &gain, &friction);

    if (measurement.floor_speed > 0.0f && measurement.floor_duty < measurement.duty_high)
    {
        float low_gain = 0.0f;
        float low_friction = 0.0f;
        if (tuning::fit_plant_two_points(measurement.velocity_high,
                                         duties[1],
                                         sign * measurement.floor_speed,
                                         static_cast<float>(direction * measurement.floor_duty),
                                         &low_gain,
                                         &low_friction))
        {
            gain = low_gain;
            friction = low_friction;
        }
    }

    const float low_magnitude = std::fabs(measurement.velocity_low);
    const float high_magnitude = std::fabs(measurement.velocity_high);
    const bool moved_command_direction =
      sign * measurement.velocity_low > 0.0f && sign * measurement.velocity_high > 0.0f;
    ESP_LOGI(TAG,
             "%s identify %s attempt %d: duty %+d/%+d -> %.2f/%.2f rad/s, gain=%.5f tau=%.3f "
             "dead=%.3f",
             WHEEL_NAMES[wheel],
             direction > 0 ? "+" : "-",
             attempt,
             direction * measurement.duty_low,
             direction * measurement.duty_high,
             static_cast<double>(measurement.velocity_low),
             static_cast<double>(measurement.velocity_high),
             static_cast<double>(gain),
             static_cast<double>(measurement.tau_s),
             static_cast<double>(measurement.dead_time_s));

    const bool valid = fit_ok && measurement.tau_s > 0.0f && moved_command_direction &&
                       measurement.floor_speed > 0.0f && high_magnitude > low_magnitude + 0.5f;
    if (!valid)
        return false;

    plant.gain_rad_s_per_duty = gain;
    // The intercept accounts for friction while the wheel is turning.
    plant.friction_duty = friction;
    plant.breakaway_duty = static_cast<float>(breakaway_duty);
    plant.tau_s = measurement.tau_s;
    plant.dead_time_s = measurement.dead_time_s;
    plant.max_velocity_rad_s = std::max(low_magnitude, high_magnitude);
    plant.min_velocity_rad_s = std::min(low_magnitude, high_magnitude);
    plant.continuous_min_rad_s = measurement.floor_speed;
    plant.continuous_min_duty = static_cast<float>(measurement.floor_duty);
    plant.valid = true;
    return true;
}

bool MotorCalibration::identify_plant(
  int wheel, int breakaway_duty, int direction, uint32_t start_us, tuning::PlantModel * plant)
{
    const int sign_direction = direction >= 0 ? 1 : -1;
    for (int attempt = 0; attempt < 2; ++attempt)
    {
        const int duty_low = attempt == 0 ? std::clamp(breakaway_duty + 150, 250, 700)
                                          : std::clamp(breakaway_duty + 250, 350, 850);
        const int duty_high = attempt == 0 ? std::clamp(duty_low + 250, duty_low + 100, 950)
                                           : std::clamp(duty_low + 300, duty_low + 100, 980);
        PlantMeasurement measurement{duty_low, duty_high};
        if (!capture_plant_response(wheel, sign_direction, start_us, measurement))
            return false;
        if (fit_plant_model(wheel, attempt, sign_direction, breakaway_duty, measurement, *plant))
            return true;
    }
    return true;
}

tuning::SettledSpeed MotorCalibration::measure_open_loop_speed(int wheel,
                                                               int duty,
                                                               uint32_t start_us)
{
    drive_wheel(wheel, duty);
    if (!wait_ms(RAMP_STEP_MS, start_us))
        return {};
    const size_t count = sample_step(wheel, RAMP_SAMPLE_MS, start_us);
    return tuning::measure_settled_speed(m_step_velocities.data(), count);
}

bool MotorCalibration::wait_for_stop(int wheel, uint32_t start_us)
{
    constexpr float standstill_rad_s = 0.3f;
    constexpr int64_t standstill_us = 10000;
    auto & estimator = WheelStateEstimator::get_instance();
    const int64_t deadline = esp_timer_get_time() + static_cast<int64_t>(STOP_TIMEOUT_MS) * 1000;
    int64_t stopped_since = 0;
    while (esp_timer_get_time() < deadline && check_stop(start_us) == 0)
    {
        WheelStateEstimator::ControlSnapshot sample;
        bool stopped = estimator.get_control_snapshot(&sample);
        const int64_t now = esp_timer_get_time();
        for (int i = 0; stopped && i < static_cast<int>(sample.sample_us.size()); ++i)
        {
            if (wheel >= 0 && i != wheel)
                continue;
            const unsigned bit = 1U << i;
            stopped = (sample.smoothed_valid_mask & bit) != 0 &&
                      ((sample.stuck_analog_mask | sample.clamped_raw_mask) & bit) == 0 &&
                      sample.sample_us[i] > 0 &&
                      now - sample.sample_us[i] <= config::driver::BL48250_FB_WINDOW_MS * 1000 &&
                      std::fabs(sample.smoothed_velocity_rad_s[i]) <= standstill_rad_s;
        }
        if (!stopped)
            stopped_since = 0;
        else if (stopped_since == 0)
            stopped_since = now;
        else if (now - stopped_since >= standstill_us)
            return true;
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
    ESP_LOGW(
      TAG, "Standstill not confirmed for %s.", wheel < 0 ? "all wheels" : WHEEL_NAMES[wheel]);
    return false;
}

tuning::StepMetrics MotorCalibration::validate_step(int wheel,
                                                    float target_rad_s,
                                                    uint32_t start_us)
{
    auto & controller = WheelController::get_instance();
    if (!wait_for_stop(wheel, start_us))
        return {};

    std::array<float, 4> targets = {};
    targets[wheel] = target_rad_s;
    controller.set_target_velocities(targets);
    const size_t count = sample_step(wheel, VALIDATE_HOLD_MS, start_us);
    controller.set_target_velocities({});

    const auto metrics =
      tuning::analyze_step(m_step_times_s.data(), m_step_velocities.data(), count, target_rad_s);
    // NOLINTNEXTLINE(bugprone-branch-clone)
    ESP_LOGI(TAG,
             "%s step %+.1f RPM: rise %.0f ms, peak %.1f%%, error %.1f%%, ripple %.1f%%",
             WHEEL_NAMES[wheel],
             static_cast<double>(target_rad_s * RAD_S_TO_RPM),
             static_cast<double>(metrics.rise_ms),
             static_cast<double>(metrics.overshoot_pct),
             static_cast<double>(metrics.ss_error_pct),
             static_cast<double>(metrics.settled_ripple_pct));
    return metrics;
}

// Copies one validated step's metrics into the published status arrays.
static void set_step_metrics(std::array<uint16_t, 4> & rise_ms,
                             std::array<uint8_t, 4> & overshoot_pct,
                             std::array<uint8_t, 4> & ss_error_pct,
                             int wheel,
                             const tuning::StepMetrics & metrics)
{
    rise_ms[wheel] = static_cast<uint16_t>(std::clamp(metrics.rise_ms, 0.0f, 65535.0f));
    overshoot_pct[wheel] = static_cast<uint8_t>(std::clamp(metrics.overshoot_pct, 0.0f, 255.0f));
    ss_error_pct[wheel] = static_cast<uint8_t>(std::clamp(metrics.ss_error_pct, 0.0f, 255.0f));
}

bool MotorCalibration::validate_gains(int wheel,
                                      float target_rad_s,
                                      uint32_t start_us,
                                      tuning::Gains gains,
                                      WheelController::Parameters & parameters)
{
    auto & controller = WheelController::get_instance();

    for (uint8_t attempt = 0; attempt <= MAX_REFINEMENTS && check_stop(start_us) == 0; ++attempt)
    {
        set_controller_progress(wheel,
                                attempt == 0 ? ControllerPhase::VALIDATE : ControllerPhase::REFINE,
                                50 + 10 * attempt);
        parameters.kp = gains.kp;
        parameters.ki = gains.ki;
        parameters.velocity_ff = gains.velocity_ff;
        parameters.velocity_ff_rev = gains.velocity_ff_rev;
        controller.apply_parameters(wheel, parameters);

        const auto forward = validate_step(wheel, target_rad_s, start_us);
        const auto reverse = validate_step(wheel, -target_rad_s, start_us);
        set_step_metrics(m_controller_working.rise_ms,
                         m_controller_working.overshoot_pct,
                         m_controller_working.ss_error_pct,
                         wheel,
                         forward);
        set_step_metrics(m_controller_working.rise_ms_rev,
                         m_controller_working.overshoot_pct_rev,
                         m_controller_working.ss_error_pct_rev,
                         wheel,
                         reverse);
        publish_controller_status();
        if (tuning::accepts_best_effort_pair(forward, reverse))
            return true;
        if (forward.valid && reverse.valid)
            gains = tuning::refine(gains, forward, reverse);
    }
    return false;
}

std::array<tuning::SettledSpeed, 4> MotorCalibration::measure_tracking(
  const std::array<float, 4> & targets, uint32_t start_us, int wheel)
{
    constexpr size_t capacity = TRACKING_SAMPLE_MS / SAMPLE_PERIOD_MS + 1;
    static std::array<float, capacity> times;
    static std::array<std::array<float, capacity>, 4> angles;
    auto & controller = WheelController::get_instance();
    auto & driver = BL48250::get_instance();
    if (!wait_for_stop(wheel, start_us))
        return {};
    controller.set_target_velocities(targets);
    if (!wait_ms(TRACKING_SETTLE_MS, start_us))
    {
        controller.set_target_velocities({});
        return {};
    }

    const auto first_counts = driver.pulse_counts();
    const int64_t start = esp_timer_get_time();
    size_t count = 0;
    for (; count < times.size() && check_stop(start_us) == 0; ++count)
    {
        const auto position = controller.get_position_angles_rad();
        times[count] = static_cast<float>(esp_timer_get_time() - start) * 1e-6f;
        for (size_t i = 0; i < targets.size(); ++i) angles[i][count] = position[i];
        if (count + 1 < times.size())
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
    const auto final_counts = driver.pulse_counts();
    controller.set_target_velocities({});
    if (count < 2 || check_stop(start_us) != 0)
        return {};

    std::array<tuning::SettledSpeed, 4> results = {};
    const float duration = times[count - 1] - times[0];
    const float speed_per_count = config::driver::BL48250_FB_RAD_S_PER_COUNT / duration;
    for (size_t i = 0; i < results.size(); ++i)
    {
        if (targets[i] == 0.0f)
            continue;
        auto & speed = results[i];
        speed = tuning::measure_settled_travel(
          times.data(), angles[i].data(), count, SPEED_OBSERVATION_S);
        const int64_t edges = static_cast<int64_t>(final_counts[i]) - first_counts[i];
        if (edges > 0 &&
            std::fabs(static_cast<float>(edges) * speed_per_count - std::fabs(speed.mean_rad_s)) >
              tuning::MATCHED_SPEED_FRACTION * std::fabs(targets[i]) + speed_per_count)
            speed.valid = false;
        ESP_LOGI(TAG,
                 "%s %+.1f: mean %.1f [%.1f, %.1f], rms %.1f RPM%s",
                 WHEEL_NAMES[i],
                 static_cast<double>(targets[i] * RAD_S_TO_RPM),
                 static_cast<double>(speed.mean_rad_s * RAD_S_TO_RPM),
                 static_cast<double>(speed.min_rad_s * RAD_S_TO_RPM),
                 static_cast<double>(speed.max_rad_s * RAD_S_TO_RPM),
                 static_cast<double>(speed.ripple_rad_s * RAD_S_TO_RPM),
                 speed.valid ? "" : " (invalid feedback)");
    }
    return results;
}

void MotorCalibration::select_pulse_duties(int wheel,
                                           uint32_t start_us,
                                           WheelController::Parameters & parameters)
{
    auto & controller = WheelController::get_instance();
    constexpr float minimum = WheelController::MIN_SMOOTH_TARGET_RPM * RPM_TO_RAD_S;
    controller.set_low_speed_mode_enabled(true);
    for (size_t branch = 0; branch < parameters.pulse_duty.size() && check_stop(start_us) == 0;
         ++branch)
    {
        if (minimum > ENTER_PULSE_FACTOR * parameters.continuous_min_rad_s[branch])
            continue;
        const float direction = branch == 0 ? 1.0f : -1.0f;
        const float breakaway =
          branch == 0 ? parameters.static_friction : parameters.static_friction_rev;
        std::array<float, 4> targets = {};
        targets[wheel] = direction * minimum;
        controller.apply_parameters(wheel, parameters);
        const auto initial = measure_tracking(targets, start_us, wheel)[wheel];
        if (initial.valid &&
            direction * initial.mean_rad_s >= (1.0f - RANGE_MEAN_ERROR_FRACTION) * minimum)
            continue;

        const float old_duty = parameters.pulse_duty[branch];
        const float stronger =
          std::min(std::max(old_duty, breakaway) + RAMP_STEP_DUTY, WheelController::MAX_PULSE_DUTY);
        if (check_stop(start_us) != 0 || stronger <= old_duty)
            continue;
        parameters.pulse_duty[branch] = stronger;
        controller.apply_parameters(wheel, parameters);
        const auto trial = measure_tracking(targets, start_us, wheel)[wheel];
        if (!trial.valid || direction * trial.mean_rad_s <= 0.0f ||
            (initial.valid && std::fabs(trial.mean_rad_s - targets[wheel]) >=
                                std::fabs(initial.mean_rad_s - targets[wheel])))
            parameters.pulse_duty[branch] = old_duty;
    }
    controller.set_low_speed_mode_enabled(false);
}

bool MotorCalibration::measure_max_speed(int wheel,
                                         uint32_t start_us,
                                         float * forward_rad_s,
                                         float * reverse_rad_s)
{
    const auto measure = [&](int direction) {
        drive_wheel(wheel, direction * MAX_SPEED_FULL_DUTY);
        size_t count = 0;
        if (wait_ms(MAX_SPEED_SETTLE_MS, start_us))
            count = sample_step(wheel, MAX_SPEED_SAMPLE_MS, start_us);
        drive_wheel(wheel, 0);
        if (!wait_for_stop(wheel, start_us) || check_stop(start_us) != 0)
            return 0.0f;
        const auto steady = tuning::measure_settled_speed(m_step_velocities.data(), count);
        return steady.valid ? direction * steady.mean_rad_s : 0.0f;
    };
    *forward_rad_s = measure(1);
    if (*forward_rad_s < MOVEMENT_THRESHOLD_RAD_S)
        return false;
    *reverse_rad_s = measure(-1);
    return *reverse_rad_s >= MOVEMENT_THRESHOLD_RAD_S;
}

MotorCalibration::ControllerError MotorCalibration::tune_wheel(
  int wheel, uint32_t start_us, WheelController::Parameters & parameters)
{
    auto & controller = WheelController::get_instance();
    set_controller_progress(wheel, ControllerPhase::FRICTION, 0);
    controller.set_open_loop(true);
    if (!wait_for_stop(wheel, start_us))
        return ControllerError::UNSTABLE;

    int forward_duty = 0;
    int reverse_duty = 0;
    float angle_sign = parameters.angle_sign;
    if (!measure_breakaway(wheel, 1, start_us, &forward_duty, &angle_sign) ||
        !measure_breakaway(wheel, -1, start_us, &reverse_duty, &angle_sign) || forward_duty == 0 ||
        reverse_duty == 0)
        return ControllerError::NO_MOVEMENT;

    set_controller_progress(wheel, ControllerPhase::MAX_SPEED, 15);
    float forward_max = 0.0f;
    float reverse_max = 0.0f;
    if (!measure_max_speed(wheel, start_us, &forward_max, &reverse_max))
        return ControllerError::NO_MOVEMENT;
    const float wheel_max = std::min(forward_max, reverse_max);
    m_controller_working.wheel_max_rpm[wheel] =
      static_cast<uint16_t>(std::clamp(std::lround(wheel_max * RAD_S_TO_RPM * 10), 0L, 65535L));
    ESP_LOGI(TAG,
             "%s maximum speed: %.1f RPM forward, %.1f RPM reverse",
             WHEEL_NAMES[wheel],
             static_cast<double>(forward_max * RAD_S_TO_RPM),
             static_cast<double>(reverse_max * RAD_S_TO_RPM));

    set_controller_progress(wheel, ControllerPhase::IDENTIFY, 25);
    tuning::PlantModel forward;
    tuning::PlantModel reverse;
    if (!identify_plant(wheel, forward_duty, 1, start_us, &forward) || !forward.valid ||
        !identify_plant(wheel, reverse_duty, -1, start_us, &reverse) || !reverse.valid)
        return ControllerError::UNSTABLE;

    const float floor = std::max(forward.continuous_min_rad_s, reverse.continuous_min_rad_s);
    const float target = std::min(0.90f * wheel_max,
                                  std::max(1.08f * floor,
                                           std::min(tuning::choose_validation_target(forward),
                                                    tuning::choose_validation_target(reverse))));
    const auto gains = tuning::synthesize(forward, reverse, target);
    const auto previous_pulse_duty = parameters.pulse_duty;
    parameters = to_parameters(gains, angle_sign, forward, reverse, parameters);
    for (size_t branch = 0; branch < previous_pulse_duty.size(); ++branch)
        if (previous_pulse_duty[branch] == 0.0f)
            parameters.pulse_floor_duty[branch] =
              std::max(0.0f, parameters.pulse_duty[branch] - RAMP_STEP_DUTY);
    controller.set_open_loop(false);
    if (!validate_gains(wheel, target, start_us, gains, parameters))
        return ControllerError::UNSTABLE;

    set_controller_progress(wheel, ControllerPhase::VALIDATE, 80);
    select_pulse_duties(wheel, start_us, parameters);
    return ControllerError::NONE;
}

bool MotorCalibration::update_max_speed_limit(uint32_t start_us)
{
    auto & controller = WheelController::get_instance();
    controller.set_max_velocity_rad_s(m_previous_max_velocity_rad_s);
    const auto & maxima = m_controller_working.wheel_max_rpm;
    const float rpm10 =
      static_cast<float>(tuning::calibrated_max_rpm(maxima.data(), maxima.size()));
    if (rpm10 < WheelController::MIN_SMOOTH_TARGET_RPM * 10 || check_stop(start_us) != 0)
        return false;

    controller.set_max_velocity_rad_s(rpm10 * 0.1f * RPM_TO_RAD_S);
    if (controller.save_max_velocity() != ESP_OK)
    {
        controller.set_max_velocity_rad_s(m_previous_max_velocity_rad_s);
        m_controller_working.error = ControllerError::SAVE_FAILED;
        return false;
    }
    return true;
}

bool MotorCalibration::verify_shared_speeds(uint32_t start_us)
{
    auto & controller = WheelController::get_instance();
    const float limit = controller.get_max_velocity_rad_s();
    constexpr float minimum = WheelController::MIN_SMOOTH_TARGET_RPM * RPM_TO_RAD_S;
    const std::array<float, 3> speeds = {limit, minimum, 2.0f * minimum};
    bool matched = true;
    for (size_t index = 0; index < speeds.size() && check_stop(start_us) == 0; ++index)
    {
        const float speed = speeds[index];
        if (index != 0 && speed >= limit)
            continue;
        std::array<float, 8> means = {};
        for (size_t branch = 0; branch < 2; ++branch)
        {
            const float direction = branch == 0 ? 1.0f : -1.0f;
            std::array<float, 4> targets;
            targets.fill(direction * speed);
            const auto measured = measure_tracking(targets, start_us, -1);
            for (size_t wheel = 0; wheel < targets.size(); ++wheel)
            {
                const auto & result = measured[wheel];
                means[branch * targets.size() + wheel] = direction * result.mean_rad_s;
                matched &= result.valid;
            }
        }
        if (tuning::unmatched_speed_mask(means, speed) != 0)
        {
            ESP_LOGW(TAG,
                     "Shared mean speeds differ at %.1f RPM.",
                     static_cast<double>(speed * RAD_S_TO_RPM));
            matched = false;
        }
    }
    return matched;
}

void MotorCalibration::run_controller_tuning()
{
    auto & controller = WheelController::get_instance();
    const uint32_t start_us = static_cast<uint32_t>(esp_timer_get_time());
    for (int wheel = 0; wheel < 4 && check_stop(start_us) == 0; ++wheel)
    {
        const auto original = controller.get_parameters()[wheel];
        auto parameters = original;
        ControllerError error = tune_wheel(wheel, start_us, parameters);
        controller.set_target_velocities({});
        if (controller.is_open_loop())
        {
            drive_wheel(-1, 0);
            controller.set_open_loop(false);
        }
        if (error == ControllerError::NONE && check_stop(start_us) == 0)
        {
            set_controller_progress(wheel, ControllerPhase::STORE, 95);
            controller.apply_parameters(wheel, parameters);
            if (controller.save_parameters(wheel) == ESP_OK)
            {
                m_controller_working.result_mask |= static_cast<uint8_t>(1U << wheel);
                set_controller_progress(wheel, ControllerPhase::STORE, 100);
                continue;
            }
            error = ControllerError::SAVE_FAILED;
        }
        if (m_controller_working.error != ControllerError::SAVE_FAILED)
            m_controller_working.error = error;
        ESP_LOGW(TAG, "%s kept its previous parameters.", WHEEL_NAMES[wheel]);
        controller.apply_parameters(wheel, original);
    }

    controller.set_low_speed_mode_enabled(true);
    const bool limit_saved = update_max_speed_limit(start_us);
    const bool all_saved = m_controller_working.result_mask == 0x0F;
    const bool complete = limit_saved && all_saved;
    if (complete)
    {
        m_controller_working.wheel = 4;
        m_controller_working.phase = ControllerPhase::VALIDATE;
        publish_controller_status();
        m_controller_working.error =
          verify_shared_speeds(start_us) ? ControllerError::NONE : ControllerError::SPEED_MISMATCH;
    }

    controller.set_target_velocities({});
    const int stop_reason = check_stop(start_us);
    if (stop_reason != 0)
        m_controller_working.error =
          stop_reason == 2 ? ControllerError::TIMEOUT : ControllerError::ABORTED;
    else if (!complete && m_controller_working.error == ControllerError::NONE)
        m_controller_working.error = ControllerError::UNSTABLE;
    m_controller_working.state = complete && stop_reason == 0 ? State::DONE : State::FAILED;
    m_controller_working.wheel = 4;
    m_controller_working.phase = ControllerPhase::DONE;
    m_controller_working.progress = 100;
    publish_controller_status();
    // NOLINTNEXTLINE(bugprone-branch-clone)
    ESP_LOGI(TAG,
             "Control calibration finished: saved mask=0x%X, error=%d.",
             static_cast<unsigned>(m_controller_working.result_mask),
             static_cast<int>(m_controller_working.error));
    release_routine();
    vTaskDelete(nullptr);
}
