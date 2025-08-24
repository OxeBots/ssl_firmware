#include "hal/adc_reader.hpp"

#include <cmath>
#include <memory>

#include "vt_kalman"
#include "vt_linalg"

#define ADC_GET_CHANNEL(p_data) ((p_data)->type1.channel)
#define ADC_GET_DATA(p_data) ((p_data)->type1.data)

// EKF types and state definition are now fully contained in the .cpp file
static constexpr int STATE_DIM = 3;    // [angle, velocity, acceleration]
static constexpr int MEAS_DIM = 1;     // [angle]
static constexpr int CONTROL_DIM = 1;  // [dt]

using EKF = vt::extended_kalman_filter_t<STATE_DIM, MEAS_DIM, CONTROL_DIM>;
using StateVector = vt::numeric_vector<STATE_DIM>;
using MeasurementVector = vt::numeric_vector<MEAS_DIM>;
using ControlVector = vt::numeric_vector<CONTROL_DIM>;

// --- Kalman Filter Tuning Parameters ---
// Process Noise (Q): How much we trust the model's prediction.
// Higher values allow for faster changes and quicker convergence.
// We expect velocity and acceleration to change more than the angle.
static vt::numeric_matrix<STATE_DIM, STATE_DIM> Q =
  vt::numeric_matrix<STATE_DIM, STATE_DIM>::diagonals({0.001, 0.1, 10.0});

// Measurement Noise (R): How much we trust the sensor reading.
// A lower value means we trust the measurement more.
static vt::numeric_matrix<MEAS_DIM, MEAS_DIM> R =
  vt::numeric_matrix<MEAS_DIM, MEAS_DIM>::diagonals(0.1);

// The full definition of KalmanState is now an implementation detail
struct ADC_Reader::KalmanState
{
    StateVector state_vec;
    EKF filter;
    int64_t last_update_us = 0;
    bool initialized = false;

    KalmanState()
    : state_vec(vt::make_numeric_vector({0.0, 0.0, 0.0})),
      filter(f_func, Fj_func, h_func, Hj_func, Q, R, state_vec)
    {
    }

    // --- Kalman Filter Model Definition ---
    static vt::numeric_vector<3> f_func(const vt::numeric_vector<3> & x,
                                        const vt::numeric_vector<1> & u)
    {
        float dt = u[0];
        float hdts = 0.5f * dt * dt;
        return vt::make_numeric_vector(
          {x[0] + dt * x[1] + hdts * x[2], x[1] + dt * x[2], x[2]});
    }

    static vt::numeric_matrix<3, 3> Fj_func(const vt::numeric_vector<3> &,
                                            const vt::numeric_vector<1> & u)
    {
        float dt = u[0];
        float hdts = 0.5f * dt * dt;
        return vt::make_numeric_matrix<3, 3>(
          {{1, dt, hdts}, {0, 1, dt}, {0, 0, 1}});
    }

    static vt::numeric_vector<1> h_func(const vt::numeric_vector<3> & x)
    {
        return vt::make_numeric_vector({x[0]});
    }

    static vt::numeric_matrix<1, 3> Hj_func(const vt::numeric_vector<3> &)
    {
        return vt::make_numeric_matrix<1, 3>({{1, 0, 0}});
    }
};

namespace
{
// Helper to normalize angle to [-PI, PI]
float normalize_angle(float angle)
{
    return angle - (2.0f * M_PI) * std::floor((angle + M_PI) / (2.0f * M_PI));
}
}  // namespace

ADC_Reader::ADC_Reader()
: m_adc_handle(NULL), m_task_handle(NULL), m_initialized(false)
{
    m_active_channels.fill(false);
}

ADC_Reader::~ADC_Reader()
{
    if (m_initialized)
    {
        adc_continuous_stop(m_adc_handle);
        adc_continuous_deinit(m_adc_handle);
    }
    if (m_task_handle) vTaskDelete(m_task_handle);
}

esp_err_t ADC_Reader::init(const std::vector<adc_channel_t> & channels)
{
    if (m_initialized) return ESP_ERR_INVALID_STATE;

    for (const auto & ch : channels)
    {
        if (static_cast<int>(ch) < ADC1_CHANNEL_MAX)
        {
            m_active_channels[ch] = true;
            m_kalman_states[ch] = std::make_unique<KalmanState>();
        }
    }

    // --- ADC Hardware Configuration ---
    adc_continuous_handle_cfg_t handle_config = {
      .max_store_buf_size = ADC_BUFFER_SIZE * 4,
      .conv_frame_size =
        static_cast<uint32_t>(channels.size() * SOC_ADC_DIGI_RESULT_BYTES),
      .flags = {.flush_pool = true},
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_config, &m_adc_handle));

    std::vector<adc_digi_pattern_config_t> pattern_config(channels.size());
    for (size_t i = 0; i < channels.size(); ++i)
    {
        pattern_config[i] = {
          .atten = ADC_ATTEN_DB_12,
          .channel = (uint8_t)(channels[i] & 0x7),
          .unit = ADC_UNIT_1,
          .bit_width = ADC_BITWIDTH_12,
        };
    }

    adc_continuous_config_t adc_config = {
      .pattern_num = (uint32_t)channels.size(),
      .adc_pattern = pattern_config.data(),
      .sample_freq_hz = SOC_ADC_SAMPLE_FREQ_THRES_LOW * channels.size(),
      .conv_mode = ADC_CONV_SINGLE_UNIT_1,
      .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    ESP_ERROR_CHECK(adc_continuous_config(m_adc_handle, &adc_config));

    adc_continuous_evt_cbs_t cb_config = {.on_conv_done = s_adc_callback,
                                          .on_pool_ovf = NULL};
    ESP_ERROR_CHECK(
      adc_continuous_register_event_callbacks(m_adc_handle, &cb_config, this));

    xTaskCreate(s_adc_task_wrapper, "Task_ADC_Reader", 4096, this, 15,
                &m_task_handle);

    m_initialized = true;
    ESP_ERROR_CHECK(adc_continuous_start(m_adc_handle));

    return ESP_OK;
}

// --- Public API Getters ---

float ADC_Reader::get_filtered_angle_deg(adc_channel_t channel)
{
    if (static_cast<int>(channel) < ADC1_CHANNEL_MAX &&
        m_active_channels[channel])
    {
        const auto & state = m_kalman_states[channel]->filter.state_vector;
        return normalize_angle(state[0]) * (180.0f / M_PI);
    }
    return 0.0f;
}

float ADC_Reader::get_filtered_rpm(adc_channel_t channel)
{
    if (static_cast<int>(channel) < ADC1_CHANNEL_MAX &&
        m_active_channels[channel])
    {
        const auto & state = m_kalman_states[channel]->filter.state_vector;
        return abs(state[1] * RAD_S_TO_RPM) <= 60.0f
                 ? 0.0f
                 : state[1] * RAD_S_TO_RPM;
    }
    return 0.0f;
}

float ADC_Reader::get_filtered_acceleration_rps2(adc_channel_t channel)
{
    if (static_cast<int>(channel) < ADC1_CHANNEL_MAX &&
        m_active_channels[channel])
    {
        const auto & state = m_kalman_states[channel]->filter.state_vector;
        return state[2] / (2.0f * M_PI);  // convert rad/s^2 to rev/s^2
    }
    return 0.0f;
}

uint32_t ADC_Reader::get_value(adc_channel_t channel)
{
    if (static_cast<int>(channel) < ADC1_CHANNEL_MAX &&
        m_active_channels[channel])
        return m_current_measurements[channel].value;
    return 0;
}

int64_t ADC_Reader::get_measurement_duration_us(adc_channel_t channel)
{
    if (static_cast<int>(channel) < ADC1_CHANNEL_MAX &&
        m_active_channels[channel])
        return m_current_measurements[channel].duration_us;
    return 0;
}

// --- Task and ISR ---

bool IRAM_ATTR ADC_Reader::s_adc_callback(
  adc_continuous_handle_t handle, const adc_continuous_evt_data_t * edata,
  void * user_data)
{
    auto * driver = static_cast<ADC_Reader *>(user_data);
    BaseType_t mustYield = pdFALSE;
    vTaskNotifyGiveFromISR(driver->m_task_handle, &mustYield);
    return (mustYield == pdTRUE);
}

void ADC_Reader::s_adc_task_wrapper(void * param)
{
    static_cast<ADC_Reader *>(param)->adc_task();
}

void ADC_Reader::adc_task()
{
    uint8_t buf[ADC_BUFFER_SIZE] = {0};

    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint32_t ret_num = 0;
        esp_err_t ret;
        do
        {
            ret =
              adc_continuous_read(m_adc_handle, buf, sizeof(buf), &ret_num, 0);

            if (ret == ESP_OK)
            {
                for (size_t i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
                {
                    auto * p = (adc_digi_output_data_t *)&buf[i];
                    auto chan_num = (adc_channel_t)ADC_GET_CHANNEL(p);
                    uint16_t data = ADC_GET_DATA(p);

                    if (static_cast<int>(chan_num) >= ADC1_CHANNEL_MAX ||
                        !m_active_channels[chan_num])
                        continue;

                    SamplingState & state = m_sampling_states[chan_num];
                    if (state.count < OVERSAMPLE_COUNT)
                    {
                        state.samples[state.count++] = data;
                    }

                    if (state.count >= OVERSAMPLE_COUNT)
                    {
                        uint32_t sum = 0;
                        for (size_t k = 0; k < OVERSAMPLE_COUNT; ++k)
                            sum += state.samples[k];

                        uint32_t avg_val = sum / OVERSAMPLE_COUNT;
                        m_current_measurements[chan_num].value = avg_val;

                        // --- KALMAN FILTER STEP ---
                        KalmanState & k_state = *m_kalman_states[chan_num];
                        int64_t now = esp_timer_get_time();

                        if (!k_state.initialized)
                        {
                            float initial_angle =
                              (avg_val & 0x0FFF) * RAW_TO_RAD;
                            k_state.state_vec = vt::make_numeric_vector(
                              {initial_angle, 0.0, 0.0});
                            k_state.last_update_us = now;
                            k_state.initialized = true;
                        }
                        else
                        {
                            float dt = (now - k_state.last_update_us) / 1e6f;
                            k_state.filter.predict(
                              vt::make_numeric_vector({dt}));
                            k_state.last_update_us = now;

                            float measured_angle =
                              (avg_val & 0x0FFF) * RAW_TO_RAD;

                            auto & predicted_state =
                              k_state.filter.state_vector;
                            float predicted_angle = predicted_state[0];
                            float innovation = normalize_angle(
                              measured_angle - predicted_angle);
                            float corrected_measurement =
                              predicted_angle + innovation;

                            k_state.filter.update(vt::make_numeric_vector(
                              {corrected_measurement}));
                        }
                        state.count = 0;
                    }
                }
            }
        } while (ret == ESP_OK && ret_num > 0);
    }
}
