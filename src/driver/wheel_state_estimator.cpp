#include "driver/wheel_state_estimator.hpp"

#include <cmath>
#include <memory>

#include "helper_func.h"
#include "vt_kalman"
#include "vt_linalg"

// AS5600 Register Addresses
#define AS5600_REG_CONF_H 0x07
#define AS5600_REG_RAWANGLE_H 0x0C
#define AS5600_ADDR 0x36

#define ADC_GET_CHANNEL(p_data) ((p_data)->type1.channel)
#define ADC_GET_DATA(p_data) ((p_data)->type1.data)

// --- EKF Implementation Details ---

// EKF types and state definition are fully contained in the .cpp file
static constexpr int STATE_DIM =
  3;  // State vector [angle, velocity, acceleration]
static constexpr int MEAS_DIM = 1;     // Measurement vector [angle]
static constexpr int CONTROL_DIM = 1;  // Control vector [dt]

using EKF = vt::extended_kalman_filter_t<STATE_DIM, MEAS_DIM, CONTROL_DIM>;
using StateVector = vt::numeric_vector<STATE_DIM>;
using MeasurementVector = vt::numeric_vector<MEAS_DIM>;
using ControlVector = vt::numeric_vector<CONTROL_DIM>;

/**
 * @brief Process Noise Covariance Matrix (Q).
 * This matrix represents the uncertainty in the process model. A higher value
 * means we trust the model less and allow the state to change more rapidly.
 * - Angle (pos): Low noise, as we don't expect it to jump randomly.
 * - Velocity (vel): Medium noise, as velocity can change.
 * - Acceleration (accel): High noise, as acceleration can be very dynamic.
 * This tuning makes the filter more responsive to changes in speed.
 */
static vt::numeric_matrix<STATE_DIM, STATE_DIM> Q =
  vt::numeric_matrix<STATE_DIM, STATE_DIM>::diagonals({0.001, 0.1, 10.0});

/**
 * @brief Measurement Noise Covariance Matrix (R).
 * This matrix represents the uncertainty in the sensor measurement. A lower
 * value means we trust the sensor reading more. A slightly higher value helps
 * smooth out sensor noise.
 */
static vt::numeric_matrix<MEAS_DIM, MEAS_DIM> R =
  vt::numeric_matrix<MEAS_DIM, MEAS_DIM>::diagonals(0.1);

/**
 * @brief The full definition of KalmanState, hidden from the header.
 * This struct contains the EKF instance and all related state variables.
 */
struct WheelStateEstimator::KalmanState
{
    StateVector state_vec;
    EKF filter;
    int64_t last_update_us = 0;
    bool initialized = false;

    /**
     * @brief Construct a new Kalman State object.
     * Initializes the state vector and constructs the EKF, passing references
     * to the model functions and noise matrices.
     */
    KalmanState()
    : state_vec(vt::make_numeric_vector({0.0, 0.0, 0.0})),
      filter(f_func, Fj_func, h_func, Hj_func, Q, R, state_vec)
    {
    }

    // --- Kalman Filter Model Definition ---

    /**
     * @brief State transition function `f(x, u)`.
     * Predicts the next state based on the current state `x` and control input
     * `u` (which is delta time `dt`). This is a constant acceleration model.
     */
    static vt::numeric_vector<3> f_func(const vt::numeric_vector<3> & x,
                                        const vt::numeric_vector<1> & u)
    {
        float dt = u[0];
        float hdts = 0.5f * dt * dt;
        // x_new = x + v*dt + 0.5*a*dt^2
        // v_new = v + a*dt
        // a_new = a
        return vt::make_numeric_vector(
          {x[0] + dt * x[1] + hdts * x[2], x[1] + dt * x[2], x[2]});
    }

    /**
     * @brief Jacobian of the state transition function `Fj(x, u)`.
     * The partial derivatives of `f` with respect to the state variables.
     */
    static vt::numeric_matrix<3, 3> Fj_func(const vt::numeric_vector<3> &,
                                            const vt::numeric_vector<1> & u)
    {
        float dt = u[0];
        float hdts = 0.5f * dt * dt;
        return vt::make_numeric_matrix<3, 3>(
          {{1, dt, hdts}, {0, 1, dt}, {0, 0, 1}});
    }

    /**
     * @brief Measurement function `h(x)`.
     * Maps the state vector `x` to the measurement space. We only measure the
     * angle.
     */
    static vt::numeric_vector<1> h_func(const vt::numeric_vector<3> & x)
    {
        return vt::make_numeric_vector({x[0]});
    }

    /**
     * @brief Jacobian of the measurement function `Hj(x)`.
     * The partial derivatives of `h` with respect to the state variables.
     */
    static vt::numeric_matrix<1, 3> Hj_func(const vt::numeric_vector<3> &)
    {
        return vt::make_numeric_matrix<1, 3>({{1, 0, 0}});
    }
};

// --- Class Method Implementations ---

WheelStateEstimator::WheelStateEstimator()
: m_adc_handle(NULL),
  m_task_handle(NULL),
  m_initialized(false),
  m_i2c_initialized(false),
  m_i2c_bus_handle(NULL),
  m_i2c_dev_handle(NULL)
{
    m_active_channels.fill(false);
}

WheelStateEstimator::~WheelStateEstimator()
{
    if (m_initialized)
    {
        adc_continuous_stop(m_adc_handle);
        adc_continuous_deinit(m_adc_handle);
    }
    if (m_task_handle) vTaskDelete(m_task_handle);

    if (m_i2c_initialized)
    {
        i2c_master_bus_rm_device(m_i2c_dev_handle);
        i2c_del_master_bus(m_i2c_bus_handle);
    }
}

esp_err_t WheelStateEstimator::init(
  const std::vector<adc_channel_t> & channels)
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

    xTaskCreate(s_adc_task_wrapper, "StateEstimatorTask", 4096, this, 15,
                &m_task_handle);

    m_initialized = true;
    ESP_ERROR_CHECK(adc_continuous_start(m_adc_handle));

    return ESP_OK;
}

esp_err_t WheelStateEstimator::init_i2c(i2c_port_t i2c_port,
                                        gpio_num_t sda_pin, gpio_num_t scl_pin)
{
    if (!m_initialized) return ESP_ERR_INVALID_STATE;
    if (m_i2c_initialized) return ESP_ERR_INVALID_STATE;

    i2c_master_bus_config_t i2c_mst_config = {
      .i2c_port = i2c_port,
      .sda_io_num = sda_pin,
      .scl_io_num = scl_pin,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags = {.enable_internal_pullup = true},
    };
    esp_err_t ret = i2c_new_master_bus(&i2c_mst_config, &m_i2c_bus_handle);

    if (ret != ESP_OK) return ret;

    i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = AS5600_ADDR,
      .scl_speed_hz = 100000,  // 100 kHz (std mode)
    };

    ret =
      i2c_master_bus_add_device(m_i2c_bus_handle, &dev_cfg, &m_i2c_dev_handle);

    if (ret != ESP_OK) return ret;
    uint8_t test_data;
    ret = read_register(AS5600_REG_CONF_H, &test_data, 1);

    if (ret == ESP_OK)
        m_i2c_initialized = true;
    else
        i2c_master_bus_rm_device(m_i2c_dev_handle);

    return ret;
}

float WheelStateEstimator::get_filtered_angle_deg(adc_channel_t channel)
{
    if (static_cast<int>(channel) < ADC1_CHANNEL_MAX &&
        m_active_channels[channel])
    {
        const auto & state = m_kalman_states[channel]->filter.state_vector;
        return normalize_angle(state[0]) * (180.0f / M_PI);
    }
    return 0.0f;
}

float WheelStateEstimator::get_filtered_rpm(adc_channel_t channel)
{
    if (static_cast<int>(channel) < ADC1_CHANNEL_MAX &&
        m_active_channels[channel])
    {
        const auto & state = m_kalman_states[channel]->filter.state_vector;
        // Apply a deadband to filter out noise when stationary
        return std::abs(state[1] * RAD_S_TO_RPM) <= 60.0f
                 ? 0.0f
                 : state[1] * RAD_S_TO_RPM;
    }
    return 0.0f;
}

float WheelStateEstimator::get_filtered_acceleration_rps2(
  adc_channel_t channel)
{
    if (static_cast<int>(channel) < ADC1_CHANNEL_MAX &&
        m_active_channels[channel])
    {
        const auto & state = m_kalman_states[channel]->filter.state_vector;
        return state[2] / (2.0f * M_PI);  // convert rad/s^2 to rev/s^2
    }
    return 0.0f;
}

uint32_t WheelStateEstimator::get_value(adc_channel_t channel)
{
    if (static_cast<int>(channel) < ADC1_CHANNEL_MAX &&
        m_active_channels[channel])
        return m_current_measurements[channel].value;
    return 0;
}

int64_t WheelStateEstimator::get_measurement_duration_us(adc_channel_t channel)
{
    if (static_cast<int>(channel) < ADC1_CHANNEL_MAX &&
        m_active_channels[channel])
        return m_current_measurements[channel].duration_us;
    return 0;
}

bool IRAM_ATTR WheelStateEstimator::s_adc_callback(
  adc_continuous_handle_t handle, const adc_continuous_evt_data_t * edata,
  void * user_data)
{
    auto * driver = static_cast<WheelStateEstimator *>(user_data);
    BaseType_t mustYield = pdFALSE;
    vTaskNotifyGiveFromISR(driver->m_task_handle, &mustYield);
    return (mustYield == pdTRUE);
}

void WheelStateEstimator::s_adc_task_wrapper(void * param)
{
    static_cast<WheelStateEstimator *>(param)->adc_task();
}

void WheelStateEstimator::adc_task()
{
    uint8_t buf[ADC_BUFFER_SIZE] = {0};

    while (true)
    {
        // Wait indefinitely for a notification from the ADC ISR
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint32_t ret_num = 0;
        esp_err_t ret;
        do
        {
            // Read all available data from the ADC driver's buffer
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

                    // --- Oversampling Step ---
                    SamplingState & state = m_sampling_states[chan_num];
                    if (state.count < OVERSAMPLE_COUNT)
                        state.samples[state.count++] = data;

                    // When the sampling buffer is full, process the data
                    if (state.count >= OVERSAMPLE_COUNT)
                    {
                        uint32_t sum = 0;
                        for (size_t k = 0; k < OVERSAMPLE_COUNT; ++k)
                            sum += state.samples[k];

                        uint16_t avg_val = sum / OVERSAMPLE_COUNT;
                        m_current_measurements[chan_num].value = avg_val;

                        // --- KALMAN FILTER STEP ---
                        KalmanState & k_state = *m_kalman_states[chan_num];
                        int64_t now = esp_timer_get_time();

                        if (!k_state.initialized)
                        {
                            // First measurement: initialize the filter state
                            float initial_angle =
                              (avg_val & 0x0FFF) * RAW_TO_RAD;
                            k_state.state_vec = vt::make_numeric_vector(
                              {initial_angle, 0.0, 0.0});
                            k_state.last_update_us = now;
                            k_state.initialized = true;
                        }
                        else
                        {
                            // --- Predict Step ---
                            float dt = (now - k_state.last_update_us) / 1e6f;
                            k_state.filter.predict(
                              vt::make_numeric_vector({dt}));
                            k_state.last_update_us = now;

                            // --- Update Step ---
                            float measured_angle =
                              (avg_val & 0x0FFF) * RAW_TO_RAD;

                            // Correct for angle wraparound (e.g., from 2PI to
                            // 0) This is crucial for circular quantities.
                            auto const & predicted_state =
                              k_state.filter.state_vector;
                            float predicted_angle = predicted_state[0];
                            float innovation = normalize_angle(
                              measured_angle - predicted_angle);
                            float corrected_measurement =
                              predicted_angle + innovation;

                            k_state.filter.update(vt::make_numeric_vector(
                              {corrected_measurement}));
                        }
                        // Reset sample count for the next window
                        state.count = 0;
                    }
                }
            }
        } while (ret == ESP_OK && ret_num > 0);
    }
}

esp_err_t WheelStateEstimator::setOutputStage(as5600_output_stage_t stage)
{
    uint16_t config;
    esp_err_t ret = read_config_register(&config);
    if (ret != ESP_OK) return ret;

    config &= ~0x0030;  // Clear OUTS bits
    config |= (stage << 4);

    return write_config_register(config);
}

esp_err_t WheelStateEstimator::setSlowFilter(as5600_slow_filter_t filter)
{
    uint16_t config;
    esp_err_t ret = read_config_register(&config);
    if (ret != ESP_OK) return ret;

    config &= ~0x0300;  // Clear SF bits
    config |= (filter << 8);

    return write_config_register(config);
}

esp_err_t WheelStateEstimator::setFastFilter(
  as5600_fast_filter_thresh_t threshold)
{
    uint16_t config;
    esp_err_t ret = read_config_register(&config);
    if (ret != ESP_OK) return ret;

    config &= ~0x1C00;  // Clear FTH bits
    config |= (threshold << 10);

    return write_config_register(config);
}

esp_err_t WheelStateEstimator::read_config_register(uint16_t * config)
{
    uint8_t buffer[2];
    esp_err_t ret = read_register(AS5600_REG_CONF_H, buffer, 2);
    if (ret == ESP_OK)
    {
        *config = (buffer[0] << 8) | buffer[1];
    }
    return ret;
}

esp_err_t WheelStateEstimator::read_register(uint8_t reg_addr, uint8_t * data,
                                             size_t len)
{
    return i2c_master_transmit_receive(m_i2c_dev_handle, &reg_addr, 1, data,
                                       len, pdMS_TO_TICKS(100));
}

esp_err_t WheelStateEstimator::write_config_register(uint16_t config)
{
    uint8_t buffer[2];
    buffer[0] = (config >> 8) & 0xFF;
    buffer[1] = config & 0xFF;
    return write_register(AS5600_REG_CONF_H, buffer, 2);
}

esp_err_t WheelStateEstimator::write_register(uint8_t reg_addr, uint8_t * data,
                                              size_t len)
{
    uint8_t write_buf[len + 1];
    write_buf[0] = reg_addr;
    memcpy(write_buf + 1, data, len);
    return i2c_master_transmit(m_i2c_dev_handle, write_buf, sizeof(write_buf),
                               pdMS_TO_TICKS(100));
}
