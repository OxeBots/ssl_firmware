#include "wheel_ekf.h"

#include <vt_kalman>
#include <vt_linalg>

static constexpr int STATE_DIM = 3;
static constexpr int MEAS_DIM = 1;
static constexpr int CONTROL_DIM = 1;

using EKF = vt::extended_kalman_filter_t<STATE_DIM, MEAS_DIM, CONTROL_DIM>;
using StateVector = vt::numeric_vector<STATE_DIM>;
using MeasurementVector = vt::numeric_vector<MEAS_DIM>;
using ControlVector = vt::numeric_vector<CONTROL_DIM>;

// Noise Matrices
// Process Noise: Uncertainty in the model (tune these values)
// Diagonal: [angle_noise, velocity_noise, acceleration_noise]
static vt::numeric_matrix<STATE_DIM, STATE_DIM> Q =
  vt::numeric_matrix<STATE_DIM, STATE_DIM>::diagonals({0.001, 0.1, 10.0});

// Measurement Noise: Uncertainty in the sensor reading (tune this value)
static vt::numeric_matrix<MEAS_DIM, MEAS_DIM> R =
  vt::numeric_matrix<MEAS_DIM, MEAS_DIM>::diagonals(0.1);

struct WheelKalmanFilter::KalmanState
{
    StateVector state_vec;
    EKF filter;
    int64_t last_update_us = 0;
    bool initialized = false;

    // The state vector is owned by the filter object itself
    KalmanState()
    : state_vec(vt::make_numeric_vector({0.0, 0.0, 0.0})),
      filter(f_func, Fj_func, h_func, Hj_func, Q, R, state_vec)
    {
    }

    // EKF Model Functions (static members)

    // State transition function f(x, u)
    static StateVector f_func(const StateVector & x, const ControlVector & u)
    {
        double dt = u[0];
        double hdts = 0.5f * dt * dt;
        // x_new = x + v*dt + 0.5*a*dt^2
        // v_new = v + a*dt
        // a_new = a
        return vt::make_numeric_vector({x[0] + dt * x[1] + hdts * x[2], x[1] + dt * x[2], x[2]});
    }

    // Jacobian of f, Fj(x, u)
    static vt::numeric_matrix<3, 3> Fj_func(const StateVector &, const ControlVector & u)
    {
        double dt = u[0];
        double hdts = 0.5f * dt * dt;
        return vt::make_numeric_matrix<3, 3>({{1, dt, hdts}, {0, 1, dt}, {0, 0, 1}});
    }

    // Measurement function h(x)
    static MeasurementVector h_func(const StateVector & x)
    {
        return vt::make_numeric_vector({x[0]});
    }

    // Jacobian of h, Hj(x)
    static vt::numeric_matrix<1, 3> Hj_func(const StateVector &)
    {
        return vt::make_numeric_matrix<1, 3>({{1, 0, 0}});
    }
};

WheelKalmanFilter::WheelKalmanFilter() : m_state(std::make_unique<KalmanState>())
{
}

WheelKalmanFilter::~WheelKalmanFilter() = default;

/**
 * @brief Updates the filter with a new angle measurement.
 * @param measured_angle The new angle in radians.
 */
void WheelKalmanFilter::update(double measured_angle_rad)
{
    int64_t now = esp_timer_get_time();

    if (!m_state->initialized)
    {
        m_state->state_vec = vt::make_numeric_vector({measured_angle_rad, 0.0, 0.0});
        m_state->last_update_us = now;
        m_state->initialized = true;
        return;
    }

    // --- Predict Step ---
    double dt = (now - m_state->last_update_us) / 1e6f;
    m_state->filter.predict(vt::make_numeric_vector({dt}));
    m_state->last_update_us = now;

    // --- Update Step ---
    // Handle angle wrapping (innovation)
    const auto & predicted_state = m_state->filter.state_vector;
    double predicted_angle = predicted_state[0];
    double innovation = normalize_angle(measured_angle_rad - predicted_angle);
    double corrected_measurement = predicted_angle + innovation;
    m_state->filter.update(vt::make_numeric_vector({corrected_measurement}));
}

/**
 * @brief Retrieves the filtered angle.
 * @return Angle in radians.
 */
double WheelKalmanFilter::get_angle_rad() const
{
    return m_state->filter.state_vector[0];
}

/**
 * @brief Retrieves the filtered angular velocity.
 * @return Velocity in rad/s.
 */
double WheelKalmanFilter::get_velocity_rad_s() const
{
    return m_state->filter.state_vector[1];
}

/**
 * @brief Retrieves the filtered angular acceleration.
 * @return Acceleration in rad/s^2.
 */
double WheelKalmanFilter::get_acceleration_rad_s2() const
{
    return m_state->filter.state_vector[2];
}
