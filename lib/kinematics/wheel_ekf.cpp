#include "wheel_ekf.h"

#include <esp_timer.h>

// Extended Kalman filter over the state [angle, velocity, acceleration].
//
// Every measurement is a single state row, so the update is written as
// sequential scalar corrections instead of the generic vt_kalman templates.
// The hand-written form avoids the temporary matrices that kept the ESP32
// acquisition task at full CPU.
static constexpr int STATE_DIM = 3;

// Measurement rows: 0 = angle, 1 = velocity.
static constexpr int MEAS_ANGLE = 0;
static constexpr int MEAS_VELOCITY = 1;

// Nominal update rate: one reading per published ADC block.
static constexpr double NOMINAL_UPDATE_HZ = 577.0;

// Process-noise densities per second, diagonal [angle, velocity,
// acceleration]. Keeping angle process noise small lets encoder angle
// innovations update velocity and acceleration directly instead of lagging
// behind fast speed steps.
static constexpr double Q_PROCESS[STATE_DIM] = {0.001, 50.0, 50000.0};

// Measurement noise variances. Bench standstill noise on the magnetic encoder
// is ~0.12 deg (~0.0007 rad on the wheel shaft); 0.005 rad^2 remains conservative
// against wrap jitter while cutting step velocity lag by ~4x.
static constexpr double R_MEAS_ANGLE = 0.005;
static constexpr double R_MEAS_VELOCITY = 0.4;

/**
 * @brief Initializes the covariance with the process noise of one nominal
 *        step, which is a small first guess for every state.
 */
WheelKalmanFilter::KalmanState::KalmanState()
{
    for (int i = 0; i < STATE_DIM; ++i) covariance[i][i] = Q_PROCESS[i] / NOMINAL_UPDATE_HZ;
}

/**
 * @brief Configures the wrap period of the sensed angle.
 * @param angle_period One full turn of the sensed shaft in radians.
 */
WheelKalmanFilter::WheelKalmanFilter(double angle_period) : m_angle_period(angle_period)
{
}

/**
 * @brief Maps an angle difference into [-period/2, period/2).
 * @param angle Angle difference in radians.
 * @return Equivalent difference across the shorter way around the turn.
 */
double WheelKalmanFilter::wrap_angle(double angle) const
{
    return angle - m_angle_period * std::floor((angle + 0.5 * m_angle_period) / m_angle_period);
}

/**
 * @brief Advances the state by dt and grows the covariance.
 *
 * x = [x0 + dt*x1 + 0.5*dt^2*x2, x1 + dt*x2, x2] and P = F P F^T + Q with
 * F = [[1, dt, 0.5*dt^2], [0, 1, dt], [0, 0, 1]]. F is upper triangular, so
 * both products expand into short row/column sums.
 */
void WheelKalmanFilter::predict(double dt)
{
    const double hdts = 0.5 * dt * dt;
    const double new_angle = m_state.state[0] + dt * m_state.state[1] + hdts * m_state.state[2];
    const double new_velocity = m_state.state[1] + dt * m_state.state[2];

    double fp[STATE_DIM][STATE_DIM];
    for (int j = 0; j < STATE_DIM; ++j)
    {
        fp[0][j] = m_state.covariance[0][j] + dt * m_state.covariance[1][j] +
                   hdts * m_state.covariance[2][j];
        fp[1][j] = m_state.covariance[1][j] + dt * m_state.covariance[2][j];
        fp[2][j] = m_state.covariance[2][j];
    }

    // (FP) * F^T: (FP*F^T)[i][j] = sum_k fp[i][k] * F[j][k], and the rows of
    // F are [1,dt,hdts] / [0,1,dt] / [0,0,1]. The angle-velocity cross terms
    // built here are what let the angle row correct the velocity state.
    for (int i = 0; i < STATE_DIM; ++i)
    {
        m_state.covariance[i][0] = fp[i][0] + dt * fp[i][1] + hdts * fp[i][2];
        m_state.covariance[i][1] = fp[i][1] + dt * fp[i][2];
        m_state.covariance[i][2] = fp[i][2];
        m_state.covariance[i][i] += Q_PROCESS[i] * dt;
    }

    m_state.state[0] = new_angle;
    m_state.state[1] = new_velocity;
}

/**
 * @brief Applies one scalar measurement row of the observation matrix.
 *
 * h(x) = state[row], so with H = e_row^T: S = P[row][row] + variance,
 * K = P[:,row] / S, x += K y and P -= K (row of P).
 */
void WheelKalmanFilter::scalar_update(int row, double innovation, double variance)
{
    const double s = m_state.covariance[row][row] + variance;
    const double k[STATE_DIM] = {m_state.covariance[0][row] / s,
                                 m_state.covariance[1][row] / s,
                                 m_state.covariance[2][row] / s};
    // The old measurement row is kept apart: the loop below overwrites it
    // while later iterations still need its original values.
    const double measured_row[STATE_DIM] = {
      m_state.covariance[row][0], m_state.covariance[row][1], m_state.covariance[row][2]};

    for (int i = 0; i < STATE_DIM; ++i) m_state.state[i] += k[i] * innovation;

    for (int i = 0; i < STATE_DIM; ++i)
        for (int j = 0; j < STATE_DIM; ++j) m_state.covariance[i][j] -= k[i] * measured_row[j];
}

/**
 * @brief Predicts to the current time and reports whether a previous estimate
 *        was already available.
 */
bool WheelKalmanFilter::prepare_step()
{
    const int64_t now = esp_timer_get_time();

    if (!m_state.initialized)
    {
        m_state.last_update_us = now;
        m_state.initialized = true;
        return false;
    }

    const double dt = static_cast<double>(now - m_state.last_update_us) / 1e6;
    predict(dt);
    m_state.last_update_us = now;
    return true;
}

/**
 * @brief Fuses one encoder angle.
 */
void WheelKalmanFilter::update(double measured_angle)
{
    if (!prepare_step())
    {
        m_state.state[MEAS_ANGLE] = measured_angle;
        return;
    }

    const double innovation = wrap_angle(measured_angle - m_state.state[MEAS_ANGLE]);
    scalar_update(MEAS_ANGLE, innovation, R_MEAS_ANGLE);
}

/**
 * @brief Fuses one encoder angle and one direct velocity in one cycle.
 */
void WheelKalmanFilter::update(double measured_angle, double measured_velocity)
{
    if (!prepare_step())
    {
        m_state.state[MEAS_ANGLE] = measured_angle;
        m_state.state[MEAS_VELOCITY] = measured_velocity;
        return;
    }

    const double angle_innovation = wrap_angle(measured_angle - m_state.state[MEAS_ANGLE]);
    scalar_update(MEAS_ANGLE, angle_innovation, R_MEAS_ANGLE);

    const double velocity_innovation = measured_velocity - m_state.state[MEAS_VELOCITY];
    scalar_update(MEAS_VELOCITY, velocity_innovation, R_MEAS_VELOCITY);
}

/**
 * @brief Drops the current estimate and its timestamps.
 */
void WheelKalmanFilter::reset()
{
    m_state = KalmanState();
}

double WheelKalmanFilter::get_angle_rad() const
{
    return m_state.state[MEAS_ANGLE];
}

double WheelKalmanFilter::get_velocity_rad_s() const
{
    return m_state.state[MEAS_VELOCITY];
}
