#include "RobotStateEstimator.h"

#include <cmath>

static float wrap_angle(float angle_rad)
{
    constexpr float TWO_PI_VALUE = 6.28318530718f;
    while (angle_rad > 3.14159265359f) angle_rad -= TWO_PI_VALUE;
    while (angle_rad < -3.14159265359f) angle_rad += TWO_PI_VALUE;
    return angle_rad;
}

static float clamp_dt(float dt_s)
{
    if (dt_s < 0.0f)
        return 0.0f;
    return dt_s > 0.2f ? 0.2f : dt_s;
}

RobotStateEstimator::RobotStateEstimator(const OmnidirectionalRobot & kinematics)
: m_kinematics(kinematics)
{
}

void RobotStateEstimator::reset()
{
    m_latest = {};
    m_last_timestamp_us = 0;
    m_has_timestamp = false;
}

/**
 * @brief Fuse wheel odometry with an optional absolute pose and IMU heading.
 *
 * Wheel velocities are transformed into body velocity by the existing omni
 * kinematics. A valid base-station pose is an absolute correction; otherwise
 * odometry is integrated and the IMU heading is used when available.
 */
RobotStateEstimate RobotStateEstimator::update(const FusionInputFrame & input)
{
    const uint64_t timestamp_us =
      input.timestamp_us != 0 ? input.timestamp_us : input.wheels.timestamp_us;
    float dt_s = 0.0f;
    if (m_has_timestamp && timestamp_us >= m_last_timestamp_us)
        dt_s = clamp_dt(static_cast<float>(timestamp_us - m_last_timestamp_us) / 1000000.0f);

    const bool wheels_valid = (input.wheels.valid_mask & 0x0FU) == 0x0FU;
    BodyVelocity body_velocity = {};
    if (wheels_valid)
    {
        vt::numeric_vector<4> wheel_velocity;
        for (size_t index = 0; index < ROBOT_WHEEL_COUNT; ++index)
            wheel_velocity(index) = input.wheels.velocity_rad_s[index];

        const vt::numeric_vector<3> body = m_kinematics.compute_body_velocities(wheel_velocity);
        body_velocity.omega_rad_s = static_cast<float>(body(0));
        body_velocity.vx_m_s = static_cast<float>(body(1));
        body_velocity.vy_m_s = static_cast<float>(body(2));
    }

    if (input.base_station_pose.valid)
    {
        m_latest.pose = input.base_station_pose.pose;
        m_latest.field_velocity = input.base_station_pose.velocity;
    }
    else
    {
        if (input.imu.valid)
            m_latest.pose.heading_rad = wrap_angle(input.imu.orientation.heading_rad);

        const float cos_heading = std::cos(m_latest.pose.heading_rad);
        const float sin_heading = std::sin(m_latest.pose.heading_rad);
        const float field_vx =
          cos_heading * body_velocity.vx_m_s - sin_heading * body_velocity.vy_m_s;
        const float field_vy =
          sin_heading * body_velocity.vx_m_s + cos_heading * body_velocity.vy_m_s;
        m_latest.pose.x_m += field_vx * dt_s;
        m_latest.pose.y_m += field_vy * dt_s;
        m_latest.pose.heading_rad =
          wrap_angle(m_latest.pose.heading_rad + body_velocity.omega_rad_s * dt_s);
        m_latest.field_velocity = {field_vx, field_vy, body_velocity.omega_rad_s};
    }

    m_latest.body_velocity = body_velocity;
    m_latest.perception = input.perception;
    m_latest.health = input.health;
    m_latest.valid = wheels_valid || input.imu.valid || input.base_station_pose.valid;
    m_latest.timestamp_us = timestamp_us;

    if (timestamp_us != 0)
    {
        m_last_timestamp_us = timestamp_us;
        m_has_timestamp = true;
    }
    return m_latest;
}

const RobotStateEstimate & RobotStateEstimator::get_latest() const
{
    return m_latest;
}
