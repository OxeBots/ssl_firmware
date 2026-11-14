#ifndef _RUNTIME_TYPES_H_
#define _RUNTIME_TYPES_H_

#include <stddef.h>
#include <stdint.h>

#include <array>

#include "ssl_robot_protocol_bp.h"

static constexpr size_t ROBOT_WHEEL_COUNT = 4;
static constexpr size_t TOF_ZONE_COUNT = 64;

struct Pose2D
{
    float x_m = 0.0f;
    float y_m = 0.0f;
    float heading_rad = 0.0f;
};

struct BodyVelocity
{
    float vx_m_s = 0.0f;
    float vy_m_s = 0.0f;
    float omega_rad_s = 0.0f;
};

struct WheelStateEstimate
{
    std::array<float, ROBOT_WHEEL_COUNT> angle_rad = {};
    std::array<float, ROBOT_WHEEL_COUNT> velocity_rad_s = {};
    std::array<float, ROBOT_WHEEL_COUNT> acceleration_rad_s2 = {};
    uint8_t valid_mask = 0;
    uint64_t timestamp_us = 0;
};

struct ImuState
{
    std::array<float, 3> acceleration_m_s2 = {};
    std::array<float, 3> velocity_m_s = {};
    Pose2D orientation;
    bool valid = false;
    uint64_t timestamp_us = 0;
};

struct LocalPerceptionState
{
    std::array<int16_t, TOF_ZONE_COUNT> range_mm = {};
    std::array<uint8_t, TOF_ZONE_COUNT> target_status = {};
    uint8_t resolution = 0;
    int16_t nearest_distance_mm = -1;
    uint8_t nearest_zone = 0;
    bool obstacle_close = false;
    bool valid = false;
    uint64_t timestamp_us = 0;
};

struct BaseStationPose
{
    Pose2D pose;
    BodyVelocity velocity;
    bool valid = false;
    uint64_t timestamp_us = 0;
};

enum class HealthSeverity : uint8_t
{
    HEALTHY = 0,
    RECOVERABLE = 1,
    CRITICAL = 2,
};

struct HealthState
{
    HealthSeverity severity = HealthSeverity::HEALTHY;
    uint16_t active_faults = 0;
    uint16_t latched_faults = 0;
    uint16_t inhibit_reasons = 0;
    bool motion_inhibited = false;
    bool kick_inhibited = true;
    uint64_t timestamp_us = 0;
};

struct FusionInputFrame
{
    WheelStateEstimate wheels;
    ImuState imu;
    LocalPerceptionState perception;
    BaseStationPose base_station_pose;
    HealthState health;
    uint64_t timestamp_us = 0;
};

struct RobotStateEstimate
{
    Pose2D pose;
    BodyVelocity body_velocity;
    BodyVelocity field_velocity;
    LocalPerceptionState perception;
    HealthState health;
    bool valid = false;
    uint64_t timestamp_us = 0;
};

struct GoalPose
{
    Pose2D pose;
    BodyVelocity velocity;
    uint8_t kick_velocity = 0;
    bool motion_inhibited = false;
    bool kick_inhibited = true;
    uint32_t command_sequence = 0;
    uint64_t received_at_us = 0;
};

struct ActuationSetpoint
{
    std::array<float, ROBOT_WHEEL_COUNT> wheel_velocity_rad_s = {};
    uint8_t kick_velocity = 0;
    bool motion_inhibited = true;
    bool kick_inhibited = true;
    uint32_t command_sequence = 0;
    uint64_t timestamp_us = 0;
};

struct HealthObservationFrame
{
    uint8_t wheel_valid_mask = 0;
    uint8_t wheel_stalled_mask = 0;
    uint8_t motor_fault_mask = 0;
    bool imu_available = false;
    bool tof_available = false;
    bool supply_valid = false;
    bool supply_undervoltage = false;
    bool supply_overvoltage = false;
    bool kicker_ready = false;
    bool kicker_fault = false;
    bool temperature_warning = false;
    uint64_t timestamp_us = 0;
};

struct CommandEnvelope
{
    RobotCommand command = {};
    uint64_t received_at_us = 0;
    uint32_t sequence = 0;
};

#endif  // _RUNTIME_TYPES_H_
