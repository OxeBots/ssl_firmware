#include "HealthMonitor.h"

#include <sdkconfig.h>

#include "ssl_robot_protocol_bp.h"

static uint16_t motor_fault_flag(size_t motor)
{
    return static_cast<uint16_t>(ERROR_MOTOR0_FAULT << motor);
}

/**
 * @brief Classify sensor, wheel, motor, supply and kicker observations.
 * Latch electrical faults, motor faults and loss of all wheel feedback.
 */
HealthState HealthMonitor::update(const HealthObservationFrame & observation)
{
    uint16_t active_faults = 0;
    uint16_t latched_faults = 0;

    if (!observation.imu_available)
        active_faults |= ERROR_IMU_FAIL;
    if (!observation.tof_available)
        active_faults |= ERROR_TOF_FAIL;
    constexpr uint8_t ALL_WHEELS = (1U << ROBOT_WHEEL_COUNT) - 1U;
    const uint8_t valid_wheels = observation.wheel_valid_mask & ALL_WHEELS;
    if (valid_wheels != ALL_WHEELS)
        active_faults |= ERROR_ENCODER_INVALID;
    if (valid_wheels == 0U)
        latched_faults |= ERROR_ENCODER_INVALID;

    for (size_t motor = 0; motor < ROBOT_WHEEL_COUNT; ++motor)
    {
        const uint8_t bit = static_cast<uint8_t>(1U << motor);
        if ((observation.motor_fault_mask & bit) != 0U ||
            (observation.wheel_stalled_mask & bit) != 0U)
            latched_faults |= motor_fault_flag(motor);
    }

    if (observation.supply_undervoltage)
        latched_faults |= ERROR_BATT_UNDERVOLT;
    if (observation.supply_overvoltage)
        latched_faults |= ERROR_BATT_OVERVOLT;
    if (!observation.kicker_ready || observation.kicker_fault)
        active_faults |= ERROR_KICKER_NO_CHARGE;
    if (observation.temperature_warning)
        active_faults |= ERROR_TEMPERATURE_WARN;

    m_state.active_faults = active_faults | latched_faults;
    m_state.latched_faults |= latched_faults;
    m_state.severity =
      m_state.active_faults != 0U ? HealthSeverity::WARNING : HealthSeverity::HEALTHY;
    m_state.kick_inhibited = !observation.kicker_ready || observation.kicker_fault;
#if !CONFIG_DISABLE_SAFE_STOP
    if (latched_faults != 0U)
        m_state.severity = HealthSeverity::CRITICAL;
    m_state.kick_inhibited |= !observation.tof_available;
#endif
    m_state.timestamp_us = observation.timestamp_us;

    return m_state;
}

const HealthState & HealthMonitor::get_state() const
{
    return m_state;
}

/**
 * @brief Clear latched faults after the caller has verified a safe recovery.
 */
void HealthMonitor::clear_latched_faults()
{
    m_state.latched_faults = 0;
}
