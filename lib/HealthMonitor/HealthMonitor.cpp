#include "HealthMonitor.h"

#include "ssl_robot_protocol_bp.h"

static uint16_t motor_fault_flag(size_t motor)
{
    return static_cast<uint16_t>(ERROR_MOTOR0_FAULT << motor);
}

/**
 * @brief Classify sensor, wheel, motor, supply and kicker observations.
 *
 * Missing optional perception is recoverable and inhibits kicking. Electrical
 * faults, motor faults and loss of all wheel feedback inhibit motion and latch.
 */
HealthState HealthMonitor::update(const HealthObservationFrame & observation)
{
    uint16_t active_faults = 0;
    uint16_t critical_faults = 0;

    if (!observation.imu_available)
        active_faults |= ERROR_IMU_FAIL;
    if (!observation.tof_available)
        active_faults |= ERROR_TOF_FAIL;
    if ((observation.wheel_valid_mask & 0x0FU) != 0x0FU)
        active_faults |= ERROR_ENCODER_INVALID;

    for (size_t motor = 0; motor < ROBOT_WHEEL_COUNT; ++motor)
    {
        const uint8_t bit = static_cast<uint8_t>(1U << motor);
        if ((observation.motor_fault_mask & bit) != 0U ||
            (observation.wheel_stalled_mask & bit) != 0U)
        {
            active_faults |= motor_fault_flag(motor);
            critical_faults |= motor_fault_flag(motor);
        }
    }

    if ((observation.wheel_valid_mask & 0x0FU) == 0U)
    {
        active_faults |= ERROR_ENCODER_INVALID;
        critical_faults |= ERROR_ENCODER_INVALID;
    }
    if (observation.supply_undervoltage)
    {
        active_faults |= ERROR_BATT_UNDERVOLT;
        critical_faults |= ERROR_BATT_UNDERVOLT;
    }
    if (observation.supply_overvoltage)
    {
        active_faults |= ERROR_BATT_OVERVOLT;
        critical_faults |= ERROR_BATT_OVERVOLT;
    }
    if (!observation.kicker_ready || observation.kicker_fault)
        active_faults |= ERROR_KICKER_NO_CHARGE;
    if (observation.temperature_warning)
        active_faults |= ERROR_TEMPERATURE_WARN;

    m_state.active_faults = active_faults;
    m_state.timestamp_us = observation.timestamp_us;
<<<<<<< Updated upstream
    m_state.motion_inhibited = critical_faults != 0U;
    // armengue: estou desativando o !observation.tof_available (ativar quando terminar a competição e o tof estiver ativo)
=======
    // TEMP (testes): não inibir movimento por falhas críticas de health
    // (mantém active_faults/latched_faults visíveis para diagnóstico).
    m_state.motion_inhibited = false;
>>>>>>> Stashed changes
    m_state.kick_inhibited =
      !observation.kicker_ready || observation.kicker_fault;
    m_state.inhibit_reasons = 0;
    if (m_state.motion_inhibited)
        m_state.inhibit_reasons |= INHIBIT_MOTION;
    if (m_state.kick_inhibited)
        m_state.inhibit_reasons |= INHIBIT_KICK;

    if (critical_faults != 0U)
    {
        m_state.severity = HealthSeverity::CRITICAL;
        m_state.latched_faults |= critical_faults;
    }
    else if (active_faults != 0U)
    {
        m_state.severity = HealthSeverity::RECOVERABLE;
    }
    else
    {
        m_state.severity = HealthSeverity::HEALTHY;
    }

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
