#include "PowerMonitor.h"

#include <cmath>

static bool is_voltage_valid(float voltage_v)
{
    return std::isfinite(voltage_v) && voltage_v >= 0.0f;
}

PowerMonitor::PowerMonitor(const PowerMonitorConfig & config) : m_config(config)
{
}

/**
 * @brief Convert a measured ADC pin voltage to supply voltage.
 * @return true when the sample is finite and within the configured input range.
 */
bool PowerMonitor::update_adc_voltage(float adc_voltage_v, uint64_t timestamp_us)
{
    if (!is_voltage_valid(adc_voltage_v) || !is_voltage_valid(m_config.adc_to_supply_ratio) ||
        m_config.adc_to_supply_ratio <= 0.0f)
        return false;

    return update_supply_voltage(adc_voltage_v * m_config.adc_to_supply_ratio, timestamp_us);
}

/**
 * @brief Store a supply-voltage sample for health publication.
 */
bool PowerMonitor::update_supply_voltage(float supply_voltage_v, uint64_t timestamp_us)
{
    if (!is_voltage_valid(supply_voltage_v))
    {
        m_valid = false;
        return false;
    }

    m_supply_voltage_v = supply_voltage_v;
    m_timestamp_us = timestamp_us;
    m_valid = true;
    return true;
}

/**
 * @brief Copy the current voltage status into a health observation frame.
 */
void PowerMonitor::fill_observation(HealthObservationFrame * observation) const
{
    if (observation == nullptr)
        return;

    observation->supply_valid = m_valid;
    observation->supply_undervoltage = is_undervoltage();
    observation->supply_overvoltage = is_overvoltage();
    observation->timestamp_us = m_timestamp_us;
}

float PowerMonitor::get_supply_voltage_v() const
{
    return m_supply_voltage_v;
}

bool PowerMonitor::is_undervoltage() const
{
    return m_valid && m_supply_voltage_v < m_config.undervoltage_v;
}

bool PowerMonitor::is_overvoltage() const
{
    return m_valid && m_supply_voltage_v > m_config.overvoltage_v;
}

bool PowerMonitor::is_valid() const
{
    return m_valid;
}
