#ifndef _POWER_MONITOR_H_
#define _POWER_MONITOR_H_

#include <stdint.h>

#include "RuntimeTypes.h"

struct PowerMonitorConfig
{
    float adc_to_supply_ratio = 1.0f;
    float undervoltage_v = 0.0f;
    float overvoltage_v = 1000.0f;
};

class PowerMonitor
{
   public:
    // Process-wide monitor configured from the VMON sdkconfig values.
    static PowerMonitor & get_instance();

    explicit PowerMonitor(const PowerMonitorConfig & config = PowerMonitorConfig());

    bool update_adc_voltage(float adc_voltage_v, uint64_t timestamp_us);
    bool update_supply_voltage(float supply_voltage_v, uint64_t timestamp_us);
    void fill_observation(HealthObservationFrame * observation) const;

    float get_supply_voltage_v() const;
    bool is_undervoltage() const;
    bool is_overvoltage() const;
    bool is_valid() const;

   private:
    PowerMonitorConfig m_config;
    float m_supply_voltage_v = 0.0f;
    uint64_t m_timestamp_us = 0;
    bool m_valid = false;
};

#endif  // _POWER_MONITOR_H_
