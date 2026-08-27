#ifndef HEALTH_MONITOR_H
#define HEALTH_MONITOR_H

#include <stdint.h>

#include "RuntimeTypes.h"

class HealthMonitor
{
   public:
    static constexpr uint16_t INHIBIT_MOTION = 1U << 0;
    static constexpr uint16_t INHIBIT_KICK = 1U << 1;

    HealthState update(const HealthObservationFrame & observation);
    const HealthState & get_state() const;
    void clear_latched_faults();

   private:
    HealthState m_state;
};

#endif  // HEALTH_MONITOR_H
