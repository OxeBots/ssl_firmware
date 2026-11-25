#ifndef _HEALTH_MONITOR_H_
#define _HEALTH_MONITOR_H_

#include <stdint.h>

#include "RuntimeTypes.h"

class HealthMonitor
{
   public:
    HealthState update(const HealthObservationFrame & observation);
    const HealthState & get_state() const;
    void clear_latched_faults();

   private:
    HealthState m_state;
};

#endif  // _HEALTH_MONITOR_H_
