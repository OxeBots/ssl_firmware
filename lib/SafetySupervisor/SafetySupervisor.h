#ifndef _SAFETY_SUPERVISOR_H_
#define _SAFETY_SUPERVISOR_H_

#include <stdint.h>

#include "RuntimeTypes.h"

enum class LifecycleState : uint8_t
{
    BOOT = 0,
    SELF_TEST = 1,
    DISARMED = 2,
    ACTIVE = 3,
    DEGRADED = 4,
    SAFE_STOP = 5,
    CALIBRATION = 6,
};

struct SafetySupervisorConfig
{
    uint64_t command_timeout_us = 250000;
    int16_t obstacle_stop_distance_mm = 150;
};

struct SafetyDecision
{
    LifecycleState lifecycle = LifecycleState::BOOT;
    GoalPose goal;
    bool command_fresh = false;
};

class SafetySupervisor
{
   public:
    explicit SafetySupervisor(const SafetySupervisorConfig & config = SafetySupervisorConfig());

    void mark_self_test_complete(bool complete);
    void set_calibration_active(bool active);
    static bool health_inhibits_motion(const HealthState & health);

    SafetyDecision update(uint64_t now_us,
                          const CommandEnvelope * command,
                          const HealthState & health,
                          const LocalPerceptionState & perception);
    LifecycleState get_state() const;

   private:
    bool is_command_fresh(uint64_t now_us, const CommandEnvelope * command) const;
    static GoalPose make_safe_goal();
    GoalPose make_goal(const CommandEnvelope & envelope, const HealthState & health) const;

    SafetySupervisorConfig m_config;
    LifecycleState m_state = LifecycleState::BOOT;
    bool m_self_test_complete = false;
    bool m_calibration_active = false;
};

using LifecycleSupervisor = SafetySupervisor;

#endif  // _SAFETY_SUPERVISOR_H_
