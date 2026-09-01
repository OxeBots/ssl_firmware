#include <unity.h>

#include "HealthMonitor.h"

void test_health_monitor_separates_recoverable_and_critical_faults()
{
    HealthMonitor monitor;
    HealthObservationFrame observation = {};
    observation.wheel_valid_mask = 0x0F;
    observation.imu_available = true;
    observation.tof_available = true;
    observation.kicker_ready = true;
    observation.timestamp_us = 10;

    HealthState health = monitor.update(observation);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(HealthSeverity::HEALTHY),
                          static_cast<int>(health.severity));
    TEST_ASSERT_EQUAL_UINT16(0, health.active_faults);
    TEST_ASSERT_FALSE(health.kick_inhibited);

    observation.tof_available = false;
    health = monitor.update(observation);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(HealthSeverity::RECOVERABLE),
                          static_cast<int>(health.severity));
    TEST_ASSERT_TRUE((health.active_faults & ERROR_TOF_FAIL) != 0);
    TEST_ASSERT_FALSE(health.motion_inhibited);
    TEST_ASSERT_TRUE(health.kick_inhibited);

    observation.wheel_valid_mask = 0;
    health = monitor.update(observation);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(HealthSeverity::CRITICAL),
                          static_cast<int>(health.severity));
    TEST_ASSERT_TRUE(health.motion_inhibited);
    TEST_ASSERT_TRUE((health.latched_faults & ERROR_ENCODER_INVALID) != 0);
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_health_monitor_separates_recoverable_and_critical_faults);
    UNITY_END();
}

extern "C" void app_main(void)
{
    setup();
}
