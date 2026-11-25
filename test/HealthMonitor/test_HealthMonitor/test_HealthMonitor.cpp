#include <sdkconfig.h>
#include <unity.h>

#include "HealthMonitor.h"

void test_health_monitor_reports_sensor_and_drive_warnings()
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
    TEST_ASSERT_EQUAL_INT(static_cast<int>(HealthSeverity::WARNING),
                          static_cast<int>(health.severity));
    TEST_ASSERT_EQUAL_UINT16(ERROR_TOF_FAIL, health.active_faults);
#if CONFIG_DISABLE_SAFE_STOP
    TEST_ASSERT_FALSE(health.kick_inhibited);
#else
    TEST_ASSERT_TRUE(health.kick_inhibited);
#endif

    observation.wheel_valid_mask = 0;
    observation.supply_undervoltage = true;
    observation.supply_overvoltage = true;
    observation.motor_fault_mask = 1;
    health = monitor.update(observation);
#if CONFIG_DISABLE_SAFE_STOP
    const HealthSeverity drive_fault_severity = HealthSeverity::WARNING;
#else
    const HealthSeverity drive_fault_severity = HealthSeverity::CRITICAL;
#endif
    TEST_ASSERT_EQUAL_INT(static_cast<int>(drive_fault_severity),
                          static_cast<int>(health.severity));
    constexpr uint16_t DRIVE_WARNINGS =
      ERROR_ENCODER_INVALID | ERROR_BATT_UNDERVOLT | ERROR_BATT_OVERVOLT | ERROR_MOTOR0_FAULT;
    TEST_ASSERT_EQUAL_UINT16(DRIVE_WARNINGS | ERROR_TOF_FAIL, health.active_faults);
    TEST_ASSERT_EQUAL_UINT16(DRIVE_WARNINGS, health.latched_faults);
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_health_monitor_reports_sensor_and_drive_warnings);
    UNITY_END();
}

extern "C" void app_main(void)
{
    setup();
}
