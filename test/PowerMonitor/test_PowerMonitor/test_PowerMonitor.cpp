#include <unity.h>

#include "PowerMonitor.h"

void test_power_monitor_converts_and_reports_limits()
{
    PowerMonitorConfig config = {};
    config.adc_to_supply_ratio = 10.0f;
    config.undervoltage_v = 11.0f;
    config.overvoltage_v = 13.0f;
    PowerMonitor monitor(config);
    HealthObservationFrame observation = {};

    TEST_ASSERT_TRUE(monitor.update_adc_voltage(1.2f, 42));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.0f, monitor.get_supply_voltage_v());
    TEST_ASSERT_FALSE(monitor.is_undervoltage());
    TEST_ASSERT_FALSE(monitor.is_overvoltage());
    monitor.fill_observation(&observation);
    TEST_ASSERT_TRUE(observation.supply_valid);
    TEST_ASSERT_TRUE(observation.timestamp_us == 42);

    TEST_ASSERT_TRUE(monitor.update_supply_voltage(10.0f, 43));
    TEST_ASSERT_TRUE(monitor.is_undervoltage());
    monitor.fill_observation(&observation);
    TEST_ASSERT_TRUE(observation.supply_undervoltage);
}

// The singleton is wired to the VMON Kconfig divider: a 24 V battery across
// the 56k/2k2 divider gives about 0.907 V at the pin, and the conversion
// must report that pin voltage back as about 24 V.
void test_power_monitor_singleton_applies_divider_ratio(void)
{
    PowerMonitor & monitor = PowerMonitor::get_instance();
    TEST_ASSERT_TRUE(monitor.update_adc_voltage(0.90725f, 100));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 24.0f, monitor.get_supply_voltage_v());
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_power_monitor_converts_and_reports_limits);
    RUN_TEST(test_power_monitor_singleton_applies_divider_ratio);
    UNITY_END();
}

extern "C" void app_main(void)
{
    setup();
}
