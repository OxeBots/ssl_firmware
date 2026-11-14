#include <driver/spi_master.h>
#include <sdkconfig.h>
#include <unity.h>

#include "../shared_test_bus.h"
#include "74HC595.h"
#include "SPIdev.h"

namespace
{
constexpr uint8_t TEST_DEVICE_KEY = 0xFD;
}

void setup_spi()
{
    shared_test_spi_bus();
}

void test_spi_device_registration_and_write()
{
    spi_device_interface_config_t device_config = {};
    device_config.clock_speed_hz = 1000000;
    device_config.mode = 0;
    device_config.spics_io_num = -1;
    device_config.queue_size = 1;
    device_config.flags = SPI_DEVICE_NO_DUMMY;

    TEST_ASSERT_TRUE(SPIdev::register_device(TEST_DEVICE_KEY, GPIO_NUM_NC, &device_config));

    const uint8_t value = 0xA5;
    TEST_ASSERT_TRUE(SPIdev::write(TEST_DEVICE_KEY, &value, sizeof(value)));
}

void test_spi_device_read_and_shift()
{
    uint8_t received = 0;
    const uint8_t command = 0;

    TEST_ASSERT_TRUE(SPIdev::read(TEST_DEVICE_KEY, &command, &received, sizeof(received)));
    (void)SPIdev::shift(TEST_DEVICE_KEY, 0x5A);
}

void test_invalid_transfers_are_rejected()
{
    uint8_t value = 0;

    TEST_ASSERT_FALSE(SPIdev::write(TEST_DEVICE_KEY, nullptr, sizeof(value)));
    TEST_ASSERT_FALSE(SPIdev::read(TEST_DEVICE_KEY, nullptr, nullptr, sizeof(value)));
    TEST_ASSERT_FALSE(SPIdev::read(TEST_DEVICE_KEY, nullptr, &value, 0));
}

extern "C" void app_main(void)
{
    setup_spi();

    UNITY_BEGIN();
    RUN_TEST(test_spi_device_registration_and_write);
    RUN_TEST(test_spi_device_read_and_shift);
    RUN_TEST(test_invalid_transfers_are_rejected);
    UNITY_END();
}
