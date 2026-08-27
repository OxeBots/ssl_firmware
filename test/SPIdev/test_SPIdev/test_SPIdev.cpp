#include <74HC595.h>
#include <SPIdev.h>
#include <driver/spi_master.h>
#include <sdkconfig.h>
#include <unity.h>

namespace
{
constexpr uint8_t TEST_DEVICE_KEY = 0xFD;
}

void setup_spi()
{
    spi_bus_config_t bus_config = {};
    bus_config.mosi_io_num = (gpio_num_t)CONFIG_MOSI_GPIO;
    bus_config.miso_io_num = (gpio_num_t)CONFIG_MISO_GPIO;
    bus_config.sclk_io_num = (gpio_num_t)CONFIG_SCLK_GPIO;
    bus_config.quadwp_io_num = -1;
    bus_config.quadhd_io_num = -1;
    TEST_ASSERT_EQUAL(ESP_OK, spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO));
    SPIdev::init(SPI2_HOST);
}

void test_spi_device_registration_and_write()
{
    spi_device_interface_config_t device_config = {};
    device_config.clock_speed_hz = 1000000;
    device_config.mode = 0;
    device_config.spics_io_num = -1;
    device_config.queue_size = 1;
    device_config.flags = SPI_DEVICE_NO_DUMMY;

    TEST_ASSERT_TRUE(SPIdev::register_device(TEST_DEVICE_KEY, &device_config));

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
