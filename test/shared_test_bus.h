// Shared test-rig buses for suites that run back-to-back in the merged test
// binary (test/all). Every hardware suite wires the same I2C port and the same
// SPI2 pins, so the buses are created once and reused instead of claiming the
// port again per suite. Standalone suites behave identically: the first call
// creates what a fresh boot would have created anyway.

#ifndef _SHARED_TEST_BUS_H_
#define _SHARED_TEST_BUS_H_

#include <driver/i2c_master.h>
#include <driver/spi_master.h>
#include <esp_err.h>
#include <sdkconfig.h>

#include "SPIdev.h"

// Single storage point so the helpers below and shared_test_i2c_adopt()
// operate on the same handle.
inline i2c_master_bus_handle_t & shared_test_i2c_storage()
{
    static i2c_master_bus_handle_t handle = nullptr;
    return handle;
}

// Returns the shared I2C master bus, creating it on first use.
inline i2c_master_bus_handle_t shared_test_i2c_bus()
{
    i2c_master_bus_handle_t & handle = shared_test_i2c_storage();
    if (handle == nullptr)
    {
        i2c_master_bus_config_t config = {};
        config.i2c_port = (i2c_port_t)CONFIG_I2C_PORT_NUM;
        config.sda_io_num = (gpio_num_t)CONFIG_SDA_GPIO;
        config.scl_io_num = (gpio_num_t)CONFIG_SCL_GPIO;
        config.clk_source = I2C_CLK_SRC_DEFAULT;
        config.glitch_ignore_cnt = 7;
        config.intr_priority = 0;
        config.trans_queue_depth = 0;
        config.flags = {.enable_internal_pullup = true, .allow_pd = false};
        esp_err_t err = i2c_new_master_bus(&config, &handle);
        if (err != ESP_OK)
        {
            handle = nullptr;
            ESP_ERROR_CHECK(err);
        }
    }
    return handle;
}

// Lets a suite hand over a bus it created itself (see the IMUGY85 suite,
// whose driver creates the bus inside its own init).
inline void shared_test_i2c_adopt(i2c_master_bus_handle_t handle)
{
    i2c_master_bus_handle_t & slot = shared_test_i2c_storage();
    if (slot == nullptr)
        slot = handle;
}

// Initializes the shared SPI bus once. NRF24L01 runs first in the merged
// binary and claims SPI2_HOST through its raw driver, so an already-claimed
// host is an expected condition there.
inline void shared_test_spi_bus()
{
    static bool initialized = false;
    if (initialized)
        return;

    spi_bus_config_t config = {};
    config.mosi_io_num = (gpio_num_t)CONFIG_MOSI_GPIO;
    config.miso_io_num = (gpio_num_t)CONFIG_MISO_GPIO;
    config.sclk_io_num = (gpio_num_t)CONFIG_SCLK_GPIO;
    config.quadwp_io_num = -1;
    config.quadhd_io_num = -1;
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &config, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(err == ESP_ERR_INVALID_STATE ? ESP_OK : err);
    SPIdev::init(SPI2_HOST);
    initialized = true;
}

#endif  // _SHARED_TEST_BUS_H_
