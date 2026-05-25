#ifndef _VL53L5CX_H_
#define _VL53L5CX_H_

#include <driver/i2c_master.h>
#include <stdint.h>

#include "I2Cdev.h"
#include "vl53l5cx_api.h"

class VL53L5CX
{
   public:
    static constexpr uint8_t DEFAULT_ADDRESS = 0x52;

    VL53L5CX(uint8_t address = DEFAULT_ADDRESS);

    esp_err_t init(i2c_master_bus_handle_t bus_handle);
    bool test_connection();
    uint8_t start_ranging();
    uint8_t check_data_ready(uint8_t *is_ready);
    uint8_t get_ranging_data(VL53L5CX_ResultsData *results);
    uint8_t stop_ranging();

   private:
    uint8_t m_dev_addr;
    VL53L5CX_Configuration m_dev;
    i2c_master_dev_handle_t m_dev_handle;
};

#endif /* _VL53L5CX_H_ */
