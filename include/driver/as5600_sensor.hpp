#ifndef DRIVER_AS5600_SENSOR_HPP
#define DRIVER_AS5600_SENSOR_HPP

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>

#include <array>
#include <map>
#include <vector>

#include "driver/adc_types_legacy.h"
#include "driver/ledc.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/adc_reader.hpp"
#include "hal/adc_types.h"
#include "soc/soc_caps.h"

namespace config
{
namespace driver
{
} // namespace driver
}  // namespace config

class AS5600_Sensor
{
   public:
    AS5600_Sensor(adc_channel_t channel, ADC_Reader & reader);
    ~AS5600_Sensor();
    int get_calibrated_voltage();

   private:
    ADC_Reader & m_adc_reader;
    adc_channel_t m_channel;
    adc_cali_handle_t m_cali_handle = NULL;
    bool m_do_calibration = false;

    bool adc_calibration_init();
};

#endif  // DRIVER_AS5600_SENSOR_HPP
