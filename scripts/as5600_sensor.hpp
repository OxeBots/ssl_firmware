#ifndef DRIVER_AS5600_SENSOR_HPP
#define DRIVER_AS5600_SENSOR_HPP

#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "hal/adc_types.h"
// include PI
#include <math.h>

namespace config
{
namespace driver
{
//  0.087890625;
constexpr float AS5600_RAW_TO_DEGREES = 360.0 / 4096;
//  0.00153398078788564122971808758949;
constexpr float AS5600_RAW_TO_RADIANS = M_PI * 2.0 / 4096;
//  4.06901041666666e-6
constexpr float AS5600_RAW_TO_RPM = 1.0 / 4096 / 60;
}  // namespace driver
}  // namespace config

class AS5600_Sensor
{
   public:
    explicit AS5600_Sensor(adc_channel_t channel);
    adc_channel_t get_channel() const { return m_channel; }
    double convert_to_angle(int raw_value);

   private:
    bool adc_calibration_init();
    adc_channel_t m_channel;
    adc_cali_handle_t m_cali_handle = nullptr;
    bool m_do_calibration = false;
};

#endif  // DRIVER_AS5600_SENSOR_HPP
