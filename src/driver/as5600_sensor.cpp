#include "driver/as5600_sensor.hpp"

AS5600_Sensor::AS5600_Sensor(adc_channel_t channel) : m_channel(channel)
{
    m_do_calibration = adc_calibration_init();
}

bool AS5600_Sensor::adc_calibration_init()
{
    adc_cali_line_fitting_config_t cali_config = {
      .unit_id = ADC_UNIT_1,
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    return adc_cali_create_scheme_line_fitting(&cali_config, &m_cali_handle) ==
           ESP_OK;
}

double AS5600_Sensor::convert_to_angle(int raw_value)
{
    int voltage = 0;
    if (m_do_calibration)
    {
        adc_cali_raw_to_voltage(m_cali_handle, raw_value, &voltage);
    }
    else
    {
        voltage = raw_value * 3300 / 4096;
    }

    voltage = (voltage < 500) ? 500 : (voltage > 4500) ? 4500 : voltage;
    return (voltage - 500) * (360.0 / 4000.0);
}
