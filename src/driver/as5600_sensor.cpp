#include "driver/as5600_sensor.hpp"

AS5600_Sensor::AS5600_Sensor(adc_channel_t channel, ADC_Reader & reader)
: m_adc_reader(reader), m_channel(channel)
{
    m_do_calibration = adc_calibration_init();
}

AS5600_Sensor::~AS5600_Sensor()
{
    if (m_cali_handle) adc_cali_delete_scheme_line_fitting(m_cali_handle);
}

int AS5600_Sensor::get_calibrated_voltage()
{
    int raw_data = m_adc_reader.get_raw_data(m_channel);
    if (raw_data == -1) return -1;

    int voltage = 0;
    if (m_do_calibration)
        adc_cali_raw_to_voltage(m_cali_handle, raw_data, &voltage);
    else
        voltage = raw_data * 3300 / 4096;

    return voltage;
}

bool AS5600_Sensor::adc_calibration_init()
{
    adc_cali_line_fitting_config_t cali_config = {
      .unit_id = ADC_UNIT_1,
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
      .default_vref = ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_TP,
    };
    esp_err_t ret =
      adc_cali_create_scheme_line_fitting(&cali_config, &m_cali_handle);

    if (ret != ESP_OK)
        ESP_LOGW("AS5600_Sensor", "Calibration failed for channel %d",
                 m_channel);

    return ret == ESP_OK;
}
