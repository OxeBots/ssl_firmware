/**
 * @file adc_reader.hpp
 * @brief This file contains the definition of the ADC_Reader class.
 */
#ifndef HAL_ADC_READER_HPP
#define HAL_ADC_READER_HPP

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>

#include <array>
#include <unordered_map>
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
#include "hal/adc_types.h"
#include "soc/soc_caps.h"

/**
 * @class ADC_Reader
 * @brief Manages ADC readings using the ESP-IDF's continuous mode via a
 * singleton pattern.
 *
 * This class is designed to handle the physical hardware resource (the
 * ADC peripheral). The singleton pattern ensures that only one instance of the
 * driver can exist, preventing resource conflicts and providing a single,
 * globally accessible point for ADC data. It uses a dedicated FreeRTOS task to
 * process ADC data, which is triggered by a hardware interrupt, ensuring
 * efficient and timely data acquisition without blocking other operations.
 */
class ADC_Reader
{
   public:
    /**
     * @brief Deleted copy constructor to enforce singleton pattern.
     */
    ADC_Reader(const ADC_Reader &) = delete;

    /**
     * @brief Deleted copy assignment operator to enforce singleton pattern.
     */
    ADC_Reader & operator=(const ADC_Reader &) = delete;

    /**
     * @brief Get the singleton instance of the ADC_Reader.
     *
     * @return A reference to the singleton instance.
     *
     * @note This prevents multiple parts of the code
     * from attempting to configure or control the ADC simultaneously, which
     * would lead to conflicts.
     */
    static ADC_Reader & get_instance()
    {
        // Use a static instance to ensure it's created only once.
        static ADC_Reader instance;
        return instance;
    }

    /**
     * @brief Initializes the ADC reader with the specified channels.
     *
     * @param channels A vector of ADC channels to read from.
     * @return ESP_OK on success, otherwise an error code.
     */
    esp_err_t init(const std::vector<adc_channel_t> & channels);

    /**
     * @brief Get the most recent raw ADC reading for a specific channel.
     *
     * @param channel The ADC channel to read from.
     * @return The raw 12-bit ADC value, or -1 if the channel is not found.
     */
    inline int get_raw_data(adc_channel_t channel)
    {
        auto it = m_adc_data.find(channel);
        if (it != m_adc_data.end()) return it->second;

        // Channel not found
        return -1;
    }

   private:
    /**
     * @brief Private default constructor for the singleton pattern.
     */
    ADC_Reader();

    /**
     * @brief Destructor that deinitializes the ADC and cleans up resources.
     */
    ~ADC_Reader();

    adc_continuous_handle_t
      m_adc_handle;  ///< Handle for the ADC continuous mode driver.
    TaskHandle_t m_task_handle;  ///< Handle for the FreeRTOS ADC reader task.
    std::unordered_map<adc_channel_t, int>
      m_adc_data;  ///< Stores the latest ADC data, mapping channel to raw
                   ///< value.
    bool m_initialized;  ///< Flag to prevent multiple initializations.

    /**
     * @brief ADC interrupt service routine (ISR) called when a conversion is
     * done.
     * @note This is an ISR and must execute quickly. It only sends a
     * notification to the `adc_task` to process the data, avoiding heavy
     * operations here.
     */
    static bool IRAM_ATTR
    s_adc_callback(adc_continuous_handle_t handle,
                   const adc_continuous_evt_data_t * edata, void * user_data);

    /**
     * @brief C-style static function that acts as a "trampoline" for the
     * FreeRTOS task.
     * @note `xTaskCreate` requires a C-style function pointer. This function
     * simply casts the `param` back to our class instance and calls the actual
     * C++ task method.
     */
    static void s_adc_task_wrapper(void * param);

    /**
     * @brief The main FreeRTOS task function for processing ADC data.
     * @note This task waits for notifications from the `s_adc_callback` ISR,
     * then reads and processes all available data from the ADC driver's
     * buffer.
     */
    void adc_task();
};

#endif  // HAL_ADC_READER_HPP
