/**
 * @file NVSManager.h
 * @brief Singleton manager for non-volatile storage (NVS) operations, ensuring hardware safety.
 */
#ifndef NVS_MANAGER_H
#define NVS_MANAGER_H

#include <esp_err.h>
#include <esp_log.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <stddef.h>
#include <stdint.h>

#include <functional>

class NVSManager
{
   public:
    /**
     * @brief Set callbacks to suspend/resume sensitive hardware (like ADC continuous mode)
     * to prevent flash contention during NVS read/write operations.
     */
    static void set_adc_callbacks(std::function<void()> suspend_cb, std::function<void()> resume_cb);

    /**
     * @brief Saves a 32-bit integer to NVS safely.
     */
    static esp_err_t save_i32(const char * ns, const char * key, int32_t value);

    /**
     * @brief Loads a 32-bit integer from NVS safely.
     */
    static esp_err_t load_i32(const char * ns, const char * key, int32_t * value);

    /**
     * @brief Saves a binary blob to NVS safely.
     */
    static esp_err_t save_blob(const char * ns, const char * key, const void * data, size_t length);

    /**
     * @brief Loads a binary blob from NVS safely.
     */
    static esp_err_t load_blob(const char * ns, const char * key, void * data, size_t * length);

    NVSManager(const NVSManager &) = delete;

    NVSManager & operator=(const NVSManager &) = delete;

   private:
    static std::function<void()> m_suspend_cb;
    static std::function<void()> m_resume_cb;

    /**
     * @brief Executes an NVS operation while wrapping it in the suspend/resume callbacks.
     */
    static void execute_safe(const std::function<void()> & func);
};

#endif  // NVS_MANAGER_H
