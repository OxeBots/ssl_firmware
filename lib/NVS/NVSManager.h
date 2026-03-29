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
    static esp_err_t init();

    static void set_adc_callbacks(std::function<void()> suspend_cb, std::function<void()> resume_cb);

    static esp_err_t save_i32(const char * ns, const char * key, int32_t value);
    static esp_err_t save_i32_direct(const char * ns, const char * key, int32_t value);
    static esp_err_t load_i32(const char * ns, const char * key, int32_t * value);

    static esp_err_t save_blob(const char * ns, const char * key, const void * data, size_t length);
    static esp_err_t load_blob(const char * ns, const char * key, void * data, size_t * length);

   private:
    static std::function<void()> m_suspend_cb;
    static std::function<void()> m_resume_cb;

    static void execute_safe(const std::function<void()> & func);
};

#endif  // NVS_MANAGER_H
