#include "NVSManager.h"

static const char * TAG = "NVSManager";

std::function<void()> NVSManager::m_suspend_cb = nullptr;
std::function<void()> NVSManager::m_resume_cb = nullptr;

void NVSManager::set_adc_callbacks(std::function<void()> suspend_cb, std::function<void()> resume_cb)
{
    m_suspend_cb = suspend_cb;
    m_resume_cb = resume_cb;
}

void NVSManager::execute_safe(const std::function<void()> & func)
{
    if (m_suspend_cb)
    {
        m_suspend_cb();
    }

    func();

    if (m_resume_cb)
    {
        m_resume_cb();
    }
}

esp_err_t NVSManager::save_i32(const char * ns, const char * key, int32_t value)
{
    esp_err_t err = ESP_OK;
    execute_safe([&]() {
        nvs_handle_t handle;
        err = nvs_open(ns, NVS_READWRITE, &handle);
        if (err == ESP_OK)
        {
            err = nvs_set_i32(handle, key, value);
            if (err == ESP_OK)
                err = nvs_commit(handle);
            nvs_close(handle);
        }
    });

    if (err != ESP_OK)
        ESP_LOGE(TAG, "Failed to save i32 [%s/%s]: %s", ns, key, esp_err_to_name(err));
    return err;
}

esp_err_t NVSManager::load_i32(const char * ns, const char * key, int32_t * value)
{
    esp_err_t err = ESP_OK;
    execute_safe([&]() {
        nvs_handle_t handle;
        err = nvs_open(ns, NVS_READONLY, &handle);
        if (err == ESP_OK)
        {
            err = nvs_get_i32(handle, key, value);
            nvs_close(handle);
        }
    });
    return err;  // No logging on read fail, expected behavior if not calibrated
}

esp_err_t NVSManager::save_blob(const char * ns, const char * key, const void * data, size_t length)
{
    esp_err_t err = ESP_OK;
    execute_safe([&]() {
        nvs_handle_t handle;
        err = nvs_open(ns, NVS_READWRITE, &handle);
        if (err == ESP_OK)
        {
            err = nvs_set_blob(handle, key, data, length);
            if (err == ESP_OK)
                err = nvs_commit(handle);
            nvs_close(handle);
        }
    });

    if (err != ESP_OK)
        ESP_LOGE(TAG, "Failed to save blob [%s/%s]: %s", ns, key, esp_err_to_name(err));
    return err;
}

esp_err_t NVSManager::load_blob(const char * ns, const char * key, void * data, size_t * length)
{
    esp_err_t err = ESP_OK;
    execute_safe([&]() {
        nvs_handle_t handle;
        err = nvs_open(ns, NVS_READONLY, &handle);
        if (err == ESP_OK)
        {
            err = nvs_get_blob(handle, key, data, length);
            nvs_close(handle);
        }
    });
    return err;  // No logging on read fail, expected behavior if not calibrated
}
