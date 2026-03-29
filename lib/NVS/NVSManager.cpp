/**
 * @file NVSManager.cpp
 * @brief Singleton manager for non-volatile storage (NVS) operations, ensuring hardware safety.
 */
#include "NVSManager.h"

static const char * TAG = "NVSManager";

std::function<void()> NVSManager::m_suspend_cb = nullptr;
std::function<void()> NVSManager::m_resume_cb = nullptr;

/**
 * @brief Initializes the NVS flash partition.
 * @return ESP_OK on success.
 */
esp_err_t NVSManager::init()
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    return ret;
}

/**
 * @brief Set callbacks to suspend/resume sensitive hardware (like ADC continuous mode)
 * to prevent flash contention during NVS read/write operations.
 * @param suspend_cb Function to call before NVS access.
 * @param resume_cb Function to call after NVS access.
 */
void NVSManager::set_adc_callbacks(std::function<void()> suspend_cb, std::function<void()> resume_cb)
{
    m_suspend_cb = suspend_cb;
    m_resume_cb = resume_cb;
}

/**
 * @brief Executes an NVS operation while wrapping it in the suspend/resume callbacks.
 * @param func Lambda or function to execute.
 */
void NVSManager::execute_safe(const std::function<void()> & func)
{
    if (m_suspend_cb)
        m_suspend_cb();

    func();

    if (m_resume_cb)
        m_resume_cb();
}

/**
 * @brief Saves a 32-bit integer to NVS safely.
 * @param ns Namespace string.
 * @param key Key string.
 * @param value Integer value to save.
 * @return ESP_OK on success.
 */
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

    if (err == ESP_OK)
        ESP_LOGI(TAG, "Saved i32  [%s/%s] = %ld", ns, key, (long)value);
    else
        ESP_LOGE(TAG, "Failed to save i32 [%s/%s]: %s", ns, key, esp_err_to_name(err));

    return err;
}

/**
 * @brief Saves a 32-bit integer directly to NVS without calling the ADC
 *        suspend/resume callbacks.  Safe to call from any task context that
 *        does not hold ADC continuous driver resources.
 */
esp_err_t NVSManager::save_i32_direct(const char * ns, const char * key, int32_t value)
{
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;

    err = nvs_open(ns, NVS_READWRITE, &handle);
    if (err == ESP_OK)
    {
        err = nvs_set_i32(handle, key, value);
        if (err == ESP_OK)
            err = nvs_commit(handle);
        nvs_close(handle);
    }

    if (err == ESP_OK)
        ESP_LOGI(TAG, "Saved i32  [%s/%s] = %ld (direct)", ns, key, (long)value);
    else
        ESP_LOGE(TAG, "Failed to save i32 [%s/%s]: %s", ns, key, esp_err_to_name(err));

    return err;
}

/**
 * @brief Loads a 32-bit integer from NVS safely.
 * @param ns Namespace string.
 * @param key Key string.
 * @param value Pointer to store loaded value.
 * @return ESP_OK on success.
 */
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

    return err;
}

/**
 * @brief Saves a binary blob to NVS safely.
 * @param ns Namespace string.
 * @param key Key string.
 * @param data Pointer to the binary data.
 * @param length Size of the binary data.
 * @return ESP_OK on success.
 */
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

    if (err == ESP_OK)
        ESP_LOGI(TAG, "Saved blob [%s/%s] (%zu bytes)", ns, key, length);
    else
        ESP_LOGE(TAG, "Failed to save blob [%s/%s]: %s", ns, key, esp_err_to_name(err));

    return err;
}

/**
 * @brief Loads a binary blob from NVS safely.
 * @param ns Namespace string.
 * @param key Key string.
 * @param data Pointer to store the loaded binary data.
 * @param length Pointer to the size of the binary data buffer.
 * @return ESP_OK on success.
 */
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

    return err;
}
