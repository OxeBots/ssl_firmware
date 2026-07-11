#include "NVSManager.h"

static const char * TAG = "NVSManager";

QueueHandle_t NVSManager::m_op_queue = nullptr;

/**
 * @brief Erase the entire NVS partition and re-initialize.
 *
 * Destroys all stored data including robot ID, wheel calibration, and PID gains.
 * The caller should reboot immediately after this call to start with a clean state.
 */
void NVSManager::erase_all()
{
    ESP_LOGW(TAG, "Erasing entire NVS partition...");
    esp_err_t err = nvs_flash_erase();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_flash_erase failed: %s", esp_err_to_name(err));
        return;
    }
    err = nvs_flash_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_flash_init after erase failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "NVS partition erased and reinitialized.");
}

/**
 * @brief Initializes the NVS flash partition.
 *
 * Calls nvs_flash_init(). If the partition is corrupt or has an incompatible
 * version, erases it and reinitializes.
 *
 * @return ESP_OK on success.
 */
esp_err_t NVSManager::init()
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS partition corrupt or incompatible — erasing and reinitializing.");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    return ret;
}

/**
 * @brief Create the internal operation queue and launch the nvs_writer task.
 *
 * Must be called after init(). The nvs_writer task (priority 4, 4096-byte stack)
 * is the sole caller of ESP-IDF nvs_* functions. Serializing all NVS access
 * through one task avoids flash contention with the ADC continuous driver.
 *
 * After this call, save_* and load_* are safe from any task context.
 */
void NVSManager::start()
{
    configASSERT(m_op_queue == nullptr);
    m_op_queue = xQueueCreate(QUEUE_DEPTH, sizeof(NvsOp));
    configASSERT(m_op_queue != nullptr);
    xTaskCreate(nvs_writer_task, "nvs_writer", 4096, nullptr, 4, nullptr);
    ESP_LOGI(TAG, "NVS writer started (queue_depth=%zu, blob_max=%zu).", QUEUE_DEPTH, BLOB_MAX);
}

/**
 * @brief Queue a 32-bit integer write.
 *
 * The value is copied into the queue message at call time. The caller's
 * data is safe to modify or free immediately after this returns.
 *
 * Safe from any task or ISR context after start() has been called.
 *
 * @param ns   Namespace (max 15 chars).
 * @param key  Key (max 15 chars).
 * @param value 32-bit integer to write.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if start() not called,
 *         ESP_ERR_TIMEOUT if the queue is full.
 */
esp_err_t NVSManager::save_i32(const char * ns, const char * key, int32_t value)
{
    if (m_op_queue == nullptr)
    {
        ESP_LOGE(TAG, "save_i32 called before start()");
        return ESP_ERR_INVALID_STATE;
    }

    NvsOp op{};
    op.type = NvsOpType::WRITE_I32;
    memcpy(op.write.data, &value, sizeof(value));
    op.write.len = sizeof(value);
    strncpy(op.ns, ns, sizeof(op.ns) - 1);
    strncpy(op.key, key, sizeof(op.key) - 1);

    if (xQueueSend(m_op_queue, &op, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(
          TAG, "NVS write queue full — i32 [%s/%s]=%ld not persisted!", ns, key, (long)value);
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGD(TAG, "Queued i32 write [%s/%s] = %ld", ns, key, (long)value);
    return ESP_OK;
}

/**
 * @brief Queue a binary blob write.
 *
 * The blob data is copied into the queue message at call time. The caller's
 * buffer is safe to modify or free immediately after this returns.
 *
 * Safe from any task or ISR context after start() has been called.
 *
 * @param ns     Namespace (max 15 chars).
 * @param key    Key (max 15 chars).
 * @param data   Pointer to binary data.
 * @param length Size of data in bytes. Must be <= BLOB_MAX (128).
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if start() not called,
 *         ESP_ERR_INVALID_ARG if length > BLOB_MAX,
 *         ESP_ERR_TIMEOUT if the queue is full.
 */
esp_err_t NVSManager::save_blob(const char * ns, const char * key, const void * data, size_t length)
{
    if (m_op_queue == nullptr)
    {
        ESP_LOGE(TAG, "save_blob called before start()");
        return ESP_ERR_INVALID_STATE;
    }

    if (length > BLOB_MAX)
    {
        ESP_LOGE(
          TAG, "save_blob: blob too large (%zu > %zu) for [%s/%s]", length, BLOB_MAX, ns, key);
        return ESP_ERR_INVALID_ARG;
    }

    NvsOp op{};
    op.type = NvsOpType::WRITE_BLOB;
    op.write.len = length;
    memcpy(op.write.data, data, length);
    strncpy(op.ns, ns, sizeof(op.ns) - 1);
    strncpy(op.key, key, sizeof(op.key) - 1);

    if (xQueueSend(m_op_queue, &op, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "NVS write queue full — blob [%s/%s] not persisted!", ns, key);
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGD(TAG, "Queued blob write [%s/%s] (%zu bytes)", ns, key, length);
    return ESP_OK;
}

/**
 * @brief Read a 32-bit integer from NVS.
 *
 * Blocks the calling task until the nvs_writer task completes the read.
 * Must NOT be called from ISR context.
 *
 * @param ns   Namespace (max 15 chars).
 * @param key  Key (max 15 chars).
 * @param out  Pointer to store the loaded value.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if start() not called,
 *         ESP_ERR_TIMEOUT if the queue is full,
 *         ESP_ERR_NOT_FOUND if the key does not exist.
 */
esp_err_t NVSManager::load_i32(const char * ns, const char * key, int32_t * out)
{
    configASSERT(!xPortInIsrContext());

    if (m_op_queue == nullptr)
    {
        ESP_LOGE(TAG, "load_i32 called before start()");
        return ESP_ERR_INVALID_STATE;
    }

    SemaphoreHandle_t sem = xSemaphoreCreateBinary();
    configASSERT(sem != nullptr);

    esp_err_t result = ESP_FAIL;

    NvsOp op{};
    op.type = NvsOpType::READ_I32;
    op.read.data_out = out;
    op.read.response_sem = sem;
    op.read.result_out = &result;
    strncpy(op.ns, ns, sizeof(op.ns) - 1);
    strncpy(op.key, key, sizeof(op.key) - 1);

    if (xQueueSend(m_op_queue, &op, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "NVS read queue full — i32 [%s/%s]", ns, key);
        vSemaphoreDelete(sem);
        return ESP_ERR_TIMEOUT;
    }

    xSemaphoreTake(sem, portMAX_DELAY);
    vSemaphoreDelete(sem);

    return result;
}

/**
 * @brief Read a binary blob from NVS.
 *
 * Blocks the calling task until the nvs_writer task completes the read.
 * Must NOT be called from ISR context.
 *
 * The caller must set *length to the size of the output buffer before calling.
 * On success, *length is updated to the actual number of bytes read.
 *
 * @param ns     Namespace (max 15 chars).
 * @param key    Key (max 15 chars).
 * @param out    Pointer to store the loaded data.
 * @param length Pointer to buffer size (in/out: updated to actual bytes read).
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if start() not called,
 *         ESP_ERR_INVALID_ARG if *length > BLOB_MAX,
 *         ESP_ERR_TIMEOUT if the queue is full,
 *         ESP_ERR_NVS_NOT_FOUND if the key does not exist.
 */
esp_err_t NVSManager::load_blob(const char * ns, const char * key, void * out, size_t * length)
{
    configASSERT(!xPortInIsrContext());

    if (m_op_queue == nullptr)
    {
        ESP_LOGE(TAG, "load_blob called before start()");
        return ESP_ERR_INVALID_STATE;
    }

    if (*length > BLOB_MAX)
    {
        ESP_LOGE(
          TAG, "load_blob: requested length (%zu) exceeds BLOB_MAX (%zu)", *length, BLOB_MAX);
        return ESP_ERR_INVALID_ARG;
    }

    SemaphoreHandle_t sem = xSemaphoreCreateBinary();
    configASSERT(sem != nullptr);

    esp_err_t result = ESP_FAIL;

    NvsOp op{};
    op.type = NvsOpType::READ_BLOB;
    op.read.data_out = out;
    op.read.len_out = length;
    op.read.response_sem = sem;
    op.read.result_out = &result;
    strncpy(op.ns, ns, sizeof(op.ns) - 1);
    strncpy(op.key, key, sizeof(op.key) - 1);

    if (xQueueSend(m_op_queue, &op, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "NVS read queue full — blob [%s/%s]", ns, key);
        vSemaphoreDelete(sem);
        return ESP_ERR_TIMEOUT;
    }

    xSemaphoreTake(sem, portMAX_DELAY);
    vSemaphoreDelete(sem);

    return result;
}

/**
 * @brief FreeRTOS task function that processes all NVS operations from the queue.
 *
 * This task is the sole caller of ESP-IDF nvs_* functions, which serializes
 * all flash access and avoids contention of the hardware.
 */
void NVSManager::nvs_writer_task(void * /*arg*/)
{
    NvsOp op;

    while (true)
    {
        if (xQueueReceive(m_op_queue, &op, portMAX_DELAY) != pdTRUE)
            continue;

        esp_err_t err = ESP_OK;
        nvs_handle_t handle;

        switch (op.type)
        {
            case NvsOpType::WRITE_I32:
            {
                int32_t value;
                memcpy(&value, op.write.data, sizeof(value));

                err = nvs_open(op.ns, NVS_READWRITE, &handle);
                if (err == ESP_OK)
                {
                    err = nvs_set_i32(handle, op.key, value);
                    if (err == ESP_OK)
                        err = nvs_commit(handle);
                    nvs_close(handle);
                }

                if (err == ESP_OK)
                    ESP_LOGD(TAG, "Saved i32  [%s/%s] = %ld", op.ns, op.key, (long)value);
                else
                    ESP_LOGE(
                      TAG, "Failed to save i32 [%s/%s]: %s", op.ns, op.key, esp_err_to_name(err));
                break;
            }

            case NvsOpType::WRITE_BLOB:
            {
                err = nvs_open(op.ns, NVS_READWRITE, &handle);
                if (err == ESP_OK)
                {
                    err = nvs_set_blob(handle, op.key, op.write.data, op.write.len);
                    if (err == ESP_OK)
                        err = nvs_commit(handle);
                    nvs_close(handle);
                }

                if (err == ESP_OK)
                    ESP_LOGD(TAG, "Saved blob [%s/%s] (%zu bytes)", op.ns, op.key, op.write.len);
                else
                    ESP_LOGE(
                      TAG, "Failed to save blob [%s/%s]: %s", op.ns, op.key, esp_err_to_name(err));
                break;
            }

            case NvsOpType::READ_I32:
            {
                err = nvs_open(op.ns, NVS_READONLY, &handle);
                if (err == ESP_OK)
                {
                    err = nvs_get_i32(handle, op.key, static_cast<int32_t *>(op.read.data_out));
                    nvs_close(handle);
                }

                *op.read.result_out = err;
                xSemaphoreGive(op.read.response_sem);
                break;
            }

            case NvsOpType::READ_BLOB:
            {
                err = nvs_open(op.ns, NVS_READONLY, &handle);
                if (err == ESP_OK)
                {
                    err = nvs_get_blob(handle, op.key, op.read.data_out, op.read.len_out);
                    nvs_close(handle);
                }

                *op.read.result_out = err;
                xSemaphoreGive(op.read.response_sem);
                break;
            }
        }
    }
}
