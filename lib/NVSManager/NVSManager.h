#ifndef NVS_MANAGER_H
#define NVS_MANAGER_H

#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <stddef.h>
#include <stdint.h>

#include <cstring>

class NVSManager
{
   public:
    static esp_err_t init();
    static void start();

    static esp_err_t save_i32(const char * ns, const char * key, int32_t value);
    static esp_err_t save_blob(const char * ns, const char * key, const void * data, size_t length);

    static esp_err_t load_i32(const char * ns, const char * key, int32_t * out);
    static esp_err_t load_blob(const char * ns, const char * key, void * out, size_t * length);

    static void erase_all();

    static constexpr size_t BLOB_MAX = 128;

   private:
    NVSManager() = delete;

    static void nvs_writer_task(void * arg);

    static QueueHandle_t m_op_queue;
    static constexpr size_t QUEUE_DEPTH = 16;

    enum class NvsOpType : uint8_t
    {
        WRITE_I32,
        WRITE_BLOB,
        READ_I32,
        READ_BLOB
    };

    struct NvsOp
    {
        NvsOpType type;
        char ns[16];
        char key[16];

        union
        {
            struct
            {
                uint8_t data[BLOB_MAX];
                size_t len;
            } write;

            struct
            {
                void * data_out;
                size_t * len_out;
                SemaphoreHandle_t response_sem;
                esp_err_t * result_out;
            } read;
        };
    };
};

#endif  // NVS_MANAGER_H
