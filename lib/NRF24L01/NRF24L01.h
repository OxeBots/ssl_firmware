#ifndef _NRF24L01_H_
#define _NRF24L01_H_

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <cstring>

#include "mirf.h"

typedef void (*data_received_callback_t)(const uint8_t * payload, uint8_t len);

class NRF24L01
{
   public:
    explicit NRF24L01(gpio_num_t irq);
    ~NRF24L01();

    esp_err_t init(uint8_t channel, uint8_t payload_size, const char * tx_addr, const char * rx_addr);
    bool start(data_received_callback_t callback);
    esp_err_t send_raw(const uint8_t * data, uint8_t len);

   private:
    static void IRAM_ATTR isr_handler(void * dev);
    static void task_wrapper(void * dev);
    void receiver_task();

    NRF24_t m_dev;
    gpio_num_t m_irq_pin;
    TaskHandle_t m_task_handle;
    QueueHandle_t m_interrupt_queue;
    QueueHandle_t m_tx_queue;
    data_received_callback_t m_on_data_received;
    uint8_t m_payload_size;
};

#endif  // _NRF24L01_H_
