/**
 * @file NRF24L01.cpp
 * @brief C++ wrapper driver for the mirf nRF24L01 library.
 *
 * Transport-only layer: sends and receives raw byte payloads.
 */

#include "NRF24L01.h"

static const char * TAG = "NRF24L01";

/**
 * @brief Construct a new NRF24L01 object.
 * @param irq The GPIO pin number connected to the NRF24L01's IRQ pin.
 */
NRF24L01::NRF24L01(gpio_num_t irq)
: m_irq_pin(irq),
  m_task_handle(NULL),
  m_interrupt_queue(NULL),
  m_tx_queue(NULL),
  m_on_data_received(nullptr),
  m_payload_size(32)
{
    memset(&m_dev, 0, sizeof(NRF24_t));
}

/**
 * @brief Destroy the NRF24L01 object and free resources.
 */
NRF24L01::~NRF24L01()
{
    if (m_task_handle)
        vTaskDelete(m_task_handle);
    if (m_interrupt_queue)
        vQueueDelete(m_interrupt_queue);
    if (m_tx_queue)
        vQueueDelete(m_tx_queue);

    gpio_isr_handler_remove(m_irq_pin);
    Nrf24_deinit(&m_dev);
}

/**
 * @brief Initializes the NRF24L01 module.
 * @param channel      RF Channel (0-125).
 * @param payload_size The fixed payload size (1-32 bytes).
 * @param tx_addr      The 5-byte transmit address.
 * @param rx_addr      The 5-byte receive address.
 * @return ESP_OK on success, or an error from the mirf library.
 */
esp_err_t NRF24L01::init(uint8_t channel, uint8_t payload_size, const char * tx_addr, const char * rx_addr)
{
    m_payload_size = payload_size;

    Nrf24_init(&m_dev);
    vTaskDelay(pdMS_TO_TICKS(100));

    Nrf24_config(&m_dev, channel, m_payload_size);

#ifdef CONFIG_RF_RATIO_250K
    Nrf24_SetSpeedDataRates(&m_dev, RF24_250KBPS);
#elif defined(CONFIG_RF_RATIO_1M)
    Nrf24_SetSpeedDataRates(&m_dev, RF24_1MBPS);
#elif defined(CONFIG_RF_RATIO_2M)
    Nrf24_SetSpeedDataRates(&m_dev, RF24_2MBPS);
#else
#error "Invalid RF Data Rate. Check menuconfig settings."
#endif

    esp_err_t ret = Nrf24_setRADDR(&m_dev, (uint8_t *)rx_addr);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "nrf24l01 not installed / failed to set RADDR (Check Wiring & add 10uF Capacitor!)");
        return ret;
    }

    // TX_ADDR must match RX_ADDR_P0 for Auto-ACK to work.
    ret = Nrf24_setTADDR(&m_dev, (uint8_t *)tx_addr);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set TADDR/RADDR_P0. NRF24 not found?");
        return ret;
    }

    ESP_LOGI(TAG, "Using Pipe 0. RX_ADDR_P0 and TX_ADDR set to: %s", tx_addr);

    Nrf24_setRetransmitCount(&m_dev, CONFIG_RETRANSMIT_COUNT);
    Nrf24_setRetransmitDelay(&m_dev, CONFIG_RETRANSMIT_DELAY);

    ESP_LOGI(TAG, "NRF24L01 Receiver Initialized:");
    Nrf24_printDetails(&m_dev);

    m_interrupt_queue = xQueueCreate(10, sizeof(uint32_t));
    m_tx_queue = xQueueCreate(10, m_payload_size * sizeof(uint8_t));
    if (m_interrupt_queue == NULL || m_tx_queue == NULL)
        return ESP_FAIL;

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << m_irq_pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;

    ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
        return ret;

    // gpio_install_isr_service() must be called in main.cpp before this.
    ret = gpio_isr_handler_add(m_irq_pin, NRF24L01::isr_handler, (void *)this);
    if (ret != ESP_OK)
        return ret;

    return ESP_OK;
}

/**
 * @brief Starts the background receiver task.
 * @param callback Called with raw bytes whenever a packet arrives.
 * @return true if the task started successfully.
 */
bool NRF24L01::start(data_received_callback_t callback)
{
    if (!callback)
        return false;
    m_on_data_received = callback;

    BaseType_t task_created =
      xTaskCreate(&NRF24L01::task_wrapper, "nrf24_receiver_task", 4096, this, 5, &m_task_handle);
    return task_created == pdPASS;
}

/**
 * @brief Static ISR handler for the IRQ pin — triggers receiver task via queue.
 */
void IRAM_ATTR NRF24L01::isr_handler(void * dev)
{
    auto * instance = static_cast<NRF24L01 *>(dev);
    uint32_t gpio_num = instance->m_irq_pin;
    xQueueSendFromISR(instance->m_interrupt_queue, &gpio_num, NULL);
}

/**
 * @brief Static trampoline to launch the FreeRTOS receiver task.
 */
void NRF24L01::task_wrapper(void * dev)
{
    static_cast<NRF24L01 *>(dev)->receiver_task();
}

/**
 * @brief Background task: drains the RX FIFO and flushes the TX queue.
 *
 * On RX: strips the dongle's prepended length byte, then passes the raw
 * payload bytes directly to the application callback — no protocol parsing.
 *
 * On TX: sends whatever raw frames the application enqueued via send_raw().
 */
void NRF24L01::receiver_task()
{
    ESP_LOGI(TAG, "Receiver task started. Listening for data...");
    uint32_t io_num;
    uint8_t buffer[32];
    uint8_t tx_buffer[32];

    while (true)
    {
        if (xQueueReceive(m_interrupt_queue, &io_num, portMAX_DELAY))
        {
            if (io_num == static_cast<uint32_t>(m_irq_pin))
            {
                // Drain all pending RX packets
                while (Nrf24_dataReady(&m_dev))
                {
                    Nrf24_getData(&m_dev, buffer);

                    // byte 0 is the dongle's payload-length prefix
                    uint8_t payload_len = buffer[0];

                    if (payload_len > 0 && payload_len <= 31 && m_on_data_received)
                    {
                        // Deliver raw bytes starting after the length byte
                        m_on_data_received(&buffer[1], payload_len);
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Dropped packet — invalid length byte: %d", payload_len);
                    }
                }

                // Flush queued TX frames
                while (xQueueReceive(m_tx_queue, &tx_buffer, 0))
                {
                    Nrf24_send(&m_dev, tx_buffer);
                    if (!Nrf24_isSend(&m_dev, 100))
                        ESP_LOGW(TAG, "TX sent (no Auto-ACK from USB adapter)");
                    else
                        ESP_LOGI(TAG, "TX sent and ACKed by USB adapter");
                }
            }
        }
    }
}

/**
 * @brief Enqueue raw bytes for over-the-air transmission.
 *
 * Builds a 32-byte RF frame by placing the payload length in byte 0 (dongle
 * convention) and copying the caller's data into bytes 1..len.  The rest of
 * the frame is zero-padded.  The caller must NOT include the length byte or
 * any padding — just the meaningful protocol payload.
 *
 * @param data  Pointer to payload bytes.
 * @param len   Payload length (must be ≤ 31).
 * @return ESP_OK on success.
 */
esp_err_t NRF24L01::send_raw(const uint8_t * data, uint8_t len)
{
    if (len > 31)
    {
        ESP_LOGE(TAG, "send_raw: payload too large (%d bytes, max 31)", len);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t frame[32] = {0};
    frame[0] = len;                // dongle length prefix
    memcpy(&frame[1], data, len);  // protocol payload

    if (xQueueSend(m_tx_queue, frame, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "send_raw: TX queue full");
        return ESP_FAIL;
    }

    return ESP_OK;
}
