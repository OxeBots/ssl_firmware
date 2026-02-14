#include "nrf24l01.h"

#include <cstring>

#include "esp_log.h"

static const char * TAG = "NRF24_CLASS";

Nrf24Receiver::Nrf24Receiver(gpio_num_t irq)
: m_irqPin(irq), m_taskHandle(NULL), m_interruptQueue(NULL), m_onDataReceived(nullptr), m_payloadSize(32)
{
    memset(&m_dev, 0, sizeof(NRF24_t));
}

Nrf24Receiver::~Nrf24Receiver()
{
    if (m_taskHandle)
        vTaskDelete(m_taskHandle);

    if (m_interruptQueue)
        vQueueDelete(m_interruptQueue);

    if (m_txQueue)
        vQueueDelete(m_txQueue);

    // Note: ISR handler removal is managed by main_esp32.cpp
    // which calls gpio_uninstall_isr_service()
    gpio_isr_handler_remove(m_irqPin);
    Nrf24_deinit(&m_dev);
}

esp_err_t Nrf24Receiver::init(uint8_t channel, uint8_t payloadSize, const char * tx_addr, const char * rx_addr)
{
    m_payloadSize = payloadSize;

    // 1. Initialize SPI and base NRF24 device
    Nrf24_init(&m_dev);

    // 2. Configure channel and payload size
    Nrf24_config(&m_dev, channel, payloadSize);

    // Match the Python script's settings: 250Kbps
    ESP_LOGI(TAG, "Setting Data Rate to 250Kbps");
    Nrf24_SetSpeedDataRates(&m_dev, 2);  // 2 = RF24_250KBPS

    esp_err_t ret = Nrf24_setRADDR(&m_dev, (uint8_t *)rx_addr);
    if (ret != ESP_OK)
    {
        ESP_LOGE(pcTaskGetName(NULL), "nrf24l01 not installed");
        while (true)
            vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // Configure ACK Handshake
    // To make Auto-Acknowledgement work, the TX_ADDR must be set
    // to the same tx_addr as the receiving pipe.
    // The `Nrf24_setTADDR` function sets BOTH `TX_ADDR` and `RX_ADDR_P0`.
    // We will use Pipe 0 for receiving.
    ret = Nrf24_setTADDR(&m_dev, (uint8_t *)tx_addr);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set TADDR/RADDR_P0. NRF24 not found?");
        return ret;
    }
    ESP_LOGI(TAG, "Using Pipe 0. RX_ADDR_P0 and TX_ADDR set to: %s", tx_addr);

    Nrf24_setRetransmitCount(&m_dev, 10);  // Up to 10 retransmits
    Nrf24_setRetransmitDelay(&m_dev, 4);   // 4 * 250us = 1250us (0x4 in upper nibble)

    // 3. Print details to confirm settings
    ESP_LOGI(TAG, "NRF24L01 Receiver Initialized:");
    Nrf24_printDetails(&m_dev);

    // 4. Configure GPIO interrupt for IRQ pin
    // Create a queue to handle gpio event from isr
    m_interruptQueue = xQueueCreate(10, sizeof(uint32_t));
    if (m_interruptQueue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create interrupt queue");
        return ESP_FAIL;
    }

    // 5. Configure TX Queue
    m_txQueue = xQueueCreate(10, m_payloadSize * sizeof(uint8_t));
    if (m_txQueue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create TX queue");
        return ESP_FAIL;
    }

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;  // Interrupt on falling edge (IRQ active LOW)
    io_conf.pin_bit_mask = (1ULL << m_irqPin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;  // Enable pull-up
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;

    ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure IRQ GPIO");
        return ret;
    }

    // Hook isr handler for specific gpio pin
    // Note: gpio_install_isr_service is called in main_esp32.cpp
    ret = gpio_isr_handler_add(m_irqPin, Nrf24Receiver::isr_handler, (void *)this);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add ISR handler");
        return ret;
    }

    return ESP_OK;
}

bool Nrf24Receiver::start(DataReceivedCallback callback)
{
    if (callback == nullptr)
    {
        ESP_LOGE(TAG, "Callback function cannot be null");
        return false;
    }
    m_onDataReceived = callback;

    // Start the receiver task
    BaseType_t task_created = xTaskCreate(&Nrf24Receiver::task_wrapper,  // Function to call
                                          "nrf24_receiver_task",         // Task name
                                          4096,                          // Stack size
                                          this,                          // Parameter to pass (this instance)
                                          5,                             // Priority
                                          &m_taskHandle                  // Task handle
    );

    if (task_created != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create receiver task");
        return false;
    }

    return true;
}

// Static ISR handler
void IRAM_ATTR Nrf24Receiver::isr_handler(void * dev)
{
    // `dev` is the `this` pointer passed during gpio_isr_handler_add
    Nrf24Receiver * instance = static_cast<Nrf24Receiver *>(dev);
    uint32_t gpio_num = instance->m_irqPin;
    // Send the pin number to the queue
    xQueueSendFromISR(instance->m_interruptQueue, &gpio_num, NULL);
}

// Static task trampoline
void Nrf24Receiver::task_wrapper(void * dev)
{
    // `dev` is the `this` pointer
    Nrf24Receiver * instance = static_cast<Nrf24Receiver *>(dev);
    // Call the member function
    instance->receiver_task();
}

// Member function task
void Nrf24Receiver::receiver_task()
{
    ESP_LOGI(TAG, "Receiver task started. Listening for data...");
    uint32_t io_num;
    uint8_t buffer[m_payloadSize];
    uint8_t tx_buffer[m_payloadSize];

    while (true)
    {
        // Wait for the ISR to notify us of an event
        if (xQueueReceive(m_interruptQueue, &io_num, portMAX_DELAY))
        {
            if (io_num == (uint32_t)m_irqPin)
            {
                // --- Drain the ENTIRE RX FIFO ---
                // Read all available packets
                while (Nrf24_dataReady(&m_dev))
                {
                    // Data is available, read it.
                    Nrf24_getData(&m_dev, buffer);

                    // Pass the data to the callback function
                    if (m_onDataReceived)
                        m_onDataReceived(buffer, m_payloadSize);
                }

                // --- Send all queued responses ---
                // Now that the RX FIFO is empty and we're in a clean state,
                // send all the pong-backs that we queued.
                while (xQueueReceive(m_txQueue, &tx_buffer, 0))
                {
                    // There is data to send
                    ESP_LOGI(TAG, "Sending pong-back...");
                    Nrf24_send(&m_dev, tx_buffer);

                    if (Nrf24_isSend(&m_dev, 100))
                        ESP_LOGI(TAG, "Pong-back sent successfully.");
                    else
                        ESP_LOGW(TAG, "Pong-back send failed.");
                }
            }
        }
    }
}

void Nrf24Receiver::send_data(const uint8_t * data, uint8_t len)
{
    if (len > 32)
    {
        ESP_LOGE(TAG, "Data length exceeds 32 bytes. Cannot send.");
        return;
    }

    // Add the data to the TX queue
    if (xQueueSend(m_txQueue, data, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to enqueue data for transmission");
        return;
    }
}
