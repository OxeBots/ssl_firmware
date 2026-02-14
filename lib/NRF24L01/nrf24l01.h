/**
 * @file nrf24l01.h
 * @brief C++ wrapper for the mirf nRF24L01 library, configured as a receiver using IRQ interrupts.
 *
 */

#ifndef NRF24L01_HPP
#define NRF24L01_HPP

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "mirf.h"

// Define the type for our data-received callback function
typedef void (*DataReceivedCallback)(uint8_t * data, uint8_t len);  // Function pointer

/**
 * @class Nrf24Receiver
 * @brief A C++ wrapper for the mirf nRF24L01 library, configured as a receiver
 * using IRQ interrupts.
 */
class Nrf24Receiver
{
   public:
    /**
     * @brief Construct a new Nrf24Receiver object.
     * @param irq The GPIO pin number connected to the NRF24L01's IRQ pin.
     */
    explicit Nrf24Receiver(gpio_num_t irq);

    /**
     * @brief Destroy the Nrf24Receiver object.
     */
    ~Nrf24Receiver();

    /**
     * @brief Initializes the NRF24L01 module.
     * NOTE: This relies on the SPI/CE/CSN pins being set in sdkconfig
     *  as required by mirf.c. Run `pio run -t menuconfig` to set these.
     *
     * @param channel RF Channel (0-125).
     * @param payloadSize The fixed payload size (1-32 bytes).
     * @param tx_addr The 5-byte transmit address.
     * @param rx_addr The 5-byte receive address.
     * @return esp_err_t ESP_OK on success, or an error from the mirf library.
     */
    esp_err_t init(uint8_t channel, uint8_t payloadSize, const char * tx_addr, const char * rx_addr);

    /**
     * @brief Starts the receiver task.
     * @param callback The function to call when new data is received.
     * @return true if the task started successfully, false otherwise.
     */
    bool start(DataReceivedCallback callback);

    /** 
     * @brief Sends data non-blocking via NRF24L01.
     * @param data Pointer to the data buffer to send.
     * @param len Length of the data to send (max 32 bytes).
     */
    void send_data(const uint8_t * data, uint8_t len);

   private:
    /**
     * @brief Static ISR handler for the IRQ pin, activates the receiver task.
     * @param dev Pointer to the Nrf24Receiver instance.
     */
    static void IRAM_ATTR isr_handler(void * dev);

    /**
     * @brief Static function to launch the FreeRTOS task.
     * @param dev Pointer to the Nrf24Receiver instance.
     */
    static void task_wrapper(void * dev);

    /**
     * The main FreeRTOS task loop for receiving data.
     */
    void receiver_task();

   private:
    NRF24_t m_dev;                          // The underlying C struct for the mirf device
    gpio_num_t m_irqPin;                    // IRQ pin
    TaskHandle_t m_taskHandle;              // Handle for the receiver task
    QueueHandle_t m_interruptQueue;         // Queue to signal from ISR to task
    QueueHandle_t m_txQueue;                // Queue for outgoing data
    DataReceivedCallback m_onDataReceived;  // Callback function for received data
    uint8_t m_payloadSize;                  // Stored payload size
};

#endif  // NRF24L01_HPP
