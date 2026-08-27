#ifndef COMMAND_INBOX_H
#define COMMAND_INBOX_H

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <stdint.h>

#include "RuntimeTypes.h"

class CommandInbox
{
   public:
    static constexpr size_t QUEUE_DEPTH = 8;

    CommandInbox() = default;
    ~CommandInbox();
    CommandInbox(const CommandInbox &) = delete;
    CommandInbox & operator=(const CommandInbox &) = delete;

    bool init();
    bool push(const RobotCommand & command, uint64_t received_at_us);
    bool pop(CommandEnvelope * envelope);
    size_t size() const;

   private:
    QueueHandle_t m_queue = nullptr;
    uint32_t m_next_sequence = 1;
};

#endif  // COMMAND_INBOX_H
