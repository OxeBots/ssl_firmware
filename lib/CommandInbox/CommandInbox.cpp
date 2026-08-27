#include "CommandInbox.h"

CommandInbox::~CommandInbox()
{
    if (m_queue != nullptr)
        vQueueDelete(m_queue);
}

bool CommandInbox::init()
{
    if (m_queue != nullptr)
        return false;

    m_queue = xQueueCreate(QUEUE_DEPTH, sizeof(CommandEnvelope));
    return m_queue != nullptr;
}

bool CommandInbox::push(const RobotCommand & command, uint64_t received_at_us)
{
    if (m_queue == nullptr)
        return false;

    CommandEnvelope envelope = {};
    envelope.command = command;
    envelope.received_at_us = received_at_us;
    envelope.sequence = m_next_sequence.fetch_add(1);
    return xQueueSend(m_queue, &envelope, 0) == pdPASS;
}

bool CommandInbox::pop(CommandEnvelope * envelope)
{
    if (m_queue == nullptr || envelope == nullptr)
        return false;

    return xQueueReceive(m_queue, envelope, 0) == pdPASS;
}

size_t CommandInbox::size() const
{
    return m_queue == nullptr ? 0 : static_cast<size_t>(uxQueueMessagesWaiting(m_queue));
}
