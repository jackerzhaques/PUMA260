#ifndef MESSAGE_QUEUE_H
#define MESSAGE_QUEUE_H

#define MAX_MESSAGES    10

//Project includes
#include "Message.h"

//Standard includes
#include <stdint.h>
#include <stdbool.h>

typedef struct MessageQueue_tag{
    Message messages[MAX_MESSAGES];
    uint32_t current;
    uint32_t start;
} MessageQueue;

void InitializeMessageQueue(MessageQueue *queue);
bool AddMessage(MessageQueue *queue, Message message);
Message* GetNextMessage(MessageQueue *queue);

#endif
