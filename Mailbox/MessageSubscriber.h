#ifndef MESSAGE_SUBSCRIBER_H
#define MESSAGE_SUBSCRIBER_H

//Project includes
#include "Message.h"

//Standard includes
#include <stdint.h>

//Tivaware includes

//Type declarations
typedef struct MessageSubscription_tag{
    uint16_t ID : 11;
    void (*pCallback)(Message);
    uint8_t reserved : 5;
} MessageSubscription;

void InitializeMessageSubscriber(MessageSubscription *subscriptions, uint32_t nSubscriptions);
void ProcessMessages(void);

#endif
