#ifndef MAILBOX_H
#define MAILBOX_H

//Standard includes
#include <stdint.h>
#include <stdbool.h>

//Projects includes
#include "Message.h"
#include "MessageQueue.h"

//Tivaware includes

void InitializeMailbox(void);
void SendMessage(Message *m);
Message* GetNextReceivedMessage(void);

#endif
