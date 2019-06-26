#include "MessageQueue.h"

#define INCREMENT_INDEX(X)  ((X) + 1 >= MAX_MESSAGES ? 0 : (X) + 1)



void InitializeMessageQueue(MessageQueue *queue){
    queue->start = 0;
    queue->current = 0;

    uint32_t i, j;
    for(i = 0; i < MAX_MESSAGES; i++){
        for(j = 0; j < MAX_MESSAGE_DATA_LENGTH; j++){
            queue->messages[i].data[j] = 0;
        }
        queue->messages[i].ID = 0;
    }
}

bool AddMessage(MessageQueue *queue, Message message){
    bool appendSuccessfull = false;

    if(INCREMENT_INDEX(queue->current) != queue->start){
        queue->messages[queue->current]     = message;
        queue->current                      = INCREMENT_INDEX(queue->current);
        appendSuccessfull                   = true;
    }
    else{
        //Stack full, do not append
    }

    return appendSuccessfull;
}

Message* GetNextMessage(MessageQueue *queue){
    Message* message = 0;

    if(queue->start != queue->current){
        message = &queue->messages[queue->start];
        queue->start = INCREMENT_INDEX(queue->start);
    }
    else{
        //There is no message in the stack, return null
    }

    return message;
}
