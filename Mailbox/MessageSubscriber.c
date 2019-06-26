#include "MessageSubscriber.h"

//Project includes
#include "UARTHandler.h"

static MessageSubscription *pSubscriptions = 0;
static uint32_t nMessageSubscriptions = 0;

void InitializeMessageSubscriber(MessageSubscription *subscriptions, uint32_t nSubscriptions){
    pSubscriptions = subscriptions;
    nMessageSubscriptions = nSubscriptions;
}

void ProcessMessages(void){
    //Parse all UART messages
    Message m = ParseMessage();

    while(m.ID != 0){
        uint32_t i = 0;
        for(i = 0; i < nMessageSubscriptions; i++){
            if(pSubscriptions[i].ID == m.ID){
                pSubscriptions[i].pCallback(m);
            }
            else{
                //This subscription does not subscribe to this message, ignore
            }
        }

        m = ParseMessage();
    }
}
