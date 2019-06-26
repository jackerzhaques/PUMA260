#include "Mailbox.h"
#include "UARTHandler.h"

#include <stdio.h>
#include <string.h>
#include <driverlib/interrupt.h>

static MessageQueue IncomingMessages;

static uint32_t nOverflows = 0;

//Function prototypes

void InitializeMailbox(void){
    InitializeMessageQueue(&IncomingMessages);
    InitializeUARTHandler();
}

void SendMessage(Message *m){
    uint8_t end[1];
    end[0] = MESSAGE_END_BYTE;

    //Maximum message ID is 4 characters long
    uint8_t header[7];
    snprintf((char*)header, 7, "%c%u%c", MESSAGE_START_BYTE, (char)m->ID, MESSAGE_ID_END_BYTE);

    //Convert bytes to ASCII
    char data[MAX_MESSAGE_DATA_LENGTH][4];  //3 chars + null termination at most
    memset(data, 0, MAX_MESSAGE_DATA_LENGTH*4);
    uint8_t i = 0;
    for(i = 0; i < m->DLC; i++){
        snprintf((char*)data[i], 4, "%u", m->data[i]);
    }

    //Write the bytes to a buffer
    uint8_t buffer[MAX_MESSAGE_ASCII_LENGTH];
    memset(buffer, 0, MAX_MESSAGE_ASCII_LENGTH);
    uint8_t j = 0, idx = 0;
    for(i = 0; i < m->DLC; i++){
        for(j = 0; j < 3; j++){
            uint8_t byte = data[i][j];
            if(byte){
                buffer[idx] = data[i][j];
                idx++;
            }
            else{
                //Do nothing
            }
        }

        //Append comma if not last index
        if(i < m->DLC - 1){
            buffer[idx] = MESSAGE_FRAMING_BYTE;
            idx++;
        }
    }

    //Disable interrupts while adding bytes to buffer
    IntMasterDisable();

    uint32_t nBytesSent;
    nBytesSent = AddBytesToBuffer(header, strlen((char*)header));
    nBytesSent += AddBytesToBuffer(buffer, strlen((char*)buffer));
    nBytesSent += AddBytesToBuffer(end, 1);

    if(nBytesSent != strlen((char*)header) + strlen((char*)buffer) + 1){
        //Buffer overflow has occurred
        //Currently we are only logging this.
        //If this number becomes too high, increase the buffer size
        nOverflows++;
    }

    IntMasterEnable();
}

Message* GetNextReceivedMessage(void){
    return GetNextMessage(&IncomingMessages);
}
