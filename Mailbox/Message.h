#ifndef MESSAGE_H
#define MESSAGE_H

#define MAX_MESSAGE_DATA_LENGTH     8  //Bytes
#define MAX_MESSAGE_ASCII_LENGTH    31 //8 bytes in ASCII representation + 7 commas

#define MESSAGE_START_BYTE          ':'
#define MESSAGE_ID_END_BYTE         ' '
#define MESSAGE_FRAMING_BYTE        ','     //Separates ascii data bytes
#define MESSAGE_END_BYTE            ';'

#include <stdint.h>

typedef struct MessageData_Tag{
    uint8_t bytes[MAX_MESSAGE_DATA_LENGTH];
} MessageData;

typedef struct Message_tag{
    uint16_t ID;
    uint8_t DLC;
    union{
        uint8_t data[8];
        uint8_t *pData;
    };
} Message;

#endif
