#ifndef UART_HANDLER_H
#define UART_HANDLER_H

//Project includes
#include "Message.h"

//Standard includes
#include <stdint.h>
#include <stdbool.h>

//Tivaware includes

//defines
#define BUFFER_SIZE     512      //Ideally 2.5x max message data length

void InitializeUARTHandler(void);
uint32_t AddBytesToBuffer(uint8_t *bytes, uint32_t nBytes);
bool MessageAvailable(void);
Message ParseMessage(void);

#endif
