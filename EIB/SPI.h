#ifndef SPI_H
#define SPI_H

#include <stdint.h>
#include <stdbool.h>

void InitializeSPI(uint32_t sysClk);
void SPIWriteBuffer(uint8_t *buffer, uint8_t nBytes);

#endif
