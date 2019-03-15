#ifndef SPI_H
#define SPI_H

#include <stdint.h>
#include <stdbool.h>

void SPI_Initialize(void);
void SPI_Write(uint8_t Data);
uint8_t SPI_Read(void);

#endif
