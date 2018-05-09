#ifndef __DRV_SPI_H__
#define __DRV_SPI_H__

#include "board.h"

void Spi_Open(uint8_t deviceNum);
uint8_t Spi_SingleWirteAndRead(uint8_t deviceNum, uint8_t dat);
void SPI_MultiWriteAndRead(uint8_t deviceNum, uint8_t *out, uint8_t *in, int len);

#endif










