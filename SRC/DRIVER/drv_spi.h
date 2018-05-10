#ifndef __DRV_SPI_H__
#define __DRV_SPI_H__

#include "board.h"

void Spi_GPIO_Init(void);
void Spi_Open(uint8_t deviceNum);
uint8_t Spi_SingleWirteAndRead(uint8_t deviceNum, uint8_t dat);
void SPI_MultiWriteAndRead(uint8_t deviceNum, uint8_t *out, uint8_t *in, int len);

void Spi_GyroSingleWrite(uint8_t reg, uint8_t value);
void Spi_GyroMultiRead(uint8_t reg,uint8_t *data, uint8_t length);
void Spi_BaroSingleWrite(uint8_t reg, uint8_t value);
void Spi_BaroMultiRead(uint8_t reg,uint8_t *data, uint8_t length);

#endif










