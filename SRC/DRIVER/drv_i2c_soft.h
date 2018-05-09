#ifndef __DRV_I2C_SOFT_H__
#define __DRV_I2C_SOFT_H__

#include "board.h"

void Soft_I2c_Open(uint8_t deviceNum);
bool Soft_I2c_Single_Write(uint8_t deviceNum, u8 SlaveAddress,u8 REG_Address,u8 REG_data);
uint8_t Soft_I2C_Single_Read(uint8_t deviceNum, u8 SlaveAddress,u8 REG_Address);
bool Soft_I2C_Multi_Read(uint8_t deviceNum, u8 SlaveAddress,u8 REG_Address,u8 * ptChar,u8 size);

#endif



