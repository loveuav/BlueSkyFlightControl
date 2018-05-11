#ifndef __DRV_FLASH_H__
#define __DRV_FLASH_H__

#include "board.h"

uint8_t Flash_ReadByte(uint32_t start_addr, uint16_t cnt);
bool Flash_WriteByte(uint32_t dest,uint8_t *src,uint32_t length);

#endif






