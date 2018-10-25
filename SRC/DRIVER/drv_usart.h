#ifndef _DRV_USART_H
#define _DRV_USART_H

#include "board.h"

typedef void (*UsartCallback)(uint8_t data);

void Usart_Open(uint8_t deviceNum, uint32_t baudrate);
void Usart_SetIRQCallback(uint8_t deviceNum, UsartCallback usartCallback);
void Usart_SendData(uint8_t deviceNum, uint8_t *DataToSend,uint8_t length);

#endif
