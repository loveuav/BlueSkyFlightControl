#ifndef _DRV_USART_H
#define _DRV_USART_H

#include "board.h"

typedef void (*UsartCallback)(uint8_t data);

void Usart_Open(uint8_t deviceNum);
void Usart_SetIRQCallback(uint8_t deviceNum, UsartCallback usartCallback);

#endif
