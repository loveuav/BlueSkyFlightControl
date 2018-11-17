#ifndef __DRV_SBUS_H__
#define __DRV_SBUS_H__

#include "board.h"

typedef void (*RcDataCallback)(RCDATA_t data);

void Sbus_Init(void);
void Sbus_SetRcDataCallback(RcDataCallback rcDataCallback);
void Sbus_Disable(void);

#endif

