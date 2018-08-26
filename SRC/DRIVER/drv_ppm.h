#ifndef __DRV_PPM_H__
#define __DRV_PPM_H__

#include "board.h"

typedef void (*RcDataCallback)(RCDATA_t data);

void PPM_Init(void);
void PPM_Disable(void);
void PPM_SetRcDataCallback(RcDataCallback rcDataCallback);

#endif




