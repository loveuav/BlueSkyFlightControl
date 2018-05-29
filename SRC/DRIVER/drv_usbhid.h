#ifndef _DRV_USBHID_H
#define	_DRV_USBHID_H

#include "board.h"

typedef void (*UsbHidCallback)(uint8_t data);

void UsbHid_Init(void);
void UsbHid_Send(uint8_t *dataToSend, uint8_t length);

#endif

