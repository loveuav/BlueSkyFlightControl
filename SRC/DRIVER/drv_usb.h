#ifndef _DRV_USB_H
#define	_DRV_USB_H

#include "board.h"

typedef void (*UsbCallback)(uint8_t data);

void Usb_Init(void);
void Usb_Send(uint8_t *dataToSend, uint8_t length);
void Usb_SetRecvCallback(UsbCallback usbCallback);

#endif
