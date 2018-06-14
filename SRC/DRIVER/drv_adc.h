#ifndef __DRV_ADC_H__
#define __DRV_ADC_H__

#include "board.h"

void Adc_Init(void);
uint16_t GetVoltageAdcValue(void);
uint16_t GetCurrentAdcValue(void);

#endif





