#ifndef __BATTERY_H
#define	__BATTERY_H

#include "mathTool.h"

enum
{
    BATTERY_NORMAL,
    BATTERY_LOW,
    BATTERY_CRITICAL_LOW
};

void BatteryVoltageUpdate(void);
int16_t GetBatteryVoltage(void);
int16_t GetBatteryCurrent(void);
uint8_t GetBatteryStatus(void);

#endif






