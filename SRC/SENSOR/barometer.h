#ifndef __BAROMETER_H
#define	__BAROMETER_H

#include "sensor.h"

void BaroDataPreTreat(void);
int32_t BaroGetAlt(void);
float BaroGetTemp(void);
float BaroGetVelocity(void);

#endif



