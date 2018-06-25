#ifndef _DATASEND_H_
#define _DATASEND_H_

#include "mathTool.h"

void SendFlightData(void);
void SendImuSensor(void);
void SendRcData(void);
void SendGpsData(void);
void SendPidAttInner(void);
void SendPidAttOuter(void);
void SendPidPosInner(void);
void SendPidPosOuter(void);

#endif


