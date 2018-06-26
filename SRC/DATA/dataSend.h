#ifndef _DATASEND_H_
#define _DATASEND_H_

#include "mathTool.h"

void BsklinkSendFlightData(void);
void BsklinkSendImuSensor(void);
void BsklinkSendGps(void);
void BsklinkSendRcData(void);
void BsklinkSendPidAttInner(void);
void BsklinkPidAttOuter(void);
void BsklinkSendPidPosInner(void);
void BsklinkSendPidPosOuter(void);

#endif


