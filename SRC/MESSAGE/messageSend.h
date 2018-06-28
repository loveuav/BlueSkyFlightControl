#ifndef _MESSAGESEND_H_
#define _MESSAGESEND_H_

#include "mathTool.h"

void BsklinkSendFlightData(uint8_t* sendFlag);
void BsklinkSendFlightStatus(uint8_t* sendFlag);
void BsklinkSendSensor(uint8_t* sendFlag);
void BsklinkSendGps(uint8_t* sendFlag);
void BsklinkSendRcData(uint8_t* sendFlag);
void BsklinkSendPidAtt(uint8_t* sendFlag);
void BsklinkSendPidPos(uint8_t* sendFlag);

#endif


