#ifndef _BSKLINKSEND_H_
#define _BSKLINKSEND_H_

#include "mathTool.h"

void BsklinkSendFlightData(uint8_t* sendFlag);
void BsklinkSendFlightStatus(uint8_t* sendFlag);
void BsklinkSendSensor(uint8_t* sendFlag);
void BsklinkSendSensorCaliData(uint8_t* sendFlag);
void BsklinkSendSensorCaliCmd(uint8_t* sendFlag, uint8_t type, uint8_t step, uint8_t success);
void BsklinkSendGps(uint8_t* sendFlag);
void BsklinkSendRcData(uint8_t* sendFlag);
void BsklinkSendMotor(uint8_t* sendFlag);
void BsklinkSendPidAtt(uint8_t* sendFlag);
void BsklinkSendPidPos(uint8_t* sendFlag);
void BsklinkSendHeartBeat(uint8_t* sendFlag);

#endif


