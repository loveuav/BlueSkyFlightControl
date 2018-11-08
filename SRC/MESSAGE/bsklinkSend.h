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
void BsklinkSendBattery(uint8_t* sendFlag);
void BsklinkSendPidAtt(uint8_t* sendFlag);
void BsklinkSendPidPos(uint8_t* sendFlag);
void BsklinkSendPidAck(uint8_t* sendFlag);
void BsklinkSetPidAck(uint8_t ack);
void BsklinkSendSysError(uint8_t* sendFlag);
void BsklinkSendSysWarning(uint8_t* sendFlag);
void BsklinkSendAttAnalyse(uint8_t* sendFlag);
void BsklinkSendVelAnalyse(uint8_t* sendFlag);
void BsklinkSendPosAnalyse(uint8_t* sendFlag);
void BsklinkSendUserDefine(uint8_t* sendFlag);
void BsklinkSendFreqSetup(uint8_t* sendFlag);
void BsklinkSendHeartBeat(uint8_t* sendFlag);

#endif


