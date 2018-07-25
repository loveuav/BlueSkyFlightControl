#ifndef _MAVLINKSEND_H_
#define _MAVLINKSEND_H_

#include "mathTool.h"

void MavlinkSendHeartbeat(uint8_t* sendFlag);
void MavlinkSendSysStatus(uint8_t* sendFlag);
void MavlinkSendParamValue(uint8_t* sendFlag);
void MavlinkSendGpsRawInt(uint8_t* sendFlag);
void MavlinkSendScaledImu(uint8_t* sendFlag);
void MavlinkSendAttitude(uint8_t* sendFlag);
void MavlinkSendCommandAck(uint8_t* sendFlag);

void MavlinkCurrentParamSet(uint16_t num, uint8_t flag);
void MavlinkSetCommandAck(uint16_t command, uint8_t result);
#endif




