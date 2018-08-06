#ifndef _MAVLINKSEND_H_
#define _MAVLINKSEND_H_

#include "mathTool.h"
#include "mavlinkNotice.h"

void MavlinkSendHeartbeat(uint8_t* sendFlag);
void MavlinkSendSysStatus(uint8_t* sendFlag);
void MavlinkSendParamValue(uint8_t* sendFlag);
void MavlinkSendGpsRawInt(uint8_t* sendFlag);
void MavlinkSendScaledImu(uint8_t* sendFlag);
void MavlinkSendAttitude(uint8_t* sendFlag);
void MavlinkSendLocalPositionNed(uint8_t* sendFlag);
void MavlinkSendRcChannels(uint8_t* sendFlag);
void MavlinkSendCommandAck(uint8_t* sendFlag);
void MavlinkSendMissionRequest(uint8_t* sendFlag);
void MavlinkSendMissionAck(uint8_t* sendFlag);
void MavlinkSendMissionCount(uint8_t* sendFlag);
void MavlinkSendMissionItem(uint8_t* sendFlag);
void MavlinkSendHomePosition(uint8_t* sendFlag);
void MavlinkSendVfrHud(uint8_t* sendFlag);
void MavlinkSendStatusText(uint8_t* sendFlag);

void MavlinkCurrentParamSet(uint16_t num);
void MavlinkSetCommandAck(uint16_t command, uint8_t result);
bool MavStatusTextSendCheck(void);
void MavlinkSendNoticeEnable(uint16_t noticeNum);
void MavlinkSendNotice(uint16_t noticeNum);
void MavlinkSendNoticeProgress(uint8_t progress);

#endif




