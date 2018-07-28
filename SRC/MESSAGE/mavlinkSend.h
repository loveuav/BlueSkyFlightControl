#ifndef _MAVLINKSEND_H_
#define _MAVLINKSEND_H_

#include "mathTool.h"

enum
{
    CAL_START_GYRO,
    CAL_START_ACC,
    CAL_START_MAG,
    CAL_START_LEVEL,
    CAL_DONE,
    CAL_FAILED,
    CAL_PROGRESS_0,
    CAL_PROGRESS_10,
    CAL_PROGRESS_20,
    CAL_PROGRESS_30,
    CAL_PROGRESS_40,
    CAL_PROGRESS_50,
    CAL_PROGRESS_60,
    CAL_PROGRESS_70,
    CAL_PROGRESS_80,
    CAL_PROGRESS_90,
    CAL_PROGRESS_100,
    CAL_UP_DETECTED,
    CAL_DOWN_DETECTED,
    CAL_LEFT_DETECTED,
    CAL_RIGHT_DETECTED,
    CAL_FRONT_DETECTED,
    CAL_BACK_DETECTED,
    CAL_UP_DONE,
    CAL_DOWN_DONE,
    CAL_LEFT_DONE,
    CAL_RIGHT_DONE,
    CAL_FRONT_DONE,
    CAL_BACK_DONE,
    STATUS_TEXT_NUM
};

void MavlinkSendHeartbeat(uint8_t* sendFlag);
void MavlinkSendSysStatus(uint8_t* sendFlag);
void MavlinkSendParamValue(uint8_t* sendFlag);
void MavlinkSendGpsRawInt(uint8_t* sendFlag);
void MavlinkSendScaledImu(uint8_t* sendFlag);
void MavlinkSendAttitude(uint8_t* sendFlag);
void MavlinkSendCommandAck(uint8_t* sendFlag);
void MavlinkSendMissionRequest(uint8_t* sendFlag);
void MavlinkSendMissionAck(uint8_t* sendFlag);
void MavlinkSendMissionCount(uint8_t* sendFlag);
void MavlinkSendMissionItem(uint8_t* sendFlag);
void MavlinkSendStatusText(uint8_t* sendFlag);

void MavlinkCurrentParamSet(uint16_t num);
void MavlinkSetCommandAck(uint16_t command, uint8_t result);
bool MavStatusTextSendCheck(void);
void MavlinkSendNoticeEnable(uint16_t noticeNum);
void MavlinkSendNotice(uint16_t noticeNum);
void MavlinkSendNoticeProgress(uint8_t progress);

#endif




