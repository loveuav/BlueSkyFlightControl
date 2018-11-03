#ifndef _MESSAGE_H_
#define _MESSAGE_H_

#include "mathTool.h"
#include "bsklink.h"
#include "common/mavlink.h"

#define MAX_SEND_FREQ   100         //最大发送频率 单位:Hz

#define MAVLINK_SYSTEM_ID       1
#define MAVLINK_COMPONENT_ID    MAV_COMP_ID_AUTOPILOT1

typedef union
{
    int8_t  i8;
    int16_t i16;
    int32_t i32;
    float   f32;
    uint8_t byte[4];
} DATA_TYPE_t;

void MessageInit(void);
void MessageSendLoop(void);
void MessageSensorCaliFeedbackEnable(uint8_t type, uint8_t step, uint8_t success);
void BsklinkSetMsgFreq(BSKLINK_MSG_ID_FREQ_SETUP_t payload);
void BsklinkSendEnable(uint8_t msgid);
void MavlinkSendEnable(uint8_t msgid);
void DataSend(uint8_t *data, uint8_t length);
bool GetMessageStatus(void);

#endif


