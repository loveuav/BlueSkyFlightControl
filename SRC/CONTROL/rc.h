#ifndef _RC_H_
#define _RC_H_

#include "mathTool.h"
#include "board.h"

enum
{
    AUX1,
    AUX2,
    AUX3,
    AUX4,
    AUX5,
    AUX6,
    AUX7,
    AUX8,
    AUX9,
    AUX10,
    AUX11,
    AUX12
};

enum
{
    LOW,
    MID,
    HIGH
};

enum
{
    ROLL,
    PITCH,
    YAW,
    THROTTLE
};

typedef struct
{
    int16_t roll;       //横滚
    int16_t pitch;      //俯仰
    int16_t yaw;        //偏航
    int16_t throttle;   //油门
} RCCOMMAND_t;

void RcInit(void);
void RcCheck(void);
RCDATA_t GetRcData(void);
RCCOMMAND_t GetRcCommad(void);

void FlightStatusUpdate(void);

#endif



