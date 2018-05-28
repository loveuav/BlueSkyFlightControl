#ifndef _RC_H_
#define _RC_H_

#include "mathTool.h"

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
    int16_t roll;   
    int16_t pitch;   
    int16_t yaw;      
    int16_t throttle;  
}RCCOMMAND_t;

void RcInit(void);
void RcCheck(void);
RCCOMMAND_t GetRcCommad(void);

void FlightStatusUpdate(void);

#endif



