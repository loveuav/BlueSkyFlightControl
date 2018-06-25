#ifndef _DATACOM_H_
#define _DATACOM_H_

#include "mathTool.h"

#define FRAME_HEAD_1    0xDE
#define FRAME_HEAD_2    0xED
#define DEVICE_TYPE     0x02

typedef union   
{  
    int8_t  i8; 
    int16_t i16;  
    int32_t i32;    
    float   f32;
    uint8_t byte[4];  
}DATA_TYPE_t;

enum
{
    FRAME_FLIGHT,
    FRAME_RC,
    FRAME_GPS,
    FRAME_PID_ATTINNER,
    FRAME_PID_ATTOUTER,
    FRAME_PID_POSINNER,
    FRAME_PID_POSOUTER,
    FRAME_NUM
};

void DataSendLoop(void);
void DataSend(uint8_t *data , uint8_t length);

#endif


