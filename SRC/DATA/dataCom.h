#ifndef _DATACOM_H_
#define _DATACOM_H_

#include "mathTool.h"

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
    MSG_FLIGHT,
    MSG_IMU_SENSOR,
    MSG_RC,
    MSG_GPS,
    MSG_PID_ATTINNER,
    MSG_PID_ATTOUTER,
    MSG_PID_POSINNER,
    MSG_PID_POSOUTER,
    MSG_NUM
};

void DataCommunicationInit(void);
void DataSendLoop(void);
void DataSend(uint8_t *data , uint8_t length);

#endif


