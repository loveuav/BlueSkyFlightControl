#ifndef _MESSAGE_H_
#define _MESSAGE_H_

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
    MSG_FLIGHT_DATA,
	MSG_FLIGHT_STATUS,
    MSG_SENSOR,
    MSG_RC,
    MSG_GPS,
    MSG_PID_ATT,
    MSG_PID_POS,
    MSG_NUM
};

void MessageInit(void);
void MessageSendLoop(void);
void DataSend(uint8_t *data , uint8_t length);

#endif


