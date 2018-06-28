#ifndef _MESSAGE_H_
#define _MESSAGE_H_

#include "mathTool.h"

#define MAX_SEND_FREQ   100         //最大发送频率 单位:Hz

typedef union   
{  
    int8_t  i8; 
    int16_t i16;  
    int32_t i32;    
    float   f32;
    uint8_t byte[4];  
}DATA_TYPE_t;

void MessageInit(void);
void MessageSendLoop(void);
void DataSend(uint8_t *data , uint8_t length);

#endif


