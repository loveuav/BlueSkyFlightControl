#ifndef _DATACOM_H_
#define _DATACOM_H_

#include "mathTool.h"

#define FRAME_HEAD_1    0xDE
#define FRAME_HEAD_2    0xED
#define DEVICE_TYPE     0x02

typedef union   
{  
    int16_t i16;  
    int32_t i32;    
    float   f32;
    uint8_t byte[4];  
}DATA_TYPE_t;

void DataSendLoop(void);
void DataSend(uint8_t *data , uint8_t length);

#endif


