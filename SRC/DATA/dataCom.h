#ifndef _DATACOM_H_
#define _DATACOM_H_

#include "mathTool.h"

typedef union   
{  
    int16_t i16;  
    int32_t i32;    
    float   f32;
    uint8_t byte[4];  
}DATA_TYPE_t;

void DataSendLoop(void);

#endif


