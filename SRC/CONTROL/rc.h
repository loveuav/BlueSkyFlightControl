#ifndef _RC_H_
#define _RC_H_

#include "mathTool.h"

typedef struct
{
    int16_t roll;   
    int16_t pitch;   
    int16_t yaw;      
    int16_t throttle;  
    int16_t aux1;  
    int16_t aux2;  
    int16_t aux3;  
    int16_t aux4;  
    int16_t aux5;  
    int16_t aux6;   
    int16_t aux7;  
    int16_t aux8;  
    int16_t aux9;  
    int16_t aux10;     
}RCDATA_t;

typedef struct
{
    int16_t roll;   
    int16_t pitch;   
    int16_t yaw;      
    int16_t throttle;  
}RCCOMMAND_t;

RCCOMMAND_t GetRcCommad(void);

#endif



