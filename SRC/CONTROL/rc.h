#ifndef _RC_H_
#define _RC_H_

#include "mathTool.h"

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

#endif



