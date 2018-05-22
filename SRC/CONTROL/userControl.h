#ifndef _USERCONTROL_H_
#define _USERCONTROL_H_

#include "mathTool.h"

typedef struct
{
    int16_t roll;   
    int16_t pitch;   
    int16_t yaw;      
    int16_t throttle;  
}RCDATA_t;

void UserControl(void);
    
#endif

