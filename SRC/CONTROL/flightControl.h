#ifndef _FLIGHTCONTROL_H_
#define _FLIGHTCONTROL_H_

#include "mathTool.h"
#include "pid.h"

enum 
{
    ROLL_INNER,
    PITCH_INNER,
    YAW_INNER,
    ROLL_OUTER,
    PITCH_OUTER,
    YAW_OUTER,
    VEL_X,
    VEL_Y,
    VEL_Z,	
    POS_X,
    POS_Y,
    POS_Z,
    PIDNUM
};

typedef struct
{
    int16_t roll;   
    int16_t pitch;   
    int16_t yaw;      
    int16_t throttle;  
}RCTARGET_t;

typedef struct
{
	PID_t      pid[PIDNUM];
	
    RCTARGET_t rcTarget;
    
	Vector3f_t attInnerTarget;
    Vector3f_t attOuterTarget;
 	Vector3f_t posInnerTarget;   
 	Vector3f_t posOuterTarget;  
	
	int32_t    heightLimit;
	
}FLIGHTCONTROL_t;

void SetRcTarget(int16_t roll, int16_t pitch, int16_t yaw, int16_t throttle);
void FlightControlInnerLoop(Vector3f_t gyro);
void AttitudeOuterControl(void);
void AltitudeOuterControl(void);
void PositionInnerControl(void);

#endif



