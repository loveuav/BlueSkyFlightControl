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
	PID_t       pid[PIDNUM];
	
	Vector3f_t  attInnerTarget;
 	Vector3f_t  velInnerTarget;   
	Vector3f_t  attOuterTarget;
	
	Vector3f_t  posTarget;
	Vector3f_t  posInnerOutput;	
	Vector3f_t  posOuterOutput;
	
	int32_t     heightLimit;
	
}FLIGHTCONTROL_t;

void FlightControlInnerLoop(Vector3f_t gyro);
    
#endif



