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
    float roll;   
    float pitch;   
    float yaw;      
    float throttle;  
}RCTARGET_t;

typedef struct
{
	PID_t      pid[PIDNUM];
	
    RCTARGET_t rcTarget;
    Vector3f_t angleLpf;
    
	Vector3f_t attInnerTarget;
    Vector3f_t attOuterTarget;
 	Vector3f_t posInnerTarget;   
 	Vector3f_t posOuterTarget;  

    Vector3f_t attInnerError;
    Vector3f_t attOuterError;
    Vector3f_t posInnerError;
 	Vector3f_t posOuterError;  
    
    uint8_t    altCtlFlag;
    uint8_t    posCtlFlag;
    uint8_t    yawHoldFlag;
}FLIGHTCONTROL_t;

void FlightControlInit(void);

void SetRcTarget(RCTARGET_t rcTarget);
void FlightControlInnerLoop(Vector3f_t gyro);
void AttitudeOuterControl(void);
void AltitudeOuterControl(void);
void PositionInnerControl(void);
void PositionOuterControl(void);

void SetYawCtlTarget(float target);

void SetAltInnerCtlTarget(float target);
void SetAltOuterCtlTarget(float target);
void SetPosInnerCtlTarget(Vector3f_t target);
void SetPosOuterCtlTarget(Vector3f_t target);

void SetAltCtlStatus(uint8_t status);
void SetPosCtlStatus(uint8_t status);
void SetYawHoldStatus(uint8_t status);

Vector3f_t GetAttInnerCtlError(void);
Vector3f_t GetAttOuterCtlError(void);
Vector3f_t GetPosInnerCtlError(void);
Vector3f_t GetPosOuterCtlError(void);

void FlightControlReset(void);

extern FLIGHTCONTROL_t fc;

#endif



