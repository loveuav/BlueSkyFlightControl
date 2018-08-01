#ifndef _FLIGHTCONTROL_H_
#define _FLIGHTCONTROL_H_

#include "mathTool.h"
#include "pid.h"

#define MAXANGLE  400               //最大飞行角度：40°
#define MAXRCDATA 450

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
	PID_t      pid[PIDNUM];         //PID参数结构体
	
    RCTARGET_t rcTarget;            //摇杆控制量
    Vector3f_t angleLpf;

    Vector3f_t attInnerCtlValue;    //姿态内环控制量
    float      altInnerCtlValue;    //高度内环控制量
    
	Vector3f_t attInnerTarget;      //姿态内环（角速度）控制目标
    Vector3f_t attOuterTarget;      //姿态外环（角度）控制目标
 	Vector3f_t posInnerTarget;      //位置内环（速度）控制目标
 	Vector3f_t posOuterTarget;      //位置外环（位置）控制目标

    Vector3f_t attInnerError;       //姿态内环（角速度）控制误差
    Vector3f_t attOuterError;       //姿态外环（角度）控制误差
    Vector3f_t posInnerError;       //位置内环（速度）控制误差
 	Vector3f_t posOuterError;       //位置外环（位置）控制误差  
    
    uint8_t    altCtlFlag;          //高度控制使能标志位
    uint8_t    posCtlFlag;          //位置控制使能标志位
    uint8_t    yawHoldFlag;         //航向锁定控制使能标志位
    
    int16_t    maxBrakeAngle;       //最大刹车角度
    int16_t    maxPosOuterCtl;      //位置控制的最大输出
    int16_t    maxAltOuterCtl;      //高度控制的最大输出
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
PID_t FcGetPID(uint8_t id);
void FcSetPID(uint8_t id, PID_t pid);
bool PIDReadFromFlash(void);

void SetMaxBrakeAngle(int16_t angle);
void SetMaxPosOuterCtl(int16_t vel);
void SetMaxAltOuterCtl(int16_t vel);

#endif



