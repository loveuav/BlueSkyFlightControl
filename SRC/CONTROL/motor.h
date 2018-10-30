#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "mathTool.h"

typedef struct
{
    float throttle;     //油门
    float roll;         //横滚
    float pitch;        //俯仰
    float yaw;          //偏航
} MOTOR_MIXER_t;

typedef struct
{
    int8_t        motorNum;
    MOTOR_MIXER_t motorMixer[8];
} MOTOR_TYPE_t;

void MotorInit(void);
void MotorControl(int16_t roll, int16_t pitch, int16_t yaw, int16_t throttle);
void EscCalibrateEnable(void);
void MotorStop(void);
int16_t* GetMotorValue(void);
int8_t GetMotorNum(void);

#endif







