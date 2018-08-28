/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     motor.c
 * @说明     电机动力分配
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "motor.h"
#include "drv_pwm.h"
#include "flightStatus.h"

//油门行程为[0:2000]
#define MINTHROTTLE	    200                     //最小油门值           
#define MAXTHROTTLE 	1800                    //最大油门值

//机型选择
#define motorType quadX

static int16_t motorPWM[8];

//四轴X型
const MOTOR_TYPE_t quadX = 
{                 
    .motorNum   = 4,                            //电机数量
    .motorMixer = 
    {
        { 1.0f, -1.0f,  1.0f, -1.0f },          //后右
        { 1.0f, -1.0f, -1.0f,  1.0f },          //前右
        { 1.0f,  1.0f,  1.0f,  1.0f },          //后左
        { 1.0f,  1.0f, -1.0f, -1.0f },          //前左
    }
};

//六轴X型
const MOTOR_TYPE_t hex6X = 
{                    
    .motorNum   = 6,                            //电机数量
    .motorMixer = 
    {    
        { 1.0f, -0.5f,  0.866025f,  1.0f },     //后右
        { 1.0f, -0.5f, -0.866025f,  1.0f },     //前右
        { 1.0f,  0.5f,  0.866025f, -1.0f },     //后左
        { 1.0f,  0.5f, -0.866025f, -1.0f },     //前左
        { 1.0f, -1.0f,  0.0f,      -1.0f },     //右
        { 1.0f,  1.0f,  0.0f,       1.0f },     //左
    }   
};

//八轴X型
const MOTOR_TYPE_t octoFlatX = 
{                    
    .motorNum   = 8,                            //电机数量
    .motorMixer = 
    {    
        { 1.0f,  1.0f, -0.5f,  1.0f },          //中前左
        { 1.0f, -0.5f, -1.0f,  1.0f },          //前右
        { 1.0f, -1.0f,  0.5f,  1.0f },          //中后右
        { 1.0f,  0.5f,  1.0f,  1.0f },          //后左
        { 1.0f,  0.5f, -1.0f, -1.0f },          //前左
        { 1.0f, -1.0f, -0.5f, -1.0f },          //中前右
        { 1.0f, -0.5f,  1.0f, -1.0f },          //后右
        { 1.0f,  1.0f,  0.5f, -1.0f },          //中后左
    }   
};

/**********************************************************************************************************
*函 数 名: MotorControl
*功能说明: 电机控制
*形    参: 横滚控制量 俯仰控制量 偏航控制量 油门控制量
*返 回 值: 无
**********************************************************************************************************/
void MotorControl(int16_t roll, int16_t pitch, int16_t yaw, int16_t throttle)
{
    int16_t maxMotorValue;
    uint8_t escCaliFlag = 0;
    static int16_t motorResetPWM[8];
    
    //电机动力分配
    for(u8 i=0; i<motorType.motorNum; i++)
    {
        motorPWM[i] = throttle * motorType.motorMixer[i].throttle + 
                      roll     * motorType.motorMixer[i].roll     + 
                      pitch    * motorType.motorMixer[i].pitch    + 
                      yaw      * motorType.motorMixer[i].yaw;
    }

    //防止电机输出饱和
	maxMotorValue = motorPWM[0];
	for (u8 i=1; i<motorType.motorNum; i++)
	{
		if(motorPWM[i] > maxMotorValue)
            maxMotorValue = motorPWM[i];				
	} 	
	for (u8 i=0; i<motorType.motorNum; i++) 
	{
		if (maxMotorValue > MAXTHROTTLE)    
			motorPWM[i] -= maxMotorValue - MAXTHROTTLE;		
        //限制电机输出的最大最小值
        motorPWM[i] = ConstrainInt16(motorPWM[i], MINTHROTTLE, MAXTHROTTLE);
	}
    
    //电调校准
    if(escCaliFlag)
    {
        if(GetSysTimeMs() < 6000)
        {
            for (u8 i=0; i<motorType.motorNum; i++)
            {
                motorPWM[i] = 2000;				
            } 	
        }
        else
        {
            escCaliFlag = 0;
        }
    }
    
    //判断飞控锁定状态，并输出电机控制量
    if(GetArmedStatus() == ARMED || escCaliFlag)
    {
        for (u8 i=0; i<motorType.motorNum; i++)
        {
            //输出PWM
            MotorPWMSet(i+1, motorPWM[i]);
            //记录当前PWM值
            motorResetPWM[i] = motorPWM[i];
        }   
    }    
    else
    {
        for (u8 i=0; i<motorType.motorNum; i++)
        {
            //电机逐步减速，防止电机刹车引发射桨
            motorResetPWM[i] -= motorResetPWM[i] * 0.003f;
            MotorPWMSet(i+1, motorResetPWM[i]);
        }               
    }
}

/**********************************************************************************************************
*函 数 名: MotorStop
*功能说明: 所有电机停转
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MotorStop(void)
{
    for (u8 i=0; i<motorType.motorNum; i++)
    {
        MotorPWMSet(i+1, 0);
    }       
}

/**********************************************************************************************************
*函 数 名: GetMotorValue
*功能说明: 获取电机PWM值
*形    参: 无
*返 回 值: PWM值
**********************************************************************************************************/
int16_t* GetMotorValue(void)
{
    return motorPWM;
}

/**********************************************************************************************************
*函 数 名: GetMotorNum
*功能说明: 获取电机数量
*形    参: 无
*返 回 值: PWM值
**********************************************************************************************************/
int8_t GetMotorNum(void)
{
    return motorType.motorNum;
}
