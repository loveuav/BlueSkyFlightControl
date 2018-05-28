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

#define motorType quadX 

static const MOTOR_TYPE_t quadX = {             //四轴X型
    .motorNum   = 4,                            //电机数量
    .motorMixer = 
    {
        { 1.0f, -1.0f,  1.0f, -1.0f },          //后右
        { 1.0f, -1.0f, -1.0f,  1.0f },          //前右
        { 1.0f,  1.0f,  1.0f,  1.0f },          //后左
        { 1.0f,  1.0f, -1.0f, -1.0f },          //前左
    }
};

static const MOTOR_TYPE_t hex6X = {             //六轴X型
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

/**********************************************************************************************************
*函 数 名: MotorControl
*功能说明: 电机控制
*形    参: 横滚控制量 俯仰控制量 偏航控制量 油门控制量
*返 回 值: 无
**********************************************************************************************************/
void MotorControl(int16_t roll, int16_t pitch, int16_t yaw, int16_t throttle)
{
    int16_t motorPWM[8];
    int16_t maxMotorValue;
    
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
    
    //判断飞控锁定状态，并输出电机控制量
    if(GetArmedStatus() == ARMED)
    {
        for (u8 i=0; i<motorType.motorNum; i++)
        {
            MotorPWMSet(i+1, motorPWM[i]);
        }   
    }    
    else
    {
        for (u8 i=0; i<motorType.motorNum; i++)
        {
            MotorPWMSet(i+1, 0);
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



