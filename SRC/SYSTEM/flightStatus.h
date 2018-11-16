#ifndef __FLIGHTSTATUS_H__
#define __FLIGHTSTATUS_H__

#include "mathTool.h"

//飞行状态
enum
{
    STANDBY,		        //待机
    TAKE_OFF,			    //起飞
    IN_AIR,				    //在空中
    LANDING,			    //降落
    FINISH_LANDING	        //降落完成
};

//初始化状态
enum
{
    HEATING,		        //加热中
    HEAT_FINISH,		    //加热完成
    INIT_FINISH             //初始化完成 （完成加速度零偏计算）
};

//放置状态
enum
{
    STATIC,		            //静止
    MOTIONAL			    //运动
};

//电机锁定状态
enum
{
    DISARMED,	            //上锁
    ARMED				    //解锁
};

//水平方向控制状态
enum
{
    POS_HOLD,			    //悬停
    POS_CHANGED,			//飞行
    POS_BRAKE,				//刹车
    POS_BRAKE_FINISH	    //刹车完成
};

//垂直方向控制状态
enum
{
    ALT_HOLD,			    //悬停
    ALT_CHANGED,            //高度改变
    ALT_CHANGED_FINISH      //高度改变完成
};

//飞行模式
enum
{
    MANUAL = 0,			    //手动		(不带定高不带定点)
    SEMIAUTO,				//半自动 	(带定高不带定点)
    AUTO,					//自动		(带定高带定点)
    SPORT,                  //运动模式
    COMMAND,                //命令模式  (用于第三方控制)
    AUTOTAKEOFF,		    //自动起飞
    AUTOLAND,				//自动降落
    RETURNTOHOME,		    //自动返航
    AUTOCIRCLE,			    //自动绕圈
    AUTOPILOT,			    //自动航线
    FOLLOWME				//自动跟随
};

void SystemInitCheck(void);

void SetAltControlStatus(uint8_t status);
uint8_t GetAltControlStatus(void);
void SetPosControlStatus(uint8_t status);
uint8_t GetPosControlStatus(void);

bool SetArmedStatus(uint8_t status);
uint8_t GetArmedStatus(void);

void PlaceStausCheck(Vector3f_t gyro);
uint8_t GetPlaceStatus(void);

void SetInitStatus(uint8_t status);
uint8_t GetInitStatus(void);
uint32_t GetInitFinishTime(void);

void SetFlightStatus(uint8_t status);
uint8_t GetFlightStatus(void);

void SetFlightMode(uint8_t mode);
uint8_t GetFlightMode(void);

void WindEstimate(void);
float GetWindSpeed(void);
float GetWindSpeedAcc(void);

void SetFailSafeStatus(bool status);
bool GetFailSafeStatus(void);

#endif



