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

//放置状态
enum
{
	STATIC,		            //静止
	MOTIONAL,			    //运动
};

//电机锁定状态
enum
{
	DISARMED,	            //上锁
	ARMED,				    //解锁
};

//水平方向控制状态
enum 
{
    POS_HOLD,			    //悬停
    POS_CHANGED,			//飞行
    POS_BRAKE,				//刹车
    POS_BRAKE_FINISH,	    //刹车完成
};

//垂直方向控制状态
enum 
{
    ALT_HOLD			    //悬停
};

#endif



