/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     userControl.c
 * @说明     用户操控逻辑处理，目前分为手动档、半自动档、自动档
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "userControl.h"
#include "flightControl.h"
#include "flightStatus.h"
#include "ahrs.h"
#include "navigation.h"
#include "board.h"

#define MAXANGLE  350
#define MAXRCDATA 450
#define ALT_SPEED_UP_MAX	500	//5m/s
#define ALT_SPEED_DOWN_MAX	300

static void ManualControl(RCDATA_t rcData, RCTARGET_t* rcTarget);
static void SemiAutoControl(RCDATA_t rcData, RCTARGET_t* rcTarget);
static void AutoControl(RCDATA_t rcData, RCTARGET_t* rcTarget);
static void YawControl(RCDATA_t rcData, RCTARGET_t* rcTarget);
static void AltControl(RCDATA_t rcData);
/**********************************************************************************************************
*函 数 名: UserControl
*功能说明: 用户控制模式下的操控逻辑处理
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void UserControl(void)
{
    uint8_t flightMode;
    RCDATA_t rcData;
    RCTARGET_t rcTarget;    
    
    //获取当前飞行模式
    flightMode = GetFlightMode();
    
    //获取摇杆数据
    //rcData = 
    
    if(flightMode == MANUAL)        
    {
        //手动档
        ManualControl(rcData, &rcTarget);
    }
    else if(flightMode ==SEMIAUTO)  
    {
        //半自动档
        SemiAutoControl(rcData, &rcTarget);        
    }
    else if(flightMode == AUTO)     
    {
        //自动档
        AutoControl(rcData, &rcTarget);       
    }

    //设置目标控制量
    SetRcTarget(rcTarget);    
}

/**********************************************************************************************************
*函 数 名: ManualControl
*功能说明: 手动档，飞机姿态与油门直接由摇杆量控制
*形    参: 摇杆量 控制目标量
*返 回 值: 无
**********************************************************************************************************/
static void ManualControl(RCDATA_t rcData, RCTARGET_t* rcTarget)
{
    static float rollRate  = (float)MAXANGLE / MAXRCDATA;
    static float pitchRate = (float)MAXANGLE / MAXRCDATA;   
    
    //将摇杆量转换为横滚俯仰的目标控制角度
    rcTarget->roll     = rcData.roll  * rollRate;
    rcTarget->pitch    = rcData.pitch * pitchRate;  

    //航向控制
    YawControl(rcData, rcTarget);
    
    //摇杆量直接转换为油门值
    rcTarget->throttle = rcData.throttle;
}

/**********************************************************************************************************
*函 数 名: SemiAutoControl
*功能说明: 半自动档，飞机姿态由摇杆量控制，高度自动控制
*形    参: 摇杆量 控制目标量
*返 回 值: 无
**********************************************************************************************************/
static void SemiAutoControl(RCDATA_t rcData, RCTARGET_t* rcTarget)
{
    static float rollRate  = (float)MAXANGLE / MAXRCDATA;
    static float pitchRate = (float)MAXANGLE / MAXRCDATA;   
    
    //将摇杆量转换为横滚俯仰的目标控制角度
    rcTarget->roll  = rcData.roll  * rollRate;
    rcTarget->pitch = rcData.pitch * pitchRate; 

    //航向控制
    YawControl(rcData, rcTarget);
    
    //高度控制
    AltControl(rcData);
}

/**********************************************************************************************************
*函 数 名: AutoControl
*功能说明: 自动档，该模式下摇杆量对应飞机飞行速度，回中时飞机自动悬停
*形    参: 摇杆量
*返 回 值: 无
**********************************************************************************************************/
static void AutoControl(RCDATA_t rcData, RCTARGET_t* rcTarget)
{
    //航向控制
    YawControl(rcData, rcTarget);
    
    //高度控制
    AltControl(rcData);  

    /**********************************************************************************************************
    位置控制：该模式下油门摇杆量控制飞行速度，回中时飞机自动悬停
    **********************************************************************************************************/    
}

/**********************************************************************************************************
*函 数 名: YawControl
*功能说明: 偏航控制
*形    参: 摇杆量 控制目标量
*返 回 值: 无
**********************************************************************************************************/
static void YawControl(RCDATA_t rcData, RCTARGET_t* rcTarget)
{
    static int16_t rcDeadband = 50;
    static float yawHold;
    
    //摇杆回中时锁定航向，摇杆量超过死区时，将摇杆量转换为目标控制角速度
    if(abs(rcData.yaw) > rcDeadband)
    {
        rcTarget->yaw = ApplyDeadbandInt(rcData.yaw, rcDeadband);
        
        //记录当前飞机航向角
        yawHold = GetCopterAngle().z;
    }
    else
    {
        //设置航向锁定目标角度
        SetYawCtlTarget(yawHold);
    }       
}

/**********************************************************************************************************
*函 数 名: AltControl
*功能说明: 高度控制
*形    参: 摇杆量
*返 回 值: 无
**********************************************************************************************************/
static void AltControl(RCDATA_t rcData)
{
    static int32_t lastTimeAltChanged = 0;
    static int16_t rcDeadband  = 50;
	static float speedUpRate   = (float)ALT_SPEED_UP_MAX / MAXRCDATA;
	static float speedDownRate = (float)ALT_SPEED_DOWN_MAX / MAXRCDATA;
    static uint8_t altHoldChanged = 0;
    static float velCtlTarget     = 0;
    static float altCtlTarget     = 0;    
    
    /**********************************************************************************************************
    高度控制：该模式下油门摇杆量控制上升下降速度，回中时飞机自动定高
    **********************************************************************************************************/
    if (abs(rcData.throttle) > rcDeadband)
    {	
        rcData.throttle = ApplyDeadbandInt(rcData.throttle, rcDeadband);
        
        //摇杆量转为目标速度，低通滤波改变操控手感
        if(rcData.throttle > 0)
        {
            velCtlTarget = velCtlTarget * 0.95f + (rcData.throttle * speedUpRate) * 0.05f;
        }
        else
            velCtlTarget = velCtlTarget * 0.95f + (rcData.throttle * speedDownRate) * 0.05f;
        
        //直接控制速度，禁用高度控制
        SetAltCtlStatus(DISABLE);
        
        //更新高度控制目标
        altCtlTarget = GetCopterPosition().z;
        
        //更新高度控制状态
        SetAltControlStatus(ALT_CHANGED);
        
        altHoldChanged = 1;
        lastTimeAltChanged = GetSysTimeMs();	
    }
    else if(altHoldChanged)
    {	
        //油门回中后先缓冲一段时间再进入定高
        if(GetSysTimeMs() - lastTimeAltChanged < 800)
        {
            velCtlTarget -= velCtlTarget * 0.08f;
        }
        else
        {
            altHoldChanged = 0;
        }	

        //更新高度控制目标
        altCtlTarget = GetCopterPosition().z;       

        //更新高度控制状态
        SetAltControlStatus(ALT_CHANGED_FINISH);        
    }
    else
    {       
        //使能高度控制
        SetAltCtlStatus(ENABLE);
        
        //更新高度控制状态
        SetAltControlStatus(ALT_HOLD);     
    }   

    //更新高度内环控制目标
    SetAltInnerCtlTarget(velCtlTarget); 
    //更新高度外环控制目标
    SetAltOuterCtlTarget(altCtlTarget);       
}



