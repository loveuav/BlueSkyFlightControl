/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     userControl.c
 * @说明     用户操控逻辑处理，目前分为手动档（自稳）、半自动档（定高）、自动档（定点）
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "userControl.h"
#include "rc.h"
#include "flightControl.h"
#include "flightStatus.h"
#include "ahrs.h"
#include "navigation.h"
#include "board.h"

#define ALT_SPEED_UP_MAX	500	    //最大上升速度：5m/s
#define ALT_SPEED_DOWN_MAX	300     //最大下降速度：3m/s
#define HORIZON_SPEED_MAX	800     //最大水平飞行速度：8m/s

static Vector3f_t posCtlTarget;
static float yawHold;

static void ManualControl(RCCOMMAND_t rcCommand, RCTARGET_t* rcTarget);
static void SemiAutoControl(RCCOMMAND_t rcCommand, RCTARGET_t* rcTarget);
static void AutoControl(RCCOMMAND_t rcCommand, RCTARGET_t* rcTarget);
static void YawControl(RCCOMMAND_t rcCommand, RCTARGET_t* rcTarget);
static void AltControl(RCCOMMAND_t rcCommand);
static void UpdateMaxBrakeAngle(Vector3f_t velocity);

/**********************************************************************************************************
*函 数 名: UserControl
*功能说明: 用户控制模式下的操控逻辑处理
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void UserControl(void)
{
    uint8_t flightMode;
    RCCOMMAND_t rcCommand;
    RCTARGET_t rcTarget;    
    static float rollRate  = (float)MAXANGLE / MAXRCDATA;
    static float pitchRate = (float)MAXANGLE / MAXRCDATA;   
    
    //获取当前飞行模式
    flightMode = GetFlightMode();
    
    //获取摇杆数据
    rcCommand = GetRcCommad();

    //通用控制部分，将摇杆量转换为横滚俯仰的目标控制角度
    rcTarget.roll  = rcCommand.roll  * rollRate;
    rcTarget.pitch = rcCommand.pitch * pitchRate;  

    if(flightMode == MANUAL)        
    {
        //手动档（自稳）
        ManualControl(rcCommand, &rcTarget);
    }
    else if(flightMode ==SEMIAUTO)  
    {
        //半自动档（定高）
        SemiAutoControl(rcCommand, &rcTarget);        
    }
    else if(flightMode == AUTO)     
    {
        //自动档（定点）
        AutoControl(rcCommand, &rcTarget);       
    }
    else
    {
        //在非用户控制模式下，更新当前位置目标
        posCtlTarget = GetCopterPosition();       

        yawHold = GetCopterAngle().z;        
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
static void ManualControl(RCCOMMAND_t rcCommand, RCTARGET_t* rcTarget)
{
    //航向控制
    YawControl(rcCommand, rcTarget);
    
    //摇杆量直接转换为油门值
    rcTarget->throttle = rcCommand.throttle;
}

/**********************************************************************************************************
*函 数 名: SemiAutoControl
*功能说明: 半自动档，飞机姿态由摇杆量控制，高度自动控制
*形    参: 摇杆量 控制目标量
*返 回 值: 无
**********************************************************************************************************/
static void SemiAutoControl(RCCOMMAND_t rcCommand, RCTARGET_t* rcTarget)
{
    //航向控制
    YawControl(rcCommand, rcTarget);
    
    //高度控制
    AltControl(rcCommand);

	//判断飞行状态
	if(abs(rcCommand.roll) > 50 || abs(rcCommand.pitch) > 50)
	{
        //更新位置控制状态
        SetPosControlStatus(POS_CHANGED);
	}
	else
	{
        //更新位置控制状态
        SetPosControlStatus(POS_HOLD);		
	}
}

/**********************************************************************************************************
*函 数 名: AutoControl
*功能说明: 自动档，该模式下摇杆量对应飞机飞行速度，回中时飞机自动悬停
*形    参: 摇杆量
*返 回 值: 无
**********************************************************************************************************/
static void AutoControl(RCCOMMAND_t rcCommand, RCTARGET_t* rcTarget)
{
    static Vector3f_t velCtlTarget, lastVelCtlTarget; 
    
    static int32_t lastTimePosChanged = 0;
    static int32_t lastTimePosBrake   = 0;
    static uint8_t posHoldChanged     = 0;
    
    const  int16_t rcDeadband         = 50;
	static float   velRate            = (float)HORIZON_SPEED_MAX / MAXRCDATA;
    static float   brakeFilter        = 0;
    const  float   velTargetIncMax    = 0.7;      //摇杆增速限幅值
    
    //航向控制
    YawControl(rcCommand, rcTarget);
    
    //高度控制
    AltControl(rcCommand);  

    /**********************************************************************************************************
    位置控制：该模式下摇杆量控制飞行速度，回中时飞机自动悬停
    **********************************************************************************************************/    
    if(abs(rcCommand.roll) > rcDeadband || abs(rcCommand.pitch) > rcDeadband)
    {
        rcCommand.roll  = ApplyDeadbandInt(rcCommand.roll, rcDeadband * 0.8f);
        rcCommand.pitch = ApplyDeadbandInt(rcCommand.pitch, rcDeadband * 0.8f);
        
        //摇杆量转为目标速度，低通滤波改变操作手感
        velCtlTarget.x = velCtlTarget.x * 0.996f + ((float)rcCommand.pitch * velRate) * 0.004f;
        velCtlTarget.y = velCtlTarget.y * 0.996f + ((float)rcCommand.roll * velRate) * 0.004f;
        
        //目标速度增量限幅，以实现近似匀加速的效果
        if(rcCommand.pitch > 0)
        {
            if(velCtlTarget.x - lastVelCtlTarget.x > velTargetIncMax)
                velCtlTarget.x = lastVelCtlTarget.x + velTargetIncMax;
        }
        else
        {
             if(velCtlTarget.x - lastVelCtlTarget.x < -velTargetIncMax)
                velCtlTarget.x = lastVelCtlTarget.x - velTargetIncMax;           
        }  
        if(rcCommand.roll > 0)
        {
            if(velCtlTarget.y - lastVelCtlTarget.y > velTargetIncMax)
                velCtlTarget.y = lastVelCtlTarget.y + velTargetIncMax;
        }
        else
        {
             if(velCtlTarget.y - lastVelCtlTarget.y < -velTargetIncMax)
                velCtlTarget.y = lastVelCtlTarget.y - velTargetIncMax;           
        }
        
        //保存本次速度目标值
        lastVelCtlTarget.x = velCtlTarget.x; 
        lastVelCtlTarget.y = velCtlTarget.y; 
    
        //直接控制速度，禁用位置控制
        SetPosCtlStatus(DISABLE);
        
         //更新位置内环控制目标    
        SetPosInnerCtlTarget(velCtlTarget);
        
        //更新位置控制目标
        posCtlTarget.x = GetCopterPosition().x;
        posCtlTarget.y = GetCopterPosition().y;

        //更新位置控制状态
        SetPosControlStatus(POS_CHANGED);
        
        posHoldChanged = 1;
        lastTimePosChanged = GetSysTimeMs();	
    }
    else if(posHoldChanged)
    {
        //重置历史速度目标值
        lastVelCtlTarget.x = 0;
        lastVelCtlTarget.y = 0;
        
        //进入刹车状态时先初始化目标速度
        if(GetPosControlStatus() == POS_CHANGED)
        {            
            //计算减速速度（斜率）
            brakeFilter = Pythagorous2(velCtlTarget.x, velCtlTarget.y) / (500 * 1.2f);  //500Hz运行频率，1.2s刹车时间
            brakeFilter = ConstrainFloat(brakeFilter, 0.3, 2);
            
            //更新位置控制状态为刹车
            SetPosControlStatus(POS_BRAKE);
            //根据当前飞行速度更新最大刹车角度
            UpdateMaxBrakeAngle(GetCopterVelocity());
        }
        else if(GetPosControlStatus() == POS_BRAKE)
        {    
            //匀减速刹车
            if(abs(velCtlTarget.x) > 0)
                velCtlTarget.x -= brakeFilter * (velCtlTarget.x / abs(velCtlTarget.x));
            if(abs(velCtlTarget.y) > 0)
                velCtlTarget.y -= brakeFilter * (velCtlTarget.y / abs(velCtlTarget.y));
	        
            //飞机速度小于一定值或超出一定时间则认为刹车完成
            if((abs(GetCopterVelocity().x) < 20 && abs(GetCopterVelocity().y) < 20) || GetSysTimeMs() - lastTimePosChanged > 3000)
            {   
                //更新位置控制状态为刹车完成
                SetPosControlStatus(POS_BRAKE_FINISH);
            }   

            lastTimePosBrake = GetSysTimeMs();	             
        }
        else if(GetPosControlStatus() == POS_BRAKE_FINISH)
        {
            //刹车完成后再缓冲一小段时间便切换为自动悬停
            if(GetSysTimeMs() - lastTimePosBrake < 1500)
            {
                velCtlTarget.x -= velCtlTarget.x * 0.03f;
                velCtlTarget.y -= velCtlTarget.y * 0.03f;
            }
            else
            {
                posHoldChanged = 0;
            }
        }
        
        //更新位置内环控制目标    
        SetPosInnerCtlTarget(velCtlTarget);
        
        //更新位置控制目标
        posCtlTarget.x = GetCopterPosition().x;
        posCtlTarget.y = GetCopterPosition().y;         
    }
    else
    {     
        //待机状态下不断刷新位置目标
        if(GetFlightStatus() == STANDBY)
        {
            posCtlTarget.x = GetCopterPosition().x;
            posCtlTarget.y = GetCopterPosition().y;         
        }
        
        //使能位置控制
        SetPosCtlStatus(ENABLE);
     
        //更新位置控制状态
        SetPosControlStatus(POS_HOLD);     
        
        //更新位置外环控制目标
        SetPosOuterCtlTarget(posCtlTarget);
        
        //设置位置环控制最大输出：cm/s
        SetMaxPosOuterCtl(150);          
    }     
}

/**********************************************************************************************************
*函 数 名: YawControl
*功能说明: 偏航控制
*形    参: 摇杆量 控制目标量
*返 回 值: 无
**********************************************************************************************************/
static void YawControl(RCCOMMAND_t rcCommand, RCTARGET_t* rcTarget)
{
    static int16_t rcDeadband = 80;
    static uint8_t yawHoldChanged = 0;
    static int32_t lastTimeyawChanged = 0;
    
    //起飞前初始化航向锁定目标
    if(GetFlightStatus() < TAKE_OFF)
    {
        yawHold = GetCopterAngle().z;        
    }
    
    //摇杆回中时锁定航向，摇杆量超过死区时，将摇杆量转换为目标控制角速度
    if(abs(rcCommand.yaw) > rcDeadband)
    {
        rcTarget->yaw = ApplyDeadbandInt(rcCommand.yaw, rcDeadband);
        
        //记录当前飞机航向角
        yawHold = GetCopterAngle().z;
        
        //失能航向锁定
        SetYawHoldStatus(DISABLE);
        
        yawHoldChanged = 1;
        lastTimeyawChanged = GetSysTimeMs();
    }
    else if(yawHoldChanged)
    {
        if(GetSysTimeMs() - lastTimeyawChanged > 80)
        {
            yawHoldChanged = 0;
        }
        else
        {
            rcTarget->yaw = 0;
            
            //记录当前飞机航向角
            yawHold = GetCopterAngle().z;            
        }
    }
    else
    {
        rcTarget->yaw = 0;
        
        //设置航向锁定目标角度
        SetYawCtlTarget(yawHold);
        
        //使能航向锁定
        SetYawHoldStatus(ENABLE);
    }       
}

/**********************************************************************************************************
*函 数 名: AltControl
*功能说明: 高度控制
*形    参: 摇杆量
*返 回 值: 无
**********************************************************************************************************/
static void AltControl(RCCOMMAND_t rcCommand)
{
    static int32_t lastTimeAltChanged = 0;
    static int16_t rcDeadband  = 100;
	static float speedUpRate   = (float)ALT_SPEED_UP_MAX / MAXRCDATA;
	static float speedDownRate = (float)ALT_SPEED_DOWN_MAX / MAXRCDATA;
    static uint8_t altHoldChanged = 0;
    static float velCtlTarget     = 0; 
    
    /**********************************************************************************************************
    高度控制：该模式下油门摇杆量控制上升下降速度，回中时飞机自动定高
    **********************************************************************************************************/
    rcCommand.throttle = (rcCommand.throttle - 1000) * 0.5f;
    
    if (abs(rcCommand.throttle) > rcDeadband)
    {	
        rcCommand.throttle = ApplyDeadbandInt((rcCommand.throttle), rcDeadband);
        
        //摇杆量转为目标速度，低通滤波改变操控手感
        if(rcCommand.throttle > 0)
        {
            velCtlTarget = velCtlTarget * 0.98f + (rcCommand.throttle * speedUpRate) * 0.02f;
        }
        else
            velCtlTarget = velCtlTarget * 0.98f + (rcCommand.throttle * speedDownRate) * 0.02f;
        
        //直接控制速度，禁用高度控制
        SetAltCtlStatus(DISABLE);
        
        //更新高度内环控制目标
        SetAltInnerCtlTarget(velCtlTarget); 
        
        //更新高度目标
        posCtlTarget.z = GetCopterPosition().z;
        
        //更新高度控制状态
        SetAltControlStatus(ALT_CHANGED);
        
        altHoldChanged = 1;
        lastTimeAltChanged = GetSysTimeMs();	
    }
    else if(altHoldChanged)
    {	
        if(GetAltControlStatus() == ALT_CHANGED)
        {            
            //更新高度控制状态
            SetAltControlStatus(ALT_CHANGED_FINISH);    
        }
        else
        {
            //油门回中后先缓冲一段时间再进入定高
            if(GetSysTimeMs() - lastTimeAltChanged < 1000)
            {
                velCtlTarget -= velCtlTarget * 0.03f;
            }
            else
            {
                altHoldChanged = 0;
            }	

            //更新高度目标
            posCtlTarget.z = GetCopterPosition().z;       
        }  

        //更新高度内环控制目标
        SetAltInnerCtlTarget(velCtlTarget);         
    }
    else
    {       
        //待机状态下不断刷新高度目标
        if(GetFlightStatus() == STANDBY)
        {
            posCtlTarget.z = GetCopterPosition().z;   
        }
        
        //使能高度控制
        SetAltCtlStatus(ENABLE);
        
        //更新高度控制状态
        SetAltControlStatus(ALT_HOLD);  

        //更新高度外环控制目标
        SetAltOuterCtlTarget(posCtlTarget.z);     

        //设置高度环控制最大输出：cm/s
        SetMaxAltOuterCtl(200);            
    }             
}

/**********************************************************************************************************
*函 数 名: UpdateMaxBrakeAngle
*功能说明: 更新最大刹车角度
*形    参: 飞行速度
*返 回 值: 无
**********************************************************************************************************/
static void UpdateMaxBrakeAngle(Vector3f_t velocity)
{
    int16_t velMag = Pythagorous2(velocity.x, velocity.y);
    int16_t maxAngle;
    
    if(velMag > 500)
    {
        maxAngle = 25 + (velMag - 500) / 50;
        maxAngle = ConstrainInt16(maxAngle, 25, 30);
    }
    else if(velMag > 300)
    {
        maxAngle = 25;
    }
    else
    {
        maxAngle = 20;
    }
      
    SetMaxBrakeAngle(maxAngle); 
}


