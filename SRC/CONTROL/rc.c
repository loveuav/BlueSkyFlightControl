/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     rc.c
 * @说明     遥控数据处理
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "rc.h"
#include "flightStatus.h"
#include "board.h"
#include "drv_sbus.h"
#include "magnetometer.h"

#define MINCHECK    1200
#define MAXCHECK    1800

RCDATA_t rcData;
RCCOMMAND_t rcCommand;
uint32_t failsafeTime = 0;

static void RcDataUpdate(RCDATA_t data);
static void RcCommandUpdate(void);
static void RcCheckSticks(void);
static void RcCheckAux(void);
static void RcCheckFailsafe(void);

/**********************************************************************************************************
*函 数 名: RcInit
*功能说明: 遥控相关功能初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void RcInit(void)
{
	#if( RC_PROTOCOL == SBUS )
		Sbus_SetRcDataCallback(RcDataUpdate);
	#elif ( RC_PROTOCOL == PPM )
	//·········
	#endif
}

/**********************************************************************************************************
*函 数 名: RcCheck
*功能说明: 遥控器各项数据以及失控检测
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void RcCheck(void)
{
	RcCheckSticks();
	RcCheckAux();
	RcCheckFailsafe();	
}

/**********************************************************************************************************
*函 数 名: RcDataUpdate
*功能说明: 摇杆数据更新
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void RcDataUpdate(RCDATA_t data)
{
	//获取摇杆数据
	rcData = data;
	
	//更新摇杆控制命令
	RcCommandUpdate();
    
    //更新时间，用于失控保护检测
    failsafeTime = GetSysTimeMs();
}

/**********************************************************************************************************
*函 数 名: RcCommandUpdate
*功能说明: 摇杆控制命令更新
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void RcCommandUpdate(void)
{
    //飞机未离地时，不计算除了油门之外的摇杆命令
    if(GetFlightStatus() >= IN_AIR)
    {
        rcCommand.roll     = rcData.roll - 1500;
        rcCommand.pitch    = rcData.pitch - 1500;
        rcCommand.yaw      = rcData.yaw - 1500;
        rcCommand.throttle = rcData.throttle - 1500;  
    }   
    else
    {
        rcCommand.roll     = 0;
        rcCommand.pitch    = 0;
        rcCommand.yaw      = 0;
        rcCommand.throttle = rcData.throttle - 1500;         
    }
}

/**********************************************************************************************************
*函 数 名: GetRcCommad
*功能说明: 获取摇杆控制命令
*形    参: 无
*返 回 值: 摇杆命令
**********************************************************************************************************/
RCCOMMAND_t GetRcCommad(void)
{
    return rcCommand;
}

/**********************************************************************************************************
*函 数 名: RcCheckSticks
*功能说明: 检查摇杆位置，判断解锁动作等
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void RcCheckSticks(void)
{
    static uint32_t armedCheckTime = 0;
    static uint32_t armedDisarmedTime = 0;
    static uint32_t caliCheckTime = 0;
    static uint8_t  armedCheckFlag = 0;    
    
    //摇杆外八字解锁,同时也可上锁，即使在飞行中，也可通过外八强制上锁
	if((rcData.roll > MAXCHECK) && (rcData.pitch < MINCHECK) &&
       (rcData.yaw < MINCHECK) && (rcData.throttle < MINCHECK))
    {   
        //上锁3秒后才可再次解锁
        if(GetArmedStatus() == DISARMED && (GetSysTimeMs() - armedDisarmedTime) > 3000)
        {
            //持续1.5秒后解锁
            if(GetSysTimeMs() - armedCheckTime > 1500)
            {
                SetArmedStatus(ARMED);
                
                //解锁检查标志置1，用于判断摇杆是否已回中，防止解锁后因为摇杆位置没变化导致自动上锁
                armedCheckFlag = 1;   
            }
        }
        else
        {
            if(armedCheckFlag == 0)
            {
                //持续3秒后强制上锁
                if(GetSysTimeMs() - armedCheckTime > 3000)
                {
                    SetArmedStatus(DISARMED);
                    
                    //记录上锁时间
                    armedDisarmedTime = GetSysTimeMs();
                }                
            }
        }
    }        
    else
    {
        armedCheckTime = GetSysTimeMs();
    }
    
    //摇杆若回中，则重置解锁标志位，此时可以再次通过外八操作将飞机上锁 
    if(rcData.throttle > 1400)
        armedCheckFlag = 0; 
    
    //摇杆内八字持续5s，进入罗盘校准
	if((rcData.roll < MINCHECK) && (rcData.pitch < MINCHECK) &&
       (rcData.yaw > MAXCHECK) && (rcData.throttle < MINCHECK))
    {  
        //只在上锁状态下进行
        if(GetArmedStatus() == DISARMED)
        {
            if(GetSysTimeMs() - caliCheckTime > 5000)
            {
                //罗盘校准使能
                MagCalibrateEnable();
            }
        }
    }
    else
    {
        caliCheckTime = GetSysTimeMs();
    }
}

/**********************************************************************************************************
*函 数 名: RcCheckAux
*功能说明: 辅助通道检测
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void RcCheckAux(void)
{
	
}

/**********************************************************************************************************
*函 数 名: RcCheckFailsafe
*功能说明: 失控保护检测，主要分两种方式，一是接收不到遥控数据，二是遥控数据出现特定数值（遥控器上一般可设的失控保护）
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void RcCheckFailsafe(void)
{
    static uint16_t failsafeCnt = 0;
    uint8_t failsafeStatus[2];
    
	if(GetSysTimeMs() < 3000)
        return;
    
    //如果一定时间内接收不到遥控数据，则进入失控保护状态
    if(GetSysTimeMs() - failsafeTime > 1500)
    {
        SetFailSafeStatus(true);
        failsafeStatus[0] = 1;
    }
    else
    {
        failsafeStatus[0] = 0;
    }
    
    //遥控器的失控保护设置为：将辅助通道1的输出变为950
    if(rcData.aux1 > 920 && rcData.aux1 < 980)
    {
        if(failsafeCnt > 150)
        {
            SetFailSafeStatus(true);
        }
        else
        {
           failsafeCnt++; 
        }
        failsafeStatus[1] = 1;
    }
    else
    {
        failsafeStatus[1] = 0;
    }
    
    //失控条件均不成立时自动解除失控保护状态
    if(failsafeStatus[0] == 0 && failsafeStatus[1] == 0)
    {
        SetFailSafeStatus(false);
    }
}



