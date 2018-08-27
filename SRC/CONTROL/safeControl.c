/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     safeControl.c
 * @说明     安全保护相关的控制功能
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.08 
**********************************************************************************************************/
#include "safeControl.h"
#include "flightStatus.h"
#include "navigation.h"
#include "ahrs.h"
#include "gps.h"
#include "battery.h"
#include "board.h"

static void FailSafeProtect(void);
static void LowPowerProtect(void);
static void CrashProtect(void);

/**********************************************************************************************************
*函 数 名: SafeControl
*功能说明: 安全保护控制
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void SafeControl(void)
{
    //失控保护
    FailSafeProtect();  
    
    //低电量/电压保护
    LowPowerProtect();
    
    //炸机保护
    CrashProtect();
}

/**********************************************************************************************************
*函 数 名: FailSafeProtect
*功能说明: 失控保护
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void FailSafeProtect(void)
{
    if(GetArmedStatus() == DISARMED)
        return;
    
    //失控状态下触发自动返航
    if(GetFailSafeStatus() == true && GetFlightMode() != RETURNTOHOME)
    {
        if(GetFlightMode() == AUTOLAND)
        {
            //当飞机处于自动降落模式时，若距离过远，也切换为自动返航模式
            if(GpsGetFixStatus() == true && GetDistanceToHome(GetCopterPosition()) > 5000)
                SetFlightMode(RETURNTOHOME);
        }
        else
        {
            SetFlightMode(RETURNTOHOME);
        }
    }
}

/**********************************************************************************************************
*函 数 名: LowPowerProtect
*功能说明: 低电量/电压保护
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void LowPowerProtect(void)
{
    static uint32_t lastCheckTime;
    const  uint32_t TIME_INTERVAL = 30000;
    
    if(GetArmedStatus() == DISARMED)
        return;
        
    //严重低电量时直接降落
    if(GetBatteryStatus() == BATTERY_CRITICAL_LOW)
    {
        SetFlightMode(AUTOLAND);
    }
    //普通低电量时按照一定时间间隔触发返航
    else if(GetBatteryStatus() == BATTERY_LOW)
    {
        if(GetSysTimeMs() - lastCheckTime > TIME_INTERVAL)
        {
            lastCheckTime = GetSysTimeMs();
            if(GetFlightMode() != RETURNTOHOME)
            {
                if(GetFlightMode() == AUTOLAND)
                {
                    //当飞机处于自动降落模式时，若距离过远，也切换为自动返航模式
                    if(GpsGetFixStatus() == true && GetDistanceToHome(GetCopterPosition()) > 5000)
                    SetFlightMode(RETURNTOHOME);
                }
                else
                {
                    SetFlightMode(RETURNTOHOME);
                }
            }
        } 
    }        
}

/**********************************************************************************************************
*函 数 名: CrashProtect
*功能说明: 炸机保护
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void CrashProtect(void)
{
    
}


