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

#define MINCHECK    200
#define MAXCHECK    800

RCDATA_t rcData;
RCCOMMAND_t rcCommand;

/**********************************************************************************************************
*函 数 名: RcCommandUpdate
*功能说明: 摇杆控制命令更新
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void RcCommandUpdate(void)
{
    //飞机未离地时，不计算除了油门之外的摇杆命令
    if(GetFlightStatus() >= IN_AIR)
    {
        rcCommand.roll     = rcData.roll - 500;
        rcCommand.pitch    = rcData.pitch - 500;
        rcCommand.yaw      = rcData.yaw - 500;
        rcCommand.throttle = rcData.throttle - 500;  
    }   
    else
    {
        rcCommand.roll     = 0;
        rcCommand.pitch    = 0;
        rcCommand.yaw      = 0;
        rcCommand.throttle = rcData.throttle - 500;         
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
void RcCheckSticks(void)
{
    static uint32_t armedCheckTime = 0;
    
    //摇杆外八字解锁
	if((rcData.roll > MAXCHECK) && (rcData.pitch < MINCHECK) &&
       (rcData.yaw < MINCHECK) && (rcData.throttle < MINCHECK))
    {
        if(GetArmedStatus() == DISARMED)
        {
            //持续1.5秒后解锁
            if(GetSysTimeMs() - armedCheckTime > 1500)
            {
                
            }
        }
        else
        {
            
        }
    }        
    else
    {
        armedCheckTime = GetSysTimeMs();
    }
}





















