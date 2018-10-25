/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     waypointControl.c
 * @说明     航点飞行
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.07
**********************************************************************************************************/
#include "waypointControl.h"
#include "flightStatus.h"
#include "flightControl.h"
#include "board.h"
#include "navigation.h"
#include "gps.h"

/*
航点飞行中的航向控制模式
0：航向不变
1：始终朝向下一个航点
*/
const uint8_t WP_YAW_MODE = 1;

mavlink_mission_item_t wpItem[0xFF];    //航点信息

uint16_t wpCount = 0;                   //航点数量
uint16_t wpCurrentCount = 0;            //当前正在执行的航点序号
uint16_t wpRecvCount = 0;               //航点接收序号
uint16_t wpSendCount = 0;               //航点发送序号

uint8_t firstPointArriveFlag = 0;

/**********************************************************************************************************
*函 数 名: WaypointControl
*功能说明: 自动航线飞行
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void WaypointControl(void)
{
    static uint8_t wpStep = 0;
    //static Vector3f_t currentPointPos, nextPointPos;
    //float disToNextPoint;
    static float directToNextPoint;

    if(GetArmedStatus() == DISARMED)
        return;

//    if(firstPointArriveFlag == 0)
//        disToNextPoint = Pythagorous2(GetCopterPosition().x - wpPosition[0].x, GetCopterPosition().y - wpPosition[0].y);
//    else
//        disToNextPoint = Pythagorous2(wpPosition[wpCurrentCount].x - wpPosition[wpCurrentCount+1].x,
//                         wpPosition[wpCurrentCount].y - wpPosition[wpCurrentCount+1].y);

    switch(wpStep)
    {
    case 0:
        //判断命令类型，如果是航点命令则进入下一步，其它命令暂时先忽略
        if(wpItem[wpCurrentCount].command == MAV_CMD_NAV_WAYPOINT)
        {
            wpStep++;
        }
        else
        {
            wpCurrentCount++;
        }
        break;

    case 1:
        //计算下一个航点的方向
//            if(firstPointArriveFlag == 0)
//                directToNextPoint = GetDirectionOfTwoPoint(GetCopterPosition(), point2);

        if(WP_YAW_MODE)
        {
            //设置机头朝向下一个航点
            SetYawCtlTarget(directToNextPoint);
        }

        wpStep++;
        break;

    case 2:
        break;

    default:
        break;
    }

    //使能高度控制
    SetAltCtlStatus(ENABLE);
    //更新高度控制状态
    SetAltControlStatus(ALT_HOLD);
    //使能位置控制
    SetPosCtlStatus(ENABLE);
    //更新位置控制状态
    SetPosControlStatus(POS_HOLD);
    //使能航向锁定
    SetYawHoldStatus(ENABLE);
}

/**********************************************************************************************************
*函 数 名: GetWaypointCount
*功能说明: 获取航点数量
*形    参: 无
*返 回 值: 航点数量
**********************************************************************************************************/
uint16_t GetWaypointCount(void)
{
    return wpCount;
}

/**********************************************************************************************************
*函 数 名: SetWaypointCount
*功能说明: 设置航点数量
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void SetWaypointCount(uint16_t count)
{
    wpCount = count;
}

/**********************************************************************************************************
*函 数 名: GetWaypointCount
*功能说明: 获取航点接收序号
*形    参: 无
*返 回 值: 航点序号
**********************************************************************************************************/
uint16_t GetWaypointRecvCount(void)
{
    return wpRecvCount;
}

/**********************************************************************************************************
*函 数 名: SetWaypointRecvCount
*功能说明: 设置航点接收序号
*形    参: 航点序号
*返 回 值: 无
**********************************************************************************************************/
void SetWaypointRecvCount(uint16_t count)
{
    wpRecvCount = count;
}

/**********************************************************************************************************
*函 数 名: GetWaypointSendCount
*功能说明: 获取航点发送序号
*形    参: 无
*返 回 值: 航点序号
**********************************************************************************************************/
uint16_t GetWaypointSendCount(void)
{
    return wpSendCount;
}

/**********************************************************************************************************
*函 数 名: SetWaypointSendCount
*功能说明: 设置航点发送序号
*形    参: 航点序号
*返 回 值: 无
**********************************************************************************************************/
void SetWaypointSendCount(uint16_t count)
{
    wpSendCount = count;
}

/**********************************************************************************************************
*函 数 名: SetWaypointItem
*功能说明: 设置航点信息
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void SetWaypointItem(uint16_t count, mavlink_mission_item_t item)
{
    if(count < 0xFF)
    {
        wpItem[count] = item;
        wpRecvCount = count + 1;
    }
}

/**********************************************************************************************************
*函 数 名: ClearAllWaypointItem
*功能说明: 清除所有航点信息
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ClearAllWaypointItem(void)
{
    for(uint8_t i=0; i<0xFF; i++)
        memset(&wpItem[i], 0, sizeof(mavlink_mission_item_t));
}

/**********************************************************************************************************
*函 数 名: GetWaypointItem
*功能说明: 获取航点信息
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
mavlink_mission_item_t GetWaypointItem(uint16_t count)
{
    return wpItem[count];
}

