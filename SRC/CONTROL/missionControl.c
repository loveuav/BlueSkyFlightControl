/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     missionControl.c
 * @说明     自主控制功能，如自动下降、自动起飞、自动返航、自动航线等
 * @版本  	 V1.1
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.07
**********************************************************************************************************/
#include "missionControl.h"
#include "waypointControl.h"
#include "flightStatus.h"
#include "flightControl.h"
#include "board.h"
#include "navigation.h"
#include "ahrs.h"
#include "gps.h"
#include "rc.h"

static Vector3f_t posCtlTarget;

uint8_t rthStep;
static float rthHeight   = 3000;  //预设返航高度 单位：cm
static float rthWaitTime = 2000;  //返航回到Home点后的悬停等待时间 单位：ms

void AutoLand(void);
void ReturnToHome(void);

/**********************************************************************************************************
*函 数 名: MissionControl
*功能说明: 自主控制任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MissionControl(void)
{
    uint8_t flightMode;

    //获取当前飞行模式
    flightMode = GetFlightMode();

    //判断当前飞行模式并选择执行对应的飞行任务
    if(flightMode == AUTOLAND)
    {
        //自动降落
        AutoLand();

        //锁定状态下转为自动模式
        if(GetArmedStatus() == DISARMED)
            SetFlightMode(AUTO);
    }
    else if(flightMode == RETURNTOHOME)
    {
        //自动返航
        ReturnToHome();

        //未起飞时转为自动降落
        if(GetFlightStatus() < IN_AIR)
            SetFlightMode(AUTOLAND);
    }
    else if(flightMode == AUTOPILOT)
    {
        //自动航线
        WaypointControl();
    }
    else
    {
        //在非任务控制模式下，更新当前位置目标
        posCtlTarget = GetCopterPosition();

        //返航步骤标志位复位
        rthStep = 0;
    }
}

/**********************************************************************************************************
*函 数 名: AutoLand
*功能说明: 自动降落
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void AutoLand(void)
{
    static float velCtlTarget = 0;
    float alttitude = GetCopterPosition().z;

    //使能航向锁定
    SetYawHoldStatus(ENABLE);

    //GPS可用时，降落过程中保持位置不变，若不可用，降落过程中飞机姿态摇杆可控
    if(GpsGetFixStatus() == true)
    {
        //使能位置控制
        SetPosCtlStatus(ENABLE);

        //更新位置控制目标
        SetPosOuterCtlTarget(posCtlTarget);
    }

    //直接控制速度，禁用高度控制
    SetAltCtlStatus(DISABLE);

    //更新高度控制状态
    SetAltControlStatus(ALT_CHANGED);

    //减速降落
    //在没有对地测距传感器的情况下，只能大致判断高度，提前进行减速
    if(alttitude < 500)
    {
        velCtlTarget = velCtlTarget * 0.99f - 70.0f * 0.01f;
    }
    else if(alttitude < 1000)
    {
        velCtlTarget = velCtlTarget * 0.99f - 80.0f * 0.01f;
    }
    else if(alttitude < 5000)
    {
        velCtlTarget = velCtlTarget * 0.99f - 200.0f * 0.01f;
    }
    else
    {
        velCtlTarget = velCtlTarget * 0.99f - 250.0f * 0.01f;
    }

    //更新高度内环控制目标
    SetAltInnerCtlTarget(velCtlTarget);
}

/**********************************************************************************************************
*函 数 名: ReturnToHome
*功能说明: 自动返航
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ReturnToHome(void)
{
    /********************************************自动返航控制逻辑**********************************************

    1.先判断当前Home点距离，若小于一定值，则直接转入步骤4
    2.记录当前飞机航向，然后更改航向目标为Home方向（机头朝Home点）
    3.判断当前高度，若小于返航高度则上升，反之进入下一步
    4.设定位置控制目标为Home点，开始返航，根据Home点距离调节飞行速度
    5.到达Home点上方，将航向目标变为初始记录值，并等待一定时间（预设值），转入自动降落模式，返航完毕

    全程检测GPS定位状态，若失去定位则转入自动降落模式
    **********************************************************************************************************/

    static uint32_t timeRecord[8];
    static float originYaw;
    static float distanceToHome;
    static float directionToHome;
    Vector3f_t position;
    static float velCtlTargert;
    Vector3f_t syncRatio;
    Vector3f_t posCtlError;
    Vector3f_t homePos;

    //获取当前位置
    position = GetCopterPosition();
    //计算Home点距离
    distanceToHome  = GetDistanceToHome(position);
    //计算Home点方向
    directionToHome = GetDirectionToHome(position);

    switch(rthStep)
    {
    case RTH_STEP_START:
        //记录起始航向
        originYaw = GetCopterAngle().z;

        //初始化飞行速度
        velCtlTargert = 0;

        //使能位置控制
        SetPosCtlStatus(ENABLE);
        //设置位置控制目标
        SetPosOuterCtlTarget(posCtlTarget);
        //更新位置控制状态
        SetPosControlStatus(POS_HOLD);

        //判断当前Home点距离，小于一定值则转入步骤4
        if(distanceToHome < 1000)
        {
            //更新位置控制目标
            posCtlTarget = GetCopterPosition();
            //转入步骤4
            rthStep = RTH_STEP_FLIGHT;
            break;
        }

        rthStep = RTH_STEP_TURN;
        break;

    case RTH_STEP_TURN:
        //设置Home方向为航向目标
        SetYawCtlTarget(directionToHome);

        if(abs(directionToHome - GetCopterAngle().z) < 3)
        {
            rthStep = RTH_STEP_CLIMB;
            posCtlTarget.z = GetCopterPosition().z;
        }

        timeRecord[RTH_STEP_TURN] = GetSysTimeMs();
        break;

    case RTH_STEP_CLIMB:
        //等待1秒
        if(GetSysTimeMs() - timeRecord[RTH_STEP_TURN] < 1000)
            break;

        //若当前高度小于返航高度，则更新高度控制目标，反之保持当前高度
        if(position.z < rthHeight)
        {
            //使能高度控制
            SetAltCtlStatus(ENABLE);

            //设置高度目标为返航高度
            SetAltOuterCtlTarget(rthHeight);
        }

        //到达目标高度后进入下一步骤
        if(position.z - rthHeight > -50)
        {
            rthStep = RTH_STEP_FLIGHT;
        }

        timeRecord[RTH_STEP_CLIMB] = GetSysTimeMs();
        break;

    case RTH_STEP_FLIGHT:
        //等待1秒
        if(GetSysTimeMs() - timeRecord[RTH_STEP_CLIMB] < 1000)
            break;

        homePos = GetHomePosition();
        posCtlError.x = posCtlTarget.x - homePos.x;
        posCtlError.y = posCtlTarget.y - homePos.y;

        //设置位置控制环最大输出
        SetMaxPosOuterCtl(800);

        //计算XY轴位置控制同步速率
        if(abs(posCtlError.x) > abs(posCtlError.y))
        {
            syncRatio.x = 1;
            syncRatio.y = abs(posCtlError.y / posCtlError.x);
        }
        else
        {
            syncRatio.x = abs(posCtlError.x / posCtlError.y);
            syncRatio.y = 1;
        }

        //根据Home点距离调整返航飞行速度
        if(distanceToHome < 1500)
            velCtlTargert = velCtlTargert * 0.999f + (100.0f / 500) * 0.001f;   //1m/s
        else if(distanceToHome < 2000)
            velCtlTargert = velCtlTargert * 0.999f + (200.0f / 500) * 0.001f;   //2m/s
        else if(distanceToHome < 3000)
            velCtlTargert = velCtlTargert * 0.999f + (300.0f / 500) * 0.001f;   //3m/s
        else if(distanceToHome < 8000)
            velCtlTargert = velCtlTargert * 0.999f + (500.0f / 500) * 0.001f;   //5m/s
        else
            velCtlTargert = velCtlTargert * 0.999f + (800.0f / 500) * 0.001f;   //8m/s

        //计算位置控制目标，目标值以一定速率匀速变化
        if(abs(posCtlError.x) > 1)
            posCtlTarget.x -= velCtlTargert * syncRatio.x * (posCtlError.x / abs(posCtlError.x));
        if(abs(posCtlError.y) > 1)
            posCtlTarget.y -= velCtlTargert * syncRatio.y * (posCtlError.y / abs(posCtlError.y));

        if(distanceToHome > 1000)
        {
            //不断更新航向目标为Home方向
            SetYawCtlTarget(directionToHome);
        }

        //Home点距离小于50cm则进入下一步
        if(distanceToHome < 50)
        {
            rthStep = RTH_STEP_TURN_BACK;
            posCtlTarget = homePos;
        }

        //设置位置控制目标
        SetPosOuterCtlTarget(posCtlTarget);

        timeRecord[RTH_STEP_FLIGHT] = GetSysTimeMs();
        break;

    case RTH_STEP_TURN_BACK:
        //等待1秒
        if(GetSysTimeMs() - timeRecord[RTH_STEP_FLIGHT] < 1000)
            break;

        //将航向控制目标设为初始记录值
        SetYawCtlTarget(originYaw);

        if(abs(originYaw - GetCopterAngle().z) < 3)
        {
            rthStep = RTH_STEP_LOITER;
        }

        timeRecord[RTH_STEP_TURN_BACK] = GetSysTimeMs();
        break;

    case RTH_STEP_LOITER:
        //等待预定的时间
        if(GetSysTimeMs() - timeRecord[RTH_STEP_TURN_BACK] < rthWaitTime)
            break;

        //返航完毕，转入自动降落模式
        SetFlightMode(AUTOLAND);
        //重置返航步骤标志位
        rthStep = RTH_STEP_START;
        break;

    default:
        break;
    }

    //使能高度控制
    SetAltCtlStatus(ENABLE);
    //使能航向锁定
    SetYawHoldStatus(ENABLE);

    //GPS不可用时直接转入自动降落模式
    if(GpsGetFixStatus() == false)
    {
        SetFlightMode(AUTOLAND);
    }
}




