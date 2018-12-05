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
#include "drv_sbus.h"
#include "drv_ppm.h"
#include "accelerometer.h"
#include "magnetometer.h"
#include "navigation.h"
#include "flightControl.h"

#define MINCHECK        1150
#define MIDCHECK        1500
#define MAXCHECK        1850

#define RC_LEGAL_MIN    980
#define RC_LEGAL_MAX    2020

#define AUX_CHECK_LOW   1100
#define AUX_CHECK_MID   1500
#define AUX_CHECK_HIGH  1900

RCDATA_t rcData;
RCCOMMAND_t rcCommand;
uint32_t failsafeTime = 0;
uint8_t rcAuxMode[12][3];

static uint8_t  armedCheckFlag = 0;

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
    Sbus_SetRcDataCallback(RcDataUpdate);
    PPM_SetRcDataCallback(RcDataUpdate);

    //遥控通道数据初始化
    rcData.roll     = 1500;
    rcData.pitch    = 1500;
    rcData.throttle = 1500;
    rcData.yaw      = 1500;
    rcData.aux1     = 1500;
    rcData.aux2     = 1500;
    rcData.aux3     = 1500;
    rcData.aux4     = 1500;
    rcData.aux5     = 1500;
    rcData.aux6     = 1500;
    rcData.aux7     = 1500;
    rcData.aux8     = 1500;
    rcData.aux9     = 1500;
    rcData.aux10    = 1500;
    rcData.aux11    = 1500;
    rcData.aux12    = 1500;

    //设置各辅助通道对应的飞行模式
    rcAuxMode[AUX1][LOW]  = SEMIAUTO;
    rcAuxMode[AUX1][MID]  = AUTO;
    rcAuxMode[AUX1][HIGH] = RETURNTOHOME;

    rcAuxMode[AUX2][LOW]  = 0xFF;//AUTO;
    rcAuxMode[AUX2][MID]  = 0xFF;//AUTOLAND;
    rcAuxMode[AUX2][HIGH] = 0xFF;//RETURNTOHOME;

    rcAuxMode[AUX3][LOW]  = 0xFF;
    rcAuxMode[AUX3][MID]  = 0xFF;
    rcAuxMode[AUX3][HIGH] = 0xFF;

    rcAuxMode[AUX4][LOW]  = 0xFF;
    rcAuxMode[AUX4][MID]  = 0xFF;
    rcAuxMode[AUX4][HIGH] = 0xFF;

    //初始化飞行模式为自动模式
    SetFlightMode(AUTO);
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
        rcCommand.throttle = (rcData.throttle - 1000) * 2;
    }
    else
    {
        rcCommand.roll     = 0;
        rcCommand.pitch    = 0;
        rcCommand.yaw      = 0;
        rcCommand.throttle = (rcData.throttle - 1000) * 2;
    }
}

/**********************************************************************************************************
*函 数 名: GetRcDATA
*功能说明: 获取摇杆数据
*形    参: 无
*返 回 值: 摇杆数据
**********************************************************************************************************/
RCDATA_t GetRcData(void)
{
    return rcData;
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
    if(rcData.throttle > MINCHECK + 50)
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
    static uint8_t auxStatus[12];

    //辅助通道1检测
    if(rcData.aux1 > RC_LEGAL_MIN && rcData.aux1 < RC_LEGAL_MAX)
    {
        if(abs(rcData.aux1 - AUX_CHECK_LOW) < 200)
        {
            if(auxStatus[AUX1] != LOW)
            {
                auxStatus[AUX1] = LOW;
                SetFlightMode(rcAuxMode[AUX1][LOW]);
            }
        }
        else if(abs(rcData.aux1 - AUX_CHECK_MID) < 200)
        {
            if(auxStatus[AUX1] != MID)
            {
                auxStatus[AUX1] = MID;
                SetFlightMode(rcAuxMode[AUX1][MID]);
            }
        }
        else if(abs(rcData.aux1 - AUX_CHECK_HIGH) < 200)
        {
            if(auxStatus[AUX1] != HIGH)
            {
                auxStatus[AUX1] = HIGH;
                SetFlightMode(rcAuxMode[AUX1][HIGH]);
            }
        }
    }

    //辅助通道2检测
    if(rcData.aux2 > RC_LEGAL_MIN && rcData.aux2 < RC_LEGAL_MAX)
    {
        if(abs(rcData.aux2 - AUX_CHECK_LOW) < 200)
        {
            if(auxStatus[AUX2] != LOW)
            {
                auxStatus[AUX2] = LOW;
                SetFlightMode(rcAuxMode[AUX2][LOW]);
            }
        }
        else if(abs(rcData.aux2 - AUX_CHECK_MID) < 200)
        {
            if(auxStatus[AUX2] != MID)
            {
                auxStatus[AUX2] = MID;
                SetFlightMode(rcAuxMode[AUX2][MID]);
            }
        }
        else if(abs(rcData.aux2 - AUX_CHECK_HIGH) < 200)
        {
            if(auxStatus[AUX2] != HIGH)
            {
                auxStatus[AUX2] = HIGH;
                SetFlightMode(rcAuxMode[AUX2][HIGH]);
            }
        }
    }

    //辅助通道3检测
    if(rcData.aux3 > RC_LEGAL_MIN && rcData.aux3 < RC_LEGAL_MAX)
    {
        if(abs(rcData.aux3 - AUX_CHECK_LOW) < 200)
        {
            if(auxStatus[AUX3] != LOW)
            {
                auxStatus[AUX3] = LOW;
                SetFlightMode(rcAuxMode[AUX3][LOW]);
            }
        }
        else if(abs(rcData.aux3 - AUX_CHECK_MID) < 200)
        {
            if(auxStatus[AUX3] != MID)
            {
                auxStatus[AUX3] = MID;
                SetFlightMode(rcAuxMode[AUX3][MID]);
            }
        }
        else if(abs(rcData.aux3 - AUX_CHECK_HIGH) < 200)
        {
            if(auxStatus[AUX3] != HIGH)
            {
                auxStatus[AUX3] = HIGH;
                SetFlightMode(rcAuxMode[AUX3][HIGH]);
            }
        }
    }

    //辅助通道4检测
    if(rcData.aux4 > RC_LEGAL_MIN && rcData.aux4 < RC_LEGAL_MAX)
    {
        if(abs(rcData.aux4 - AUX_CHECK_LOW) < 200)
        {
            if(auxStatus[AUX4] != LOW)
            {
                auxStatus[AUX4] = LOW;
                SetFlightMode(rcAuxMode[AUX4][LOW]);
            }
        }
        else if(abs(rcData.aux4 - AUX_CHECK_MID) < 200)
        {
            if(auxStatus[AUX4] != MID)
            {
                auxStatus[AUX4] = MID;
                SetFlightMode(rcAuxMode[AUX4][MID]);
            }
        }
        else if(abs(rcData.aux4 - AUX_CHECK_HIGH) < 200)
        {
            if(auxStatus[AUX4] != HIGH)
            {
                auxStatus[AUX4] = HIGH;
                SetFlightMode(rcAuxMode[AUX4][HIGH]);
            }
        }
    }
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

/**********************************************************************************************************
*函 数 名: FlightStatusUpdate
*功能说明: 飞行状态更新
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void FlightStatusUpdate(void)
{
    static uint8_t landVibraFlag;
    static uint32_t disArmedCheckTime;
    static uint32_t landCheckTime[10];

    /********************************************待机***********************************************/
    if(GetFlightStatus() == STANDBY)
    {
        //解锁状态下通过摇杆位置判断是否进入起飞状态
        if(GetArmedStatus() == ARMED)
        {
            //手动模式时油门位置低于中点时也会起飞，所以分开判断
            if(GetFlightMode() == MANUAL)
            {
                if(rcData.throttle > MINCHECK)
                    SetFlightStatus(TAKE_OFF);
            }
            else
            {
                if(rcData.throttle > MIDCHECK && GetAltControlStatus() == ALT_CHANGED)
                    SetFlightStatus(TAKE_OFF);
            }
        }

        //进入待机模式且油门杆位最低则锁定电机，自动降落模式下立即锁定
        if(GetArmedStatus() == ARMED)
        {
            if(GetFlightMode() == AUTOLAND)
            {
                SetArmedStatus(DISARMED);
            }
            else
            {
                if(rcData.throttle < MINCHECK && abs(rcData.yaw - MIDCHECK) < 50 && !armedCheckFlag)
                {
                    if(GetSysTimeMs() - disArmedCheckTime > 500)
                        SetArmedStatus(DISARMED);
                }
                else
                {
                    disArmedCheckTime = GetSysTimeMs();
                }
            }
        }

        //控制相关复位
        FlightControlReset();
    }
    /********************************************起飞***********************************************/
    else if(GetFlightStatus() == TAKE_OFF)
    {
        //判断垂直速度大于一定值则进入空中状态
        //后面有时间再慢慢优化起飞步骤，把起飞操作变成一个半自动过程，改善飞行体验
        //因为对于一个完全没有接触过飞机的人来说，手动起飞可能会出现各种意外（比如摇杆往上拨了一点点就立马收油）
        if(GetCopterVelocity().z > 5)
            SetFlightStatus(IN_AIR);
        else if(rcData.throttle < MINCHECK)
            SetFlightStatus(STANDBY);
    }
    /********************************************空中***********************************************/
    else if(GetFlightStatus() == IN_AIR)
    {
        if(GetFlightMode() == AUTOLAND)
        {
            SetFlightStatus(LANDING);
        }
        else if(GetFlightMode() == MANUAL)
        {
            if(rcData.throttle < MIDCHECK)
                SetFlightStatus(LANDING);
        }
        else if(rcData.throttle < MIDCHECK && GetAltControlStatus() == ALT_CHANGED)
        {
            SetFlightStatus(LANDING);
        }

        //重置落地震动检测标志位
        landVibraFlag = 0;

        landCheckTime[0] = GetSysTimeMs();
        landCheckTime[1] = GetSysTimeMs();
        landCheckTime[2] = GetSysTimeMs();
        landCheckTime[3] = GetSysTimeMs();
    }
    /********************************************降落***********************************************/
    else if(GetFlightStatus() == LANDING)
    {
        //返回IN_AIR状态
        if(GetFlightMode() != AUTOLAND)
        {
            if(GetAltControlStatus() == ALT_HOLD || rcData.throttle > MIDCHECK)
                SetFlightStatus(IN_AIR);
        }
        //降落检测实现比较麻烦，因为要保证安全（不出现误检测）的同时要提升检测的速度（落地后能够立即完成检测）
        //目前的检测方式为检测垂直方向的控制误差超过一定值并持续一定时间便认为已落地
        //通过检测落地时产生的震动来加快检测
        //在有TOF或超声波等对地测距方式时，再额外添加检测方式

        //加速度模值微分值大小超过一定值时认为检测到震动
//        if()
//            landVibraFlag = 1;

        if(GetFlightMode() != MANUAL && landVibraFlag && abs(GetPosInnerCtlError().z) > 35 &&
                abs(GetCopterVelocity().z) < 100 && abs(GetAccMag() - 1) < 0.1f)
        {
            if(GetSysTimeMs() - landCheckTime[0] > 1000)
                SetFlightStatus(FINISH_LANDING);
        }
        else
            landCheckTime[0] = GetSysTimeMs();

        if(GetFlightMode() == MANUAL)           //手动模式
        {
            if(rcData.throttle < MINCHECK && abs(GetCopterVelocity().z) < 80)
            {
                if(GetSysTimeMs() - landCheckTime[3] > 2000)
                    SetFlightStatus(FINISH_LANDING);
            }
            else
                landCheckTime[3] = GetSysTimeMs();
        }
        else if(GetFlightMode() == AUTOLAND)    //自动降落模式
        {
            if(abs(GetPosInnerCtlError().z) > 50 && abs(GetCopterVelocity().z) < 50)
            {
                if(GetSysTimeMs() - landCheckTime[2] > 2000)
                    SetFlightStatus(FINISH_LANDING);
            }
            else
                landCheckTime[2] = GetSysTimeMs();
        }
        else                                    //其它模式
        {
            if(rcData.throttle < MINCHECK && abs(GetPosInnerCtlError().z) > 200 && abs(GetCopterVelocity().z) < 50)
            {
                if(GetSysTimeMs() - landCheckTime[3] > 2000)
                    SetFlightStatus(FINISH_LANDING);
            }
            else
                landCheckTime[3] = GetSysTimeMs();
        }
    }
    /*******************************************降落完成**********************************************/
    else if(GetFlightStatus() == FINISH_LANDING)
    {
        //这一步目前没什么用
        SetFlightStatus(STANDBY);
    }
}


