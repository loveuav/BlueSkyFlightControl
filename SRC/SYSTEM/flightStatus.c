/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     flightStatus.c
 * @说明     飞行状态分类与检测
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05
**********************************************************************************************************/
#include "flightStatus.h"
#include "faultDetect.h"
#include "rc.h"
#include "navigation.h"
#include "ahrs.h"
#include "board.h"
#include "gps.h"
#include "gyroscope.h"
#include "message.h"
#include "mavlinkSend.h"

typedef struct
{
    uint8_t  init;           //初始化状态
    uint8_t  failsafe;       //失控保护状态
    uint8_t  armed;          //电机锁定状态
    uint8_t  flight;         //飞行状态
    uint8_t  placement;      //放置状态
    uint8_t  altControl;     //高度控制状态
    uint8_t  posControl;     //位置控制状态
    uint8_t  mode;
    uint32_t initFinishTime; //初始化完成时间
} FLIGHT_STATUS_t;

FLIGHT_STATUS_t flyStatus;
float windSpeed, windSpeedAcc;

/**********************************************************************************************************
*函 数 名: SystemInitCheck
*功能说明: 系统初始化检查
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void SystemInitCheck(void)
{
    static uint8_t initStep = 0;

    switch(initStep)
    {
    case 0:
        if(GetMessageStatus())
        {
            OsDelayMs(3000);
            initStep = 1;
        }
        break;

    /*欢迎信息*/
    case 1:
        MavlinkSendNoticeEnable(INIT_WELCOME);
        OsDelayMs(1000);
        initStep++;
        break;

    /*开始初始化*/
    case 2:
        MavlinkSendNoticeEnable(INIT_INIT_START);
        OsDelayMs(1000);
        initStep++;
        break;

    /*开始传感器检查*/
    case 3:
        MavlinkSendNoticeEnable(SENSOR_CHECK_START);
        OsDelayMs(1000);
        initStep++;
        break;

    /*传感器检查*/
    case 4:
        //检测陀螺仪状态
        if(FaultDetectGetErrorStatus(GYRO_UNDETECTED))
            MavlinkSendNoticeEnable(SENSOR_GYRO_UNDETECTED);
        else if(FaultDetectGetErrorStatus(GYRO_UNCALIBRATED))
            MavlinkSendNoticeEnable(SENSOR_GYRO_UNCALI);
        else if(GetGyroHealthStatus() == SENSOR_UNHEALTH)
            MavlinkSendNoticeEnable(SENSOR_GYRO_NEED_CALI);
        else
            MavlinkSendNoticeEnable(SENSOR_GYRO_OK);

        //检测加速度计状态
        if(FaultDetectGetErrorStatus(GYRO_UNDETECTED))
            ;
        else if(FaultDetectGetErrorStatus(ACC_UNCALIBRATED))
            MavlinkSendNoticeEnable(SENSOR_ACC_UNCALI);
        else if(GetAccHealthStatus() == SENSOR_UNHEALTH)
            MavlinkSendNoticeEnable(SENSOR_ACC_NEED_CALI);
        else
            MavlinkSendNoticeEnable(SENSOR_ACC_OK);

        //检测磁力计状态
        if(FaultDetectGetErrorStatus(MAG_UNDETECTED))
            MavlinkSendNoticeEnable(SENSOR_MAG_UNDETECTED);
        else if(FaultDetectGetErrorStatus(MAG_UNCALIBRATED))
            MavlinkSendNoticeEnable(SENSOR_MAG_UNCALI);
        else
            MavlinkSendNoticeEnable(SENSOR_MAG_OK);

        //检测气压计状态
        if(FaultDetectGetErrorStatus(BARO_UNDETECTED))
            MavlinkSendNoticeEnable(SENSOR_BARO_UNDETECTED);
        else
            MavlinkSendNoticeEnable(SENSOR_BARO_OK);

        //检测GPS状态
        if(FaultDetectGetErrorStatus(GPS_UNDETECTED))
            MavlinkSendNoticeEnable(SENSOR_GPS_UNDETECTED);
        else
            MavlinkSendNoticeEnable(SENSOR_GPS_OK);

        OsDelayMs(1000);
        initStep++;
        break;

    /*传感器检查结束*/
    case 5:
        if(SensorCheckStatus())
        {
            MavlinkSendNoticeEnable(SENSOR_CHECK_FINISH);
            OsDelayMs(1000);
            initStep++;
        }
        else
        {
            MavlinkSendNoticeEnable(SENSOR_CHECK_ERROR);
            initStep = 0xFF;
        }
        break;

    /*开始加热*/
    case 6:
        if(configUSE_SENSORHEAT == 1)
        {
            MavlinkSendNoticeEnable(INIT_HEATING);
            OsDelayMs(1000);
            initStep++;
        }
        else
        {
            MavlinkSendNoticeEnable(INIT_HEAT_DISABLE);
            OsDelayMs(1000);
            initStep = 8;
        }
        break;

    /*加热完成*/
    case 7:
        if(flyStatus.init >= HEAT_FINISH)
        {
            MavlinkSendNoticeEnable(INIT_HEAT_FINISH);
            OsDelayMs(1000);
            initStep++;
        }
        break;

    /*初始化完成*/
    case 8:
        if(flyStatus.init >= INIT_FINISH)
        {
            MavlinkSendNoticeEnable(INIT_INIT_FINISH);
            OsDelayMs(1000);
            initStep++;
        }
        break;

    default:
        break;
    }
}

/**********************************************************************************************************
*函 数 名: ArmedCheck
*功能说明: 解锁检查
*形    参: 无
*返 回 值: 检查结果
**********************************************************************************************************/
bool ArmedCheck(void)
{
    if(GetInitStatus() != INIT_FINISH)
        return false;

    return true;
}

/**********************************************************************************************************
*函 数 名: SetArmedStatus
*功能说明: 设置飞控锁定状态
*形    参: 状态
*返 回 值: 无
**********************************************************************************************************/
bool SetArmedStatus(uint8_t status)
{
    if(status == DISARMED)	//上锁
    {
        //导航数据复位
        NavigationReset();

        flyStatus.armed = DISARMED;

        return true;
    }
    else if(status == ARMED)	//解锁
    {
        if(flyStatus.armed == ARMED)
            return true;

        //解锁检查
        if(!ArmedCheck())
            return false;

        //不使用恒温的话，每次解锁时校准一下陀螺仪
#if(configUSE_SENSORHEAT == 0)
        GyroCalibrateEnable();
        OsDelayMs(100);
        while(GetGyroCaliStatus()) {}
#endif

        //导航数据复位
        NavigationReset();

        //刷新Home点坐标
        GpsResetHomePosition();

        flyStatus.armed = ARMED;

        return true;
    }
    else
        return false;
}

/**********************************************************************************************************
*函 数 名: GetArmedStatus
*功能说明: 获取飞控锁定状态
*形    参: 无
*返 回 值: 状态
**********************************************************************************************************/
uint8_t GetArmedStatus(void)
{
    return flyStatus.armed;
}

/**********************************************************************************************************
*函 数 名: SetFlightStatus
*功能说明: 获取飞行状态
*形    参: 状态
*返 回 值: 无
**********************************************************************************************************/
void SetFlightStatus(uint8_t status)
{
    flyStatus.flight = status;
}

/**********************************************************************************************************
*函 数 名: GetFlightStatus
*功能说明: 获取飞行状态
*形    参: 无
*返 回 值: 状态
**********************************************************************************************************/
uint8_t GetFlightStatus(void)
{
    return flyStatus.flight;
}

/**********************************************************************************************************
*函 数 名: PlaceStausCheck
*功能说明: 飞行器放置状态检测：静止或运动
*形    参: 角速度
*返 回 值: 无
**********************************************************************************************************/
void PlaceStausCheck(Vector3f_t gyro)
{
    Vector3f_t gyroDiff;
    static Vector3f_t lastGyro;
    static float threshold = 1.0f;
    static uint16_t checkNum = 0;
    static int16_t count = 0;

    gyroDiff.x = gyro.x - lastGyro.x;
    gyroDiff.y = gyro.y - lastGyro.y;
    gyroDiff.z = gyro.z - lastGyro.z;
    lastGyro = gyro;

    if(count < 30)
    {
        count++;
        //陀螺仪数值变化大于阈值
        if(abs(gyroDiff.x) > threshold || abs(gyroDiff.y) > threshold || abs(gyroDiff.z) > threshold)
        {
            checkNum++;
        }
    }
    else
    {
        //陀螺仪数据抖动次数大于一定值时认为飞机不处于静止状态
        if(checkNum > 10)
            flyStatus.placement = MOTIONAL;
        else
            flyStatus.placement = STATIC;

        checkNum = 0;
        count = 0;
    }
}

/**********************************************************************************************************
*函 数 名: GetPlaceStatus
*功能说明: 获取飞行器放置状态
*形    参: 无
*返 回 值: 状态
**********************************************************************************************************/
uint8_t GetPlaceStatus(void)
{
    return flyStatus.placement;
}

/**********************************************************************************************************
*函 数 名: SetAltControlStatus
*功能说明: 设置当前的高度控制状态
*形    参: 状态
*返 回 值: 无
**********************************************************************************************************/
void SetAltControlStatus(uint8_t status)
{
    flyStatus.altControl = status;
}

/**********************************************************************************************************
*函 数 名: GetAltControlStatus
*功能说明: 设置当前的高度控制状态
*形    参: 状态
*返 回 值: 无
**********************************************************************************************************/
uint8_t GetAltControlStatus(void)
{
    return flyStatus.altControl;
}

/**********************************************************************************************************
*函 数 名: SetPosControlStatus
*功能说明: 设置当前的位置控制状态
*形    参: 状态
*返 回 值: 无
**********************************************************************************************************/
void SetPosControlStatus(uint8_t status)
{
    flyStatus.posControl = status;
}

/**********************************************************************************************************
*函 数 名: GetPosControlStatus
*功能说明: 获取当前的位置控制状态
*形    参: 无
*返 回 值: 状态
**********************************************************************************************************/
uint8_t GetPosControlStatus(void)
{
    return flyStatus.posControl;
}

/**********************************************************************************************************
*函 数 名: SetInitStatus
*功能说明: 设置飞控初始化状态
*形    参: 无
*返 回 值: 状态
**********************************************************************************************************/
void SetInitStatus(uint8_t status)
{
    flyStatus.init = status;
    
    if(status == INIT_FINISH)
        flyStatus.initFinishTime = GetSysTimeMs();
}

/**********************************************************************************************************
*函 数 名: GetInitStatus
*功能说明: 获取飞控初始化状态
*形    参: 无
*返 回 值: 状态
**********************************************************************************************************/
uint8_t GetInitStatus(void)
{
    return flyStatus.init;
}

/**********************************************************************************************************
*函 数 名: GetInitFinishTime
*功能说明: 获取飞控初始化完成时间
*形    参: 无
*返 回 值: 时间(ms)
**********************************************************************************************************/
uint32_t GetInitFinishTime(void)
{
    return flyStatus.initFinishTime;
}

/**********************************************************************************************************
*函 数 名: SetFlightMode
*功能说明: 设置飞行模式
*形    参: 模式
*返 回 值: 无
**********************************************************************************************************/
void SetFlightMode(uint8_t mode)
{
    switch(mode)
    {
    case    MANUAL:
        flyStatus.mode = MANUAL;			//手动模式（自稳）
        break;
    case    SEMIAUTO:
        flyStatus.mode = SEMIAUTO;			//半自动模式（定高）
        break;
    case    AUTO:
        flyStatus.mode = AUTO;				//自动模式（定点）
        break;
    case    AUTOTAKEOFF:
        //flyStatus.mode = AUTOTAKEOFF;		//自动起飞
        break;
    case    AUTOLAND:
        if(flyStatus.armed == ARMED)
            flyStatus.mode = AUTOLAND;		//自动降落
        break;
    case    RETURNTOHOME:
        if(flyStatus.armed == ARMED)
            flyStatus.mode = RETURNTOHOME;	//自动返航
        break;
    case    AUTOCIRCLE:
        //flyStatus.mode = AUTOCIRCLE;		//自动环绕
        break;
    case    AUTOPILOT:
        flyStatus.mode = AUTOPILOT;		//自动航线
        break;
    case    FOLLOWME:
        //flyStatus.mode = FOLLOWME;		//自动跟随
        break;
    case    0xFF:
        break;
    default:
        flyStatus.mode = AUTO;
        break;
    }
}

/**********************************************************************************************************
*函 数 名: GetFlightMode
*功能说明: 获取当前的飞行模式
*形    参: 无
*返 回 值: 模式
**********************************************************************************************************/
uint8_t GetFlightMode(void)
{
    return flyStatus.mode;
}

/**********************************************************************************************************
*函 数 名: WindEstimate
*功能说明: 环境风速估计, 设定大小范围为[0:100]
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void WindEstimate(void)
{
    Vector3f_t angle = GetCopterAngle();
    static float tiltAngle;
    static float lastWindSpeed;

    static uint64_t previousT;
    float deltaT = (GetSysTimeUs() - previousT) * 1e-6;
    previousT = GetSysTimeUs();

    //目前的风速估计思路为判断悬停状态下飞机悬停倾角大小

    if(GetPosControlStatus() == POS_HOLD)
    {
        //计算悬停倾角, 并低通滤波
        angle.x = ApplyDeadbandFloat(angle.x, 3);
        angle.y = ApplyDeadbandFloat(angle.y, 3);
        tiltAngle = tiltAngle * 0.995f + Pythagorous2(angle.x, angle.y) * 0.005f;

        //将飞行倾角转换为环境风速
        windSpeed = 100 * sinf(Radians(tiltAngle));

        //计算风速变化，并低通滤波
        windSpeedAcc = windSpeedAcc * 0.95f + ((windSpeed - lastWindSpeed) / deltaT) * 0.05f;
        lastWindSpeed = windSpeed;
    }
    else
    {
        windSpeed    -= windSpeed * 0.005f;
        windSpeedAcc -= windSpeedAcc * 0.005f;
    }

}

/**********************************************************************************************************
*函 数 名: GetWindSpeed
*功能说明: 获取当前环境风速
*形    参: 无
*返 回 值: 风速
**********************************************************************************************************/
float GetWindSpeed(void)
{
    return windSpeed;
}

/**********************************************************************************************************
*函 数 名: GetWindSpeedAcc
*功能说明: 获取当前环境风加速
*形    参: 无
*返 回 值: 风加速
**********************************************************************************************************/
float GetWindSpeedAcc(void)
{
    return windSpeedAcc;
}

/**********************************************************************************************************
*函 数 名: SetFailSafeStatus
*功能说明: 设置失控状态
*形    参: 无
*返 回 值: 状态
**********************************************************************************************************/
void SetFailSafeStatus(bool status)
{
    flyStatus.failsafe = status;
}

/**********************************************************************************************************
*函 数 名: GetFailSafeStatus
*功能说明: 获取失控状态
*形    参: 无
*返 回 值: 状态
**********************************************************************************************************/
bool GetFailSafeStatus(void)
{
    return flyStatus.failsafe;
}



