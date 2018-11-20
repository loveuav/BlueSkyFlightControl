/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     navigation.c
 * @说明     组合导航
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05
**********************************************************************************************************/
#include "navigation.h"
#include "ahrs.h"
#include "board.h"
#include "kalman3.h"
#include "kalmanVel.h"
#include "gps.h"
#include "barometer.h"
#include "flightStatus.h"

#include "FreeRTOS.h"
#include "task.h"

NAVGATION_t nav;
KalmanVel_t kalmanVel;
Kalman_t kalmanPos;

static void KalmanVelInit(void);
static void KalmanPosInit(void);

/**********************************************************************************************************
*函 数 名: NavigationInit
*功能说明: 导航参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void NavigationInit(void)
{
    KalmanVelInit();
    KalmanPosInit();
}

/**********************************************************************************************************
*函 数 名: VelocityEstimate
*功能说明: 飞行速度估计 融合加速度、GPS、气压计及TOF等多个传感器的数据
*          速度的计算均在机体坐标系下进行，所以GPS速度在参与融合时需要先转换到机体坐标系
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void VelocityEstimate(void)
{
    static uint64_t previousT;
    float deltaT;
    Vector3f_t gpsVel;
    static uint32_t count;
    static bool fuseFlag;
        
    //计算时间间隔，用于积分
    deltaT = (GetSysTimeUs() - previousT) * 1e-6;
    deltaT = ConstrainFloat(deltaT, 0.0005, 0.005);
    previousT = GetSysTimeUs();

    //获取运动加速度
    nav.accel = GetCopterAccEf();

    //加速度数据更新频率1KHz，而气压数据更新频率只有25Hz，GPS数据只有10Hz
    //这里将气压与GPS参与融合的频率强制统一为25Hz
    if(count++ % 40 == 0)
    {
        if(GpsGetFixStatus())
        {
            //获取GPS速度测量值，转换速度值到机体坐标系
            TransVelToBodyFrame(GpsGetVelocity(), &gpsVel, GetCopterAngle().z);
        }
        else
        {
            gpsVel.x = 0;
            gpsVel.y = 0;
            gpsVel.z = 0;
        }

        nav.velMeasure[0] = gpsVel.x;           //GPS速度x轴
        nav.velMeasure[1] = gpsVel.y;           //GPS速度y轴
        nav.velMeasure[2] = gpsVel.z;           //GPS速度z轴
        nav.velMeasure[3] = BaroGetVelocity();  //气压速度值
        nav.velMeasure[4] = 0;                  //TOF速度值
        nav.velMeasure[5] = 0; 
        
        //GPS已定位且精度高于一定值时才使用GPS速度z轴数据
        if(GpsGetFixStatus() && (GpsGetAccuracy() < 1.5f))
            KalmanVelUseMeasurement(&kalmanVel, GPS_VEL_Z, true);
        else if((!GpsGetFixStatus()) || (GpsGetAccuracy() > 2.0f))
            KalmanVelUseMeasurement(&kalmanVel, GPS_VEL_Z, false);
            
        //禁用TOF速度观测量：尚未装备TOF传感器
        KalmanVelUseMeasurement(&kalmanVel, TOF_VEL, false);
        
        fuseFlag = true;
    }
    else
    {
        fuseFlag = false;
    }
    
    /*
    更新卡尔曼滤波器
    估计飞行速度及加速度bias
    */
    KalmanVelUpdate(&kalmanVel, &nav.velocity, &nav.accel_bias, nav.accel, nav.velMeasure, deltaT, fuseFlag);
}

/**********************************************************************************************************
*函 数 名: PositionEstimate
*功能说明: 位置估计 目前只融合了GPS与气压，以后还会有光流、TOF等模块数据的参与
*          位置的计算均在地理坐标系（东北天）下进行
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void PositionEstimate(void)
{
    static uint64_t previousT;
    float deltaT;
    Vector3f_t velocityEf;
    Vector3f_t input;
    static uint32_t count;
    static bool fuseFlag;

    //计算时间间隔，用于积分
    deltaT = (GetSysTimeUs() - previousT) * 1e-6;
    deltaT = ConstrainFloat(deltaT, 0.0005, 0.002);
    previousT = GetSysTimeUs();

    //速度数据更新频率1KHz，而气压数据更新频率只有25Hz，GPS数据只有10Hz
    //这里将气压与GPS参与融合的频率强制统一为25Hz
    if(count++ % 40 == 0)
    {
        if(GpsGetFixStatus())
        {
            //获取GPS位置
            nav.posMeasure = GpsGetPosition();
        }
        else
        {
            nav.posMeasure.x = 0;
            nav.posMeasure.y = 0;
        }
        //获取气压高度测量值
        nav.posMeasure.z = BaroGetAlt();

        fuseFlag = true;
    }
    else
    {
        fuseFlag = false;
    }

    //转换速度值到导航系
    TransVelToEarthFrame(nav.velocity, &velocityEf, GetCopterAngle().z);

    //速度积分
    input.x = velocityEf.x * deltaT;
    input.y = velocityEf.y * deltaT;
    input.z = velocityEf.z * deltaT;

    //位置估计
    KalmanUpdate(&kalmanPos, input, nav.posMeasure, fuseFlag);
    nav.position = kalmanPos.state;
}

/**********************************************************************************************************
*函 数 名: AltCovarianceSelfAdaptation
*功能说明: 高度误差协方差自适应
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void AltCovarianceSelfAdaptation(void)
{
    static Vector3f_t accelLpf;
    float accelMag;
    float windSpeed, windSpeedAcc;

    //对运动加速度进行低通滤波
    accelLpf.x = accelLpf.x * 0.99f + nav.accel.x * 0.01f;
    accelLpf.y = accelLpf.y * 0.99f + nav.accel.y * 0.01f;

    //计算运动加速度模值
    accelMag = Pythagorous2(accelLpf.x, accelLpf.y);

    //获取当前环境风速与风加速（悬停时）
    windSpeed = GetWindSpeed() * 0.02f;
    windSpeedAcc = GetWindSpeedAcc() * 0.005f;

    //飞行中，速度变化会带来气压变化（伯努利效应），引起高度计算误差
    //除了要适当补偿气压误差外，还可以在这种状态下增大高度测量协方差，减小气压融合权重
    if(GetPosControlStatus() == POS_CHANGED)
    {
        if(GetAltControlStatus() == ALT_HOLD)
        {
            kalmanVel.r[3][3] = Sq(50 * (1 + ConstrainFloat(accelMag * 1.5f, 0, 1.5f)));
            kalmanPos.r[8]    = Sq(40 * (1 + ConstrainFloat(accelMag, 0, 1.0f)));
        }
        else
        {
            kalmanVel.r[3][3] = Sq(50 * (1 + ConstrainFloat(accelMag, 0, 0.5f)));
            kalmanPos.r[8]    = 1000;
        }
    }
    else if(GetPosControlStatus() == POS_HOLD)
    {
        //悬停时,气压误差会随着环境风速的变化而增大
        if(GetAltControlStatus() == ALT_HOLD)
        {
            kalmanVel.r[3][3] = Sq(50 * (1 + ConstrainFloat(windSpeed * 0.8f + windSpeedAcc * 0.2f, 0, 0.5f)));
            kalmanPos.r[8]    = Sq(37 * (1 + ConstrainFloat(windSpeed * 0.8f + windSpeedAcc * 0.2f, 0, 1.0f)));
        }
        else if(GetAltControlStatus() == ALT_CHANGED)
        {
            kalmanVel.r[3][3] = Sq(50 * (1 + ConstrainFloat(windSpeed * 0.8f + windSpeedAcc * 0.2f, 0, 0.5f)));
            kalmanPos.r[8]    = Sq(30 * (1 + ConstrainFloat(windSpeed * 0.8f + windSpeedAcc * 0.2f, 0, 1.0f)));
        }
        else
        {
            kalmanVel.r[3][3] = 1500;
            kalmanPos.r[8]    = 1000;
        }
    }
    else
    {
        kalmanVel.r[3][3] = 2000;
        kalmanPos.r[8]    = 1000;
    }
    
    //根据GPS定位精度改变GPS速度z轴的噪声方差
    kalmanVel.r[2][2] = Sq(50 * (ConstrainFloat(GpsGetAccuracy() - 0.5f, 0.9f, 1.5f)));
}

/**********************************************************************************************************
*函 数 名: PosCovarianceSelfAdaptation
*功能说明: 位置误差协方差自适应
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void PosCovarianceSelfAdaptation(void)
{
    //获取GPS水平定位精度
    float gpsAcc = GpsGetAccuracy();

    if(GetPosControlStatus() == POS_HOLD)
    {
        kalmanVel.r[0][0] = kalmanVel.r[1][1] = Sq(8 * (1 + ConstrainFloat((gpsAcc - 0.8f), -0.2, +1)));

        kalmanPos.r[0] = ConstrainFloat(sqrtf(kalmanPos.r[0]) + 0.002f, 10, 100);
        kalmanPos.r[4] = ConstrainFloat(sqrtf(kalmanPos.r[4]) + 0.002f, 10, 100);
        kalmanPos.r[0] = Sq(kalmanPos.r[0]);
        kalmanPos.r[4] = Sq(kalmanPos.r[4]);
    }
    else if(GetPosControlStatus() == POS_CHANGED)
    {
        kalmanVel.r[0][0] = kalmanVel.r[1][1] = Sq(8 * (1 + ConstrainFloat((gpsAcc - 0.8f), -0.2, +1)));
        kalmanPos.r[0] = kalmanPos.r[4] = 10;
    }
    else if(GetPosControlStatus() == POS_BRAKE)
    {
        kalmanVel.r[0][0] = kalmanVel.r[1][1] = Sq(45);
        kalmanPos.r[0] = kalmanPos.r[4] = 10;
    }
    else if(GetPosControlStatus() == POS_BRAKE_FINISH)
    {
        kalmanVel.r[0][0] = kalmanVel.r[1][1] = Sq(10 * (1 + ConstrainFloat((gpsAcc - 0.8f), -0.2, +1)));
        kalmanPos.r[0] = kalmanPos.r[4] = 10;
    }
}

/**********************************************************************************************************
*函 数 名: KalmanVelInit
*功能说明: 飞行速度估计的卡尔曼结构体初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void KalmanVelInit(void)
{
    float qMatInit[6][6] = {{0.05, 0, 0, 0, 0, 0},
                            {0, 0.05, 0, 0, 0, 0},
                            {0, 0, 0.03, 0, 0, 0},      
                            {0.003, 0, 0, 0, 0, 0},
                            {0, 0.003, 0, 0, 0, 0},
                            {0, 0, 0.003, 0, 0, 0}};

    float rMatInit[6][6] = {{50, 0, 0, 0, 0, 0},            //GPS速度x轴数据噪声方差
                            {0, 50, 0, 0, 0, 0},            //GPS速度y轴数据噪声方差
                            {0, 0, 2000, 0, 0, 0},          //GPS速度z轴数据噪声方差     
                            {0, 0, 0, 2500, 0, 0},          //气压速度数据噪声方差
                            {0, 0, 0, 0, 2000, 0},          //TOF速度数据噪声方差
                            {0, 0, 0, 0, 0, 500000}};       //z轴速度高通滤波系数

    float pMatInit[6][6] = {{5, 0, 0, 0, 0, 0},
                            {0, 5, 0, 0, 0, 0},
                            {0, 0, 5, 0, 0, 0},      
                            {2, 0, 0, 2, 0, 0},
                            {0, 2, 0, 0, 2, 0},
                            {0, 0, 6, 0, 0, 2}};    //增大协方差P的初值，可以提高初始化时bias的收敛速度

    float hMatInit[6][6] = {{1, 0, 0, 0, 0, 0},
                            {0, 1, 0, 0, 0, 0},
                            {0, 0, 1, 0, 0, 0},      
                            {0, 0, 1, 0, 0, 0},
                            {0, 0, 1, 0, 0, 0},
                            {0, 0, 1, 0, 0, 0}};    //h[5][2]:速度z轴增加少许高通滤波效果

    float fMatInit[6][6] = {{1, 0, 0, 0, 0, 0},
                            {0, 1, 0, 0, 0, 0},
                            {0, 0, 1, 0, 0, 0},      
                            {0, 0, 0, 1, 0, 0},
                            {0, 0, 0, 0, 1, 0},
                            {0, 0, 0, 0, 0, 1}};
    
    float bMatInit[6][6] = {{1, 0, 0, 0, 0, 0},
                            {0, 1, 0, 0, 0, 0},
                            {0, 0, 1, 0, 0, 0},      
                            {0, 0, 0, 0, 0, 0},
                            {0, 0, 0, 0, 0, 0},
                            {0, 0, 0, 0, 0, 0}};
    
    //初始化卡尔曼滤波器的相关矩阵
    KalmanVelQMatSet(&kalmanVel, qMatInit);
    KalmanVelRMatSet(&kalmanVel, rMatInit);
    KalmanVelCovarianceMatSet(&kalmanVel, pMatInit);
    KalmanVelObserveMapMatSet(&kalmanVel, hMatInit);
    KalmanVelStateTransMatSet(&kalmanVel, fMatInit);
    KalmanVelBMatSet(&kalmanVel, bMatInit);
                            
    //状态滑动窗口，用于解决卡尔曼状态估计量与观测量之间的相位差问题
    kalmanVel.slidWindowSize = 250;
    kalmanVel.stateSlidWindow = pvPortMalloc(kalmanVel.slidWindowSize * sizeof(Vector3f_t));
    kalmanVel.fuseDelay[GPS_VEL_X] = 220;    //GPS速度x轴数据延迟参数：0.22s
    kalmanVel.fuseDelay[GPS_VEL_Y] = 220;    //GPS速度y轴数据延迟参数：0.22s
    kalmanVel.fuseDelay[GPS_VEL_Z] = 220;    //GPS速度z轴数据延迟参数：0.22s
    kalmanVel.fuseDelay[BARO_VEL]  = 100;    //气压速度数据延迟参数：0.1s
    kalmanVel.fuseDelay[TOF_VEL]   = 30;     //TOF速度数据延迟参数：
}

/**********************************************************************************************************
*函 数 名: KalmanPosInit
*功能说明: 位置估计的卡尔曼结构体初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void KalmanPosInit(void)
{
    float qMatInit[9] = {0.003, 0, 0, 0, 0.003, 0, 0, 0, 0.03};
    float rMatInit[9] = {500, 0,  0, 0, 500, 0, 0, 0, 1000};
    float pMatInit[9] = {3, 0, 0, 0, 3, 0, 0, 0, 5};
    float fMatInit[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float hMatInit[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float bMatInit[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    //初始化卡尔曼滤波器的相关矩阵
    KalmanQMatSet(&kalmanPos, qMatInit);
    KalmanRMatSet(&kalmanPos, rMatInit);
    KalmanBMatSet(&kalmanPos, bMatInit);
    KalmanCovarianceMatSet(&kalmanPos, pMatInit);
    KalmanStateTransMatSet(&kalmanPos, fMatInit);
    KalmanObserveMapMatSet(&kalmanPos, hMatInit);

    //状态滑动窗口，用于解决卡尔曼状态估计量与观测量之间的相位差问题
    kalmanPos.slidWindowSize = 200;
    kalmanPos.statusSlidWindow = pvPortMalloc(kalmanPos.slidWindowSize * sizeof(kalmanPos.state));
    kalmanPos.fuseDelay.x = 200;    //0.2s延时
    kalmanPos.fuseDelay.y = 200;    //0.2s延时
    kalmanPos.fuseDelay.z = 50;    //0.05s延时
}

/**********************************************************************************************************
*函 数 名: NavigationReset
*功能说明: 导航相关数据复位
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void NavigationReset(void)
{
    kalmanVel.state[0] = 0;
    kalmanVel.state[1] = 0;
    kalmanVel.state[2] = 0;

    if(GpsGetFixStatus())
    {
        kalmanPos.state.x = GpsGetPosition().x;
        kalmanPos.state.y = GpsGetPosition().y;
    }
    else
    {
        kalmanPos.state.x = 0;
        kalmanPos.state.y = 0;
    }
    kalmanPos.state.z = BaroGetAlt();
}

/**********************************************************************************************************
*函 数 名: GetCopterAccel
*功能说明: 获取飞行加速度
*形    参: 无
*返 回 值: 加速度值
**********************************************************************************************************/
Vector3f_t GetCopterAccel(void)
{
    return nav.accel;
}

/**********************************************************************************************************
*函 数 名: GetAccelBias
*功能说明: 获取加速度bias
*形    参: 无
*返 回 值: 加速度bias值
**********************************************************************************************************/
Vector3f_t GetAccelBias(void)
{
    return nav.accel_bias;
}

/**********************************************************************************************************
*函 数 名: GetCopterVelocity
*功能说明: 获取飞行速度估计值
*形    参: 无
*返 回 值: 速度值
**********************************************************************************************************/
Vector3f_t GetCopterVelocity(void)
{
    return nav.velocity;
}

/**********************************************************************************************************
*函 数 名: GetCopterVelMeasure
*功能说明: 获取飞行速度测量值
*形    参: 无
*返 回 值: 速度值
**********************************************************************************************************/
float* GetCopterVelMeasure(void)
{
    return nav.velMeasure;
}

/**********************************************************************************************************
*函 数 名: GetCopterPosition
*功能说明: 获取位置估计值
*形    参: 无
*返 回 值: 位置值
**********************************************************************************************************/
Vector3f_t GetCopterPosition(void)
{
    return nav.position;
}

/**********************************************************************************************************
*函 数 名: GetCopterPosMeasure
*功能说明: 获取位置测量值
*形    参: 无
*返 回 值: 速度值
**********************************************************************************************************/
Vector3f_t GetCopterPosMeasure(void)
{
    return nav.posMeasure;
}
