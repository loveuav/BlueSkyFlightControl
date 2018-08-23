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
#include "gps.h"
#include "barometer.h"
#include "flightStatus.h"

#include "FreeRTOS.h"
#include "task.h"

NAVGATION_t nav;
Kalman_t kalmanVel;
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
*功能说明: 飞行速度估计 目前只融合了GPS与气压，以后还会有光流、TOF等模块数据的参与
*          速度的计算均在机体坐标系下进行，所以GPS速度在参与融合时需要先转换到机体坐标系 
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void VelocityEstimate(void)
{
	static uint64_t previousT;	
	float deltaT;
	Vector3f_t gpsVel;	
    Vector3f_t input;
    static uint32_t count;
    static bool fuseFlag;
    static Vector3f_t accel_bias = {0, 0, 0};
    static float velErrorIntRate = 0.00001f;
    
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
        }
        
        nav.velMeasure.x = gpsVel.x;
        nav.velMeasure.y = gpsVel.y;
        //获取气压速度测量值
        nav.velMeasure.z = BaroGetVelocity();	
        
        fuseFlag = true;
    }
    else
    {
        fuseFlag = false;
    }

    //修正加速度零偏
    nav.accel.x += accel_bias.x;
    nav.accel.y += accel_bias.y;
    nav.accel.z += accel_bias.z;
    
    //加速度积分，并转换单位为cm
    input.x = nav.accel.x * GRAVITY_ACCEL * deltaT * 100;
    input.y = nav.accel.y * GRAVITY_ACCEL * deltaT * 100;
    input.z = nav.accel.z * GRAVITY_ACCEL * deltaT * 100;
    
    //速度估计
    KalmanUpdate(&kalmanVel, input, nav.velMeasure, fuseFlag);
    nav.velocity = kalmanVel.state;
    
    //加速度（导航系）零偏估计
    accel_bias.x += (nav.velMeasure.x - kalmanVel.statusSlidWindow[kalmanVel.slidWindowSize - kalmanVel.fuseDelay.x].x) * deltaT * velErrorIntRate;
    accel_bias.y += (nav.velMeasure.y - kalmanVel.statusSlidWindow[kalmanVel.slidWindowSize - kalmanVel.fuseDelay.y].y) * deltaT * velErrorIntRate;        
    accel_bias.z += (nav.velMeasure.z - kalmanVel.statusSlidWindow[kalmanVel.slidWindowSize - kalmanVel.fuseDelay.z].z) * deltaT * velErrorIntRate;
    
    accel_bias.x  = ConstrainFloat(accel_bias.x, -0.03, 0.03);
    accel_bias.y  = ConstrainFloat(accel_bias.y, -0.03, 0.03);
    accel_bias.z  = ConstrainFloat(accel_bias.z, -0.03, 0.03);
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
			kalmanVel.r[8] = Sq(50 * (1 + ConstrainFloat(accelMag * 1.5f, 0, 1.5f)));
			kalmanPos.r[8] = Sq(40 * (1 + ConstrainFloat(accelMag, 0, 1.0f)));
		}
		else
		{
			kalmanVel.r[8] = Sq(50 * (1 + ConstrainFloat(accelMag, 0, 0.5f)));
			kalmanPos.r[8] = 500;			
		}
	}
	else if(GetPosControlStatus() == POS_HOLD)	
	{
		//悬停时,气压误差会随着环境风速的变化而增大
		if(GetAltControlStatus() == ALT_HOLD)
		{
			kalmanVel.r[8] = Sq(50 * (1 + ConstrainFloat(windSpeed * 0.8f + windSpeedAcc * 0.2f, 0, 0.5f)));
			kalmanPos.r[8] = Sq(40 * (1 + ConstrainFloat(windSpeed * 0.8f + windSpeedAcc * 0.2f, 0, 1.0f)));	
		}
		else if(GetAltControlStatus() == ALT_CHANGED)
		{
			kalmanVel.r[8] = 2000;
			kalmanPos.r[8] = 200;	
		}
		else
		{
			kalmanVel.r[8] = 1500;
			kalmanPos.r[8] = 500;			
		}
	}
	else
	{
		kalmanVel.r[8] = 2000;
		kalmanPos.r[8] = 1000;	
	}
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
        kalmanVel.r[0] = kalmanVel.r[4] = Sq(8 * (1 + ConstrainFloat((gpsAcc - 0.8f), -0.2, +1)));
        
        kalmanPos.r[0] = ConstrainFloat(sqrtf(kalmanPos.r[0]) + 0.002f, 10, 100);
        kalmanPos.r[4] = ConstrainFloat(sqrtf(kalmanPos.r[4]) + 0.002f, 10, 100);
        kalmanPos.r[0] = Sq(kalmanPos.r[0]);
        kalmanPos.r[4] = Sq(kalmanPos.r[4]);    
	}
	else if(GetPosControlStatus() == POS_CHANGED)	
	{
        kalmanVel.r[0] = kalmanVel.r[4] = Sq(8 * (1 + ConstrainFloat((gpsAcc - 0.8f), -0.2, +1)));
        kalmanPos.r[0] = kalmanPos.r[4] = 10;
	}
	else if(GetPosControlStatus() == POS_BRAKE)	
	{
        kalmanVel.r[0] = kalmanVel.r[4] = Sq(45);
        kalmanPos.r[0] = kalmanPos.r[4] = 10;
	}
	else if(GetPosControlStatus() == POS_BRAKE_FINISH)	
	{
        kalmanVel.r[0] = kalmanVel.r[4] = Sq(10 * (1 + ConstrainFloat((gpsAcc - 0.8f), -0.2, +1)));
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
    float qMatInit[9] = {0.05, 0, 0, 0, 0.05, 0, 0, 0, 0.03};
    float rMatInit[9] = {50, 0,  0, 0, 50, 0, 0, 0, 2500};
    float pMatInit[9] = {5, 0, 0, 0, 5, 0, 0, 0, 8};
    float fMatInit[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float hMatInit[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float bMatInit[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    //初始化卡尔曼滤波器的相关矩阵
    KalmanQMatSet(&kalmanVel, qMatInit);
    KalmanRMatSet(&kalmanVel, rMatInit);  
    KalmanBMatSet(&kalmanVel, bMatInit);  
    KalmanCovarianceMatSet(&kalmanVel, pMatInit);    
    KalmanStateTransMatSet(&kalmanVel, fMatInit);
    KalmanObserveMapMatSet(&kalmanVel, hMatInit);
    
    //状态滑动窗口，用于解决卡尔曼状态估计量与观测量之间的相位差问题
    kalmanVel.slidWindowSize = 220;
    kalmanVel.statusSlidWindow = pvPortMalloc(kalmanVel.slidWindowSize * sizeof(kalmanVel.state));
    kalmanVel.fuseDelay.x = 220;    //0.22s延时
    kalmanVel.fuseDelay.y = 220;    //0.22s延时
    kalmanVel.fuseDelay.z = 200;    //0.2s延时
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
    kalmanPos.fuseDelay.z = 100;    //0.1s延时
}

/**********************************************************************************************************
*函 数 名: NavigationReset
*功能说明: 导航相关数据复位
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void NavigationReset(void)
{
    kalmanVel.state.x = 0;
    kalmanVel.state.y = 0;    
    kalmanVel.state.z = 0;    

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
Vector3f_t GetCopterVelMeasure(void)
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
