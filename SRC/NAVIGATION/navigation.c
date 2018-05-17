/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     navigation.c
 * @说明     组合导航，暂时先写个基本功能，后续还需要上机测试，以及性能优化。
 *           导航精度的提升，重点在于提高姿态估计的精度，并找到比较合理的误差模型，实现自适应融合
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

NAVGATION_t nav;
Kalman_t kalmanVel;

static void KalmanVelInit(void);

/**********************************************************************************************************
*函 数 名: NavigationInit
*功能说明: 导航参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void NavigationInit(void)
{
    KalmanVelInit();
}

/**********************************************************************************************************
*函 数 名: VelocityEstimate
*功能说明: 飞行速度估计 目前只融合了GPS与气压，以后还会有光流、TOF等模块数据的参与
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void VelocityEstimate(void)
{
	static uint32_t previousT;	
	float deltaT;
	Vector3f_t velMeasure;
    Vector3f_t input;
    static uint32_t count;
    static bool fuseFlag;
    static Vector3f_t velErrorInt;
    static float velErrorIntRate = 0.0001f;
    
    //计算时间间隔，用于积分
    deltaT = (GetSysTimeUs() - previousT) * 1e-6;
    deltaT = ConstrainFloat(deltaT, 0.0005, 0.002);	
	previousT = GetSysTimeUs();		
	
	//获取运动加速度
	nav.accel = GetCopterAccEf();
	
    //加速度数据更新频率1KHz，而气压数据更新频率只有50Hz，GPS数据只有10Hz
    //这里将气压与GPS参与融合的频率强制统一为50Hz
    if(count++ % 40 == 0)
    {
        if(GetGpsFixStatus())
        {
            //获取GPS速度测量值，转换速度值到机体坐标系
            TransVelToBodyFrame(GetGpsVelocity(), &velMeasure, GetCopterAngle().z);
        }
        else
        {
            velMeasure.x = 0;
            velMeasure.y = 0;
        }
        //获取气压速度测量值
        velMeasure.z = BaroGetVelocity();	
        
        fuseFlag = true;
    }
    else
    {
        fuseFlag = false;
    }

    //加速度积分，并转换单位为cm
    input.x = nav.accel.x * deltaT * 1000;
    input.y = nav.accel.y * deltaT * 1000;   
    input.z = nav.accel.z * deltaT * 1000;    
    
    //加速度值始终存在零偏误差，这里使用误差积分来修正零偏
    input.x += velErrorInt.x * 0.0001f;
    input.y += velErrorInt.y * 0.0001f;
    input.z += velErrorInt.z * 0.0001f;
    
    //卡尔曼滤波器更新
    KalmanUpdate(&kalmanVel, input, velMeasure, fuseFlag);
    nav.velocity = kalmanVel.status;
    
    //计算误差积分
    //为了提高效果，最好只在飞行器悬停的时候计算误差积分
    velErrorInt.x += (velMeasure.x - nav.velocity.x) * velErrorIntRate;
    velErrorInt.y += (velMeasure.y - nav.velocity.y) * velErrorIntRate;
    velErrorInt.z += (velMeasure.z - nav.velocity.z) * velErrorIntRate;
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
    float rMatInit[9] = {500, 0,  0, 0, 500, 0, 0, 0, 3000};
    float pMatInit[9] = {0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5};
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
}




































