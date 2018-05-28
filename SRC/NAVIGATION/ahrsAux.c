/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     ahrsAux.c
 * @说明     辅助姿态估计
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "ahrsAux.h"
#include "ahrs.h"
#include "board.h"
#include "kalman3.h"
#include "accelerometer.h"

AHRSAUX_t ahrsAux;
Kalman_t kalmanAux;

static void KalmanAuxInit(void);
static void AttitudeEstimateRollPitch(Vector3f_t deltaAngle, Vector3f_t acc);

/**********************************************************************************************************
*函 数 名: AHRSAuxInit
*功能说明: 辅助姿态估计参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void AHRSAuxInit(void)
{
    KalmanAuxInit();   
}

/**********************************************************************************************************
*函 数 名: AttitudeAuxInitAlignment
*功能说明: 辅助姿态初始对准
*形    参: 横滚俯仰的卡尔曼结构体指针 加速度测量值 
*返 回 值: 对准完成状态
**********************************************************************************************************/
int8_t AttitudeAuxInitAlignment(Kalman_t* rollPitch, Vector3f_t acc)
{
	static int16_t alignCnt = 200;
	static Vector3f_t accSum;
	static uint8_t alignFinishFlag = 0;
    
    if(alignFinishFlag)
    {
        return 1;
    }
    
	if(alignCnt > 0)
	{
		accSum.x += acc.x;
		accSum.y += acc.y;
		accSum.z += acc.z;
		
		alignCnt--;
		return 0;
	}
	else
	{
		rollPitch->status.x = accSum.x / 200;
		rollPitch->status.y = accSum.y / 200;
		rollPitch->status.z = accSum.z / 200;
		
        alignFinishFlag = 1;
        
		return 1;
	}
}

/**********************************************************************************************************
*函 数 名: AttitudeAuxEstimate
*功能说明: 辅助姿态估计
*形    参: 角速度测量值 加速度测量值
*返 回 值: 无
**********************************************************************************************************/
void AttitudeAuxEstimate(Vector3f_t gyro, Vector3f_t acc)
{
    Vector3f_t deltaAngle;	
	static uint32_t previousT;
	float deltaT = (GetSysTimeUs() - previousT) * 1e-6;	
    deltaT = ConstrainFloat(deltaT, 0.0005, 0.002);	
	previousT = GetSysTimeUs();		  

    //姿态初始对准
    if(!AttitudeAuxInitAlignment(&kalmanAux, acc))
        return;
    
	//计算角度变化量，单位为弧度
	deltaAngle.x = Radians(gyro.x * deltaT); 
	deltaAngle.y = Radians(gyro.y * deltaT); 
	deltaAngle.z = Radians(gyro.z * deltaT);	    
    
    //俯仰横滚角估计
    AttitudeEstimateRollPitch(deltaAngle, acc);
}

/**********************************************************************************************************
*函 数 名: KalmanAuxInit
*功能说明: 辅助姿态估计的卡尔曼结构体初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void KalmanAuxInit(void)
{
    float qMatInit[9] = {0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001};
    float rMatInit[9] = {1500, 0,  0, 0, 1500, 0, 0, 0, 1500};
    float pMatInit[9] = {0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5};
    float fMatInit[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float hMatInit[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float bMatInit[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

    //初始化卡尔曼滤波器的相关矩阵
    KalmanQMatSet(&kalmanAux, qMatInit);
    KalmanRMatSet(&kalmanAux, rMatInit);  
    KalmanBMatSet(&kalmanAux, bMatInit);  
    KalmanCovarianceMatSet(&kalmanAux, pMatInit);    
    KalmanStateTransMatSet(&kalmanAux, fMatInit);
    KalmanObserveMapMatSet(&kalmanAux, hMatInit);
}

/**********************************************************************************************************
*函 数 名: AttitudeEstimateRollPitch
*功能说明: 俯仰与横滚角估计
*形    参: 描述姿态转动的方向余弦矩阵 加速度测量值
*返 回 值: 无
**********************************************************************************************************/
static void AttitudeEstimateRollPitch(Vector3f_t deltaAngle, Vector3f_t acc)
{
    static Vector3f_t input = {0, 0, 0};
    float dcMat[9];
    static bool fuseFlag = true;
    
    if(abs(GetAccMag() - 1) > 0.15f)
    {
        fuseFlag = false;
    }
    else
    {
        fuseFlag = true;
    }
    
    //角度变化量转换为方向余弦矩阵
    EulerAngleToDCM(deltaAngle, dcMat);
    
    //更新卡尔曼状态转移矩阵
    KalmanStateTransMatSet(&kalmanAux, dcMat);
    
    //卡尔曼滤波器更新
    KalmanUpdate(&kalmanAux, input, acc, fuseFlag);
    ahrsAux.vectorRollPitch = kalmanAux.status;
    
	//转换成欧拉角
	ahrsAux.angle.x = Degrees(atan2f(ahrsAux.vectorRollPitch.y, Pythagorous2(ahrsAux.vectorRollPitch.x, ahrsAux.vectorRollPitch.z)));
	ahrsAux.angle.y = Degrees(atan2f(-ahrsAux.vectorRollPitch.x, ahrsAux.vectorRollPitch.z));
    
    //表示姿态误差
	ahrsAux.vectorRollPitchError.x = ahrsAux.vectorRollPitchError.x * 0.999f + (acc.x - ahrsAux.vectorRollPitch.x) * 0.001f;
	ahrsAux.vectorRollPitchError.y = ahrsAux.vectorRollPitchError.y * 0.999f + (acc.y - ahrsAux.vectorRollPitch.y) * 0.001f;
	ahrsAux.vectorRollPitchError.z = ahrsAux.vectorRollPitchError.z * 0.999f + (acc.z - ahrsAux.vectorRollPitch.z) * 0.001f;	

    //转化加速度到地理坐标系
	BodyFrameToEarthFrame(ahrsAux.angle, acc, &ahrsAux.accEf);

    //减去重力加速度(0,0,g)    
	ahrsAux.accEf.z = ahrsAux.accEf.z - GRAVITY_ACCEL;   
}

/**********************************************************************************************************
*函 数 名: GetSportAccBf
*功能说明: 获取运动加速度
*形    参: 无
*返 回 值: 加速度
**********************************************************************************************************/
Vector3f_t GetSportAccEf(void)
{
    return ahrsAux.accEf;
}

/**********************************************************************************************************
*函 数 名: GetAuxAngle
*功能说明: 获取表示飞行器姿态的欧拉角
*形    参: 无
*返 回 值: 角度值
**********************************************************************************************************/
Vector3f_t GetAuxAngle(void)
{
    return ahrsAux.angle;
}

