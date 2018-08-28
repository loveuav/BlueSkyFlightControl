/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     ahrs.c
 * @说明     姿态估计
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "ahrs.h"
#include "ahrsAux.h"
#include "board.h"
#include "kalman3.h"
#include "gps.h"
#include "flightStatus.h"
#include "accelerometer.h"
#include "navigation.h"

#include "FreeRTOS.h"
#include "task.h"

AHRS_t ahrs;
Kalman_t kalmanRollPitch, kalmanYaw;

static void AttitudeEstimateUpdate(Vector3f_t* angle, Vector3f_t gyro, Vector3f_t acc, Vector3f_t mag, float deltaT);
static void KalmanRollPitchInit(void);
static void KalmanYawInit(void);
static void TransAccToEarthFrame(Vector3f_t angle, Vector3f_t acc, Vector3f_t* accEf, Vector3f_t* accEfLpf, Vector3f_t* accBfOffset);
static Vector3f_t AccSportCompensate(Vector3f_t acc, Vector3f_t sportAccEf, Vector3f_t angle, Vector3f_t accBfOffset);
static void GyroEfUpdate(Vector3f_t gyro, Vector3f_t angle, Vector3f_t* gyroEf);
static void CentripetalAccUpdate(Vector3f_t* centripetalAcc, Vector3f_t velocity, float gyroYawEf);

/**********************************************************************************************************
*函 数 名: AHRSInit
*功能说明: 姿态估计参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void AHRSInit(void)
{
    KalmanRollPitchInit();
    KalmanYawInit();
}

/**********************************************************************************************************
*函 数 名: AttitudeInitAlignment
*功能说明: 姿态初始对准
*形    参: 横滚俯仰的卡尔曼结构体指针 航向的卡尔曼结构体指针 加速度测量值 磁场强度测量值
*返 回 值: 对准完成状态
**********************************************************************************************************/
int8_t AttitudeInitAlignment(Kalman_t* rollPitch, Kalman_t* yaw, Vector3f_t acc, Vector3f_t mag)
{
	static int16_t alignCnt = 200;
	static Vector3f_t accSum, magSum;
	static uint8_t alignFinishFlag = 0;
    
    if(alignFinishFlag)
    {
        return 1;
    }
    
	if(alignCnt > 0)
	{
        //传感器采样值累加
        accSum = Vector3f_Add(accSum, acc);	
		magSum = Vector3f_Add(magSum, mag);

		alignCnt--;
		return 0;
	}
	else
	{
        //求平均值
		rollPitch->state.x = accSum.x / 200;
		rollPitch->state.y = accSum.y / 200;
		rollPitch->state.z = accSum.z / 200;

		yaw->state.x = magSum.x / 200;
		yaw->state.y = magSum.y / 200;		
		yaw->state.z = magSum.z / 200;
		
        alignFinishFlag = 1;
        
		return 1;
	}
}

/**********************************************************************************************************
*函 数 名: AttitudeEstimate
*功能说明: 姿态估计
*形    参: 角速度测量值 加速度测量值 磁场强度测量值
*返 回 值: 无
**********************************************************************************************************/
void AttitudeEstimate(Vector3f_t gyro, Vector3f_t acc, Vector3f_t mag)
{
    Vector3f_t accCompensate;
	static uint64_t previousT;
	float deltaT = (GetSysTimeUs() - previousT) * 1e-6;	
    deltaT = ConstrainFloat(deltaT, 0.0005, 0.002);	
	previousT = GetSysTimeUs();		  

    //姿态初始对准
    if(!AttitudeInitAlignment(&kalmanRollPitch, &kalmanYaw, acc, mag))
        return;
    
    //运动加速度补偿
    accCompensate = AccSportCompensate(acc, GetSportAccEf(), ahrs.angle, ahrs.accBfOffset);

    //向心加速度误差补偿
    accCompensate = Vector3f_Sub(accCompensate, ahrs.centripetalAccBf);  
    
    //加速度零偏补偿
    accCompensate = Vector3f_Sub(accCompensate, ahrs.accBfOffset);
    
    //姿态更新
    AttitudeEstimateUpdate(&ahrs.angle, gyro, accCompensate, mag, deltaT);
    
    //计算导航系下的运动加速度
    TransAccToEarthFrame(ahrs.angle, acc, &ahrs.accEf, &ahrs.accEfLpf, &ahrs.accBfOffset);
    
    //转换角速度至导航系
    GyroEfUpdate(gyro, ahrs.angle, &ahrs.gyroEf);
    
    //计算导航系下的向心加速度
    CentripetalAccUpdate(&ahrs.centripetalAcc, GetCopterVelocity(), ahrs.gyroEf.z);
    
    //转换向心加速度至机体系，用于姿态更新补偿
    EarthFrameToBodyFrame(ahrs.angle, ahrs.centripetalAcc, &ahrs.centripetalAccBf);
}

/**********************************************************************************************************
*函 数 名: KalmanRollPitchInit
*功能说明: 俯仰横滚估计的卡尔曼结构体初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void KalmanRollPitchInit(void)
{
    float qMatInit[9] = {0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001};
    float rMatInit[9] = {3500, 0,  0, 0, 3500, 0, 0, 0, 3500};
    float pMatInit[9] = {0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5};
    float fMatInit[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float hMatInit[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float bMatInit[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

    //初始化卡尔曼滤波器的相关矩阵
    KalmanQMatSet(&kalmanRollPitch, qMatInit);
    KalmanRMatSet(&kalmanRollPitch, rMatInit);  
    KalmanBMatSet(&kalmanRollPitch, bMatInit);  
    KalmanCovarianceMatSet(&kalmanRollPitch, pMatInit);    
    KalmanStateTransMatSet(&kalmanRollPitch, fMatInit);
    KalmanObserveMapMatSet(&kalmanRollPitch, hMatInit);
    
    //状态滑动窗口，用于解决卡尔曼状态估计量与观测量之间的相位差问题
    kalmanRollPitch.slidWindowSize = 1;
    kalmanRollPitch.statusSlidWindow = pvPortMalloc(kalmanRollPitch.slidWindowSize * sizeof(kalmanRollPitch.state));
    kalmanRollPitch.fuseDelay.x = 1;
    kalmanRollPitch.fuseDelay.y = 1;
    kalmanRollPitch.fuseDelay.z = 1;
}

/**********************************************************************************************************
*函 数 名: KalmanYawInit
*功能说明: 航向估计的卡尔曼结构体初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void KalmanYawInit(void)
{
    float qMatInit[9] = {0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001};
    float rMatInit[9] = {2500, 0,  0, 0, 2500, 0, 0, 0, 2500};
    float pMatInit[9] = {0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5};
    float fMatInit[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float hMatInit[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float bMatInit[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

    //初始化卡尔曼滤波器的相关矩阵
    KalmanQMatSet(&kalmanYaw, qMatInit);
    KalmanRMatSet(&kalmanYaw, rMatInit);  
    KalmanBMatSet(&kalmanYaw, bMatInit);  
    KalmanCovarianceMatSet(&kalmanYaw, pMatInit);    
    KalmanStateTransMatSet(&kalmanYaw, fMatInit);
    KalmanObserveMapMatSet(&kalmanYaw, hMatInit);
 
    //状态滑动窗口，用于解决卡尔曼状态估计量与观测量之间的相位差问题
    kalmanYaw.slidWindowSize = 1;
    kalmanYaw.statusSlidWindow = pvPortMalloc(kalmanYaw.slidWindowSize * sizeof(kalmanYaw.state));
    kalmanYaw.fuseDelay.x = 1;
    kalmanYaw.fuseDelay.y = 1;
    kalmanYaw.fuseDelay.z = 1;
}

/**********************************************************************************************************
*函 数 名: AttitudeEstimateUpdate
*功能说明: 姿态更新
*形    参: 姿态角指针 角速度 加速度测量值 磁力计测量值 时间间隔
*返 回 值: 无
**********************************************************************************************************/
static void AttitudeEstimateUpdate(Vector3f_t* angle, Vector3f_t gyro, Vector3f_t acc, Vector3f_t mag, float deltaT)
{
    Vector3f_t deltaAngle;
    static Vector3f_t gError;	
    static Vector3f_t gyro_bias = {0, 0, 0};    //陀螺仪零偏
    static Vector3f_t input = {0, 0, 0};
    float dcMat[9];
    Vector3f_t mVectorEf;
    static uint32_t count = 0;
    
    //修正陀螺仪零偏
    gyro.x -= gyro_bias.x; 
    gyro.y -= gyro_bias.y; 
    gyro.z -= gyro_bias.z; 
    
    //一阶积分计算角度变化量，单位为弧度
	deltaAngle.x = Radians(gyro.x * deltaT); 
	deltaAngle.y = Radians(gyro.y * deltaT); 
	deltaAngle.z = Radians(gyro.z * deltaT); 
    
    //角度变化量转换为方向余弦矩阵
    EulerAngleToDCM(deltaAngle, dcMat);
    
    //更新卡尔曼状态转移矩阵
    KalmanStateTransMatSet(&kalmanRollPitch, dcMat);
    KalmanStateTransMatSet(&kalmanYaw, dcMat);
    
    //卡尔曼滤波器更新
    //磁强数据更新频率要低于陀螺仪，因此磁强数据未更新时只进行状态预估计
    KalmanUpdate(&kalmanRollPitch, input, acc, true);
    KalmanUpdate(&kalmanYaw, input, mag, count++ % 10 == 0?true:false);   
    
	//计算俯仰与横滚角
    AccVectorToRollPitchAngle(angle, kalmanRollPitch.state);
	angle->x = Degrees(angle->x);
	angle->y = Degrees(angle->y);
    
    //计算偏航角，并修正磁偏角误差
    //磁偏角东偏为正，西偏为负，中国除新疆外大部分地区为西偏，比如深圳地区为-2°左右
	BodyFrameToEarthFrame(*angle, kalmanYaw.state, &mVectorEf);
    MagVectorToYawAngle(angle, mVectorEf);
	angle->z = WrapDegree360(Degrees(angle->z) + GetMagDeclination());      
    
	//向量观测值与估计值进行叉积运算得到旋转误差矢量
	gError = VectorCrossProduct(acc, kalmanRollPitch.state);
    BodyFrameToEarthFrame(*angle, gError, &gError);
    
    //计算偏航误差
    float magAngle;
    BodyFrameToEarthFrame(*angle, mag, &mVectorEf);
    magAngle = WrapDegree360(Degrees(atan2f(-mVectorEf.y, mVectorEf.x)) + GetMagDeclination());   
    if(abs(angle->z - magAngle) < 10 && abs(angle->x) < 10 && abs(angle->y)< 10)
    {
       gError.z = Radians(angle->z - magAngle);
    }
    else
    {
        gError.z = 0;
    }

    //陀螺仪零偏估计
    gyro_bias.x += (gError.x * deltaT) * 0.2f;
    gyro_bias.y += (gError.y * deltaT) * 0.2f;
    gyro_bias.z += (gError.z * deltaT) * 0.05f;
    
    //陀螺仪零偏限幅
    gyro_bias.x = ConstrainFloat(gyro_bias.x, -1.0f, 1.0f);
    gyro_bias.y = ConstrainFloat(gyro_bias.y, -1.0f, 1.0f);   
    gyro_bias.z = ConstrainFloat(gyro_bias.z, -1.0f, 1.0f); 
	
    /************************************近似计算姿态角误差，用于观察和调试**********************************/
    Vector3f_t angleObserv;
    AccVectorToRollPitchAngle(&angleObserv, acc);
	ahrs.angleError.x = ahrs.angleError.x * 0.999f + (angle->x - Degrees(angleObserv.x)) * 0.001f;
	ahrs.angleError.y = ahrs.angleError.y * 0.999f + (angle->y - Degrees(angleObserv.y)) * 0.001f;  
    
    Vector3f_t magEf;    
    BodyFrameToEarthFrame(*angle, mag, &magEf);
    angleObserv.z = WrapDegree360(Degrees(atan2f(-magEf.y, magEf.x)) + GetMagDeclination());  
    ahrs.angleError.z = ahrs.angleError.z * 0.999f + (angle->z - angleObserv.z) * 0.001f;  
    /********************************************************************************************************/
}

/**********************************************************************************************************
*函 数 名: AttCovarianceSelfAdaptation
*功能说明: 姿态观测误差协方差自适应
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void AttCovarianceSelfAdaptation(void)
{  
    float accelMag = Pythagorous2(GetCopterAccEfLpf().x, GetCopterAccEfLpf().y);
    
    kalmanRollPitch.r[0] = Sq(45 * (1 + ConstrainFloat(accelMag * 10, 0, 9)));
    kalmanRollPitch.r[4] = Sq(45 * (1 + ConstrainFloat(accelMag * 10, 0, 9)));	
    kalmanRollPitch.r[8] = Sq(45 * (1 + ConstrainFloat(accelMag * 10, 0, 9)));
}

/**********************************************************************************************************
*函 数 名: BodyFrameToEarthFrame
*功能说明: 转换向量到地理坐标系
*形    参: 转动角度 转动向量 转动后的向量指针
*返 回 值: 无
**********************************************************************************************************/
void BodyFrameToEarthFrame(Vector3f_t angle, Vector3f_t vector, Vector3f_t* vectorEf)
{
	Vector3f_t anglerad;

	anglerad.x = Radians(angle.x);
	anglerad.y = Radians(angle.y);
	anglerad.z = 0;    
    *vectorEf  = VectorRotateToEarthFrame(vector, anglerad); 
}

/**********************************************************************************************************
*函 数 名: EarthFrameToBodyFrame
*功能说明: 转换向量到机体坐标系
*形    参: 转动角度 转动向量 转动后的向量指针
*返 回 值: 无
**********************************************************************************************************/
void EarthFrameToBodyFrame(Vector3f_t angle, Vector3f_t vector, Vector3f_t* vectorBf)
{
	Vector3f_t anglerad;
	
	anglerad.x = Radians(angle.x);
	anglerad.y = Radians(angle.y);
	anglerad.z = 0;
    *vectorBf  = VectorRotateToBodyFrame(vector, anglerad);
}

/**********************************************************************************************************
*函 数 名: TransAccToEarthFrame
*功能说明: 转换加速度到地理坐标系，并去除重力加速度
*形    参: 当前飞机姿态 加速度 地理系加速度 地理系加速度低通滤波 加速度零偏
*返 回 值: 无
**********************************************************************************************************/
static void TransAccToEarthFrame(Vector3f_t angle, Vector3f_t acc, Vector3f_t* accEf, Vector3f_t* accEfLpf, Vector3f_t* accBfOffset)
{
	static uint16_t offset_cnt = 8000;	//计算零偏的次数
    static Vector3f_t accAngle;   //用于计算初始零偏
    Vector3f_t gravityBf;
    
    //即使经过校准并对传感器做了恒温处理，加速度的零偏误差还是存在不稳定性，即相隔一定时间后再上电加速度零偏会发生变化
    //由于加速度零偏对导航积分计算影响较大，因此每次上电工作都需要计算零偏并补偿
    //计算初始零偏时，直接使用加速度值计算角度，因为飞控初始化时角度估计值存在误差，导致计算出来的零偏有误差   
    
    if(GetInitStatus() == INIT_FINISH)
    {
        //计算重力加速度在机体坐标系的投影
        gravityBf.x = 0;
        gravityBf.y = 0;    
        gravityBf.z = 1;
        EarthFrameToBodyFrame(angle, gravityBf, &gravityBf);
        
        //减去重力加速度和加速度零偏
        acc.x -= (gravityBf.x + accBfOffset->x);
        acc.y -= (gravityBf.y + accBfOffset->y);
        acc.z -= (gravityBf.z + accBfOffset->z);

        //转化加速度到地理坐标系
        BodyFrameToEarthFrame(angle, acc, accEf);
        
        //向心加速度误差补偿
        accEf->x -= ahrs.centripetalAcc.x;
        accEf->y -= ahrs.centripetalAcc.y;
        
		//地理系加速度低通滤波
		accEfLpf->x = accEfLpf->x * 0.998f + accEf->x * 0.002f;
		accEfLpf->y = accEfLpf->y * 0.998f + accEf->y * 0.002f;
		accEfLpf->z = accEfLpf->z * 0.998f + accEf->z * 0.002f; 
    }
    
	//系统初始化时，计算加速度零偏
	if(GetSysTimeMs() > 5000 && GetInitStatus() == HEAT_FINISH)
	{
        //飞机静止时才进行零偏计算
		if(GetPlaceStatus() == STATIC)
		{
            //直接使用加速度数据计算姿态角
            AccVectorToRollPitchAngle(&accAngle, acc);
            accAngle.x = Degrees(accAngle.x);
            accAngle.y = Degrees(accAngle.y);
            
            //转换重力加速度到机体坐标系并计算零偏误差
            gravityBf.x = 0;
            gravityBf.y = 0;    
            gravityBf.z = 1;
            EarthFrameToBodyFrame(accAngle, gravityBf, &gravityBf);
            
			accBfOffset->x = accBfOffset->x * 0.998f + (acc.x - gravityBf.x) * 0.002f;
			accBfOffset->y = accBfOffset->y * 0.998f + (acc.y - gravityBf.y) * 0.002f; 
			accBfOffset->z = accBfOffset->z * 0.998f + (acc.z - gravityBf.z) * 0.002f; 
			offset_cnt--;
		}
		else
		{
			//计算过程中如果出现晃动，则重新开始
			offset_cnt 	   = 8000;
			accBfOffset->x = 0;
			accBfOffset->y = 0;
			accBfOffset->z = 0;
		}
        //完成零偏计算，系统初始化结束
        if(offset_cnt == 0)
            SetInitStatus(INIT_FINISH);
	}
}

/**********************************************************************************************************
*函 数 名: AccSportCompensate
*功能说明: 运动加速度补偿
*形    参: 加速度 地理系运动加速度 姿态角 机体系加速度零偏
*返 回 值: 经过补偿的加速度
**********************************************************************************************************/
static Vector3f_t AccSportCompensate(Vector3f_t acc, Vector3f_t sportAccEf, Vector3f_t angle, Vector3f_t accBfOffset)
{
    Vector3f_t sportAccBf;
    
    //转换运动加速度到机体坐标系   
    EarthFrameToBodyFrame(angle, sportAccEf, &sportAccBf);
    
    //减去加速度零偏
    sportAccBf = Vector3f_Sub(sportAccBf, accBfOffset);
    
    //应用死区
    sportAccBf.x = ApplyDeadbandFloat(sportAccBf.x, 0.03f);
    sportAccBf.y = ApplyDeadbandFloat(sportAccBf.y, 0.03f);
    sportAccBf.z = ApplyDeadbandFloat(sportAccBf.z, 0.03f);
    
    //补偿到姿态估计主回路中的加速度
    acc.x = acc.x - sportAccBf.x * 0.95f;
    acc.y = acc.y - sportAccBf.y * 0.95f;
    acc.z = acc.z - sportAccBf.z * 0.95f;   

    return acc;
}

/**********************************************************************************************************
*函 数 名: GyroEfUpdate
*功能说明: 计算机体的地理系角速度
*形    参: 角速度 机体角度 地理系角速度指针
*返 回 值: 无
**********************************************************************************************************/
static void GyroEfUpdate(Vector3f_t gyro, Vector3f_t angle, Vector3f_t* gyroEf)
{
    BodyFrameToEarthFrame(angle, gyro, gyroEf);
}

/**********************************************************************************************************
*函 数 名: CentripetalAccUpdate
*功能说明: 计算圆周运动时产生的向心加速度误差
*形    参: 向心加速度指针 飞行速度 地理系的z轴加速度
*返 回 值: 无
**********************************************************************************************************/
static void CentripetalAccUpdate(Vector3f_t* centripetalAcc, Vector3f_t velocity, float gyroYawEf)
{    
    if(GpsGetFixStatus() == true)
    {
        centripetalAcc->x = velocity.y * 0.01f * Radians(gyroYawEf) / GRAVITY_ACCEL;
        centripetalAcc->y = velocity.x * 0.01f * Radians(gyroYawEf) / GRAVITY_ACCEL;
        centripetalAcc->z = 0;
    }
    else
    {
        centripetalAcc->x = 0;
        centripetalAcc->y = 0;
        centripetalAcc->z = 0;
    }
    
    centripetalAcc->x = ConstrainFloat(centripetalAcc->x, -1, 1);
    centripetalAcc->y = ConstrainFloat(centripetalAcc->y, -1, 1);
    centripetalAcc->z = ConstrainFloat(centripetalAcc->z, -1, 1);    
}

/**********************************************************************************************************
*函 数 名: GetCopterAccEf
*功能说明: 获取地理坐标系下的运动加速度
*形    参: 无
*返 回 值: 加速度
**********************************************************************************************************/
Vector3f_t GetCopterAccEf(void)
{
    return ahrs.accEf;
}

/**********************************************************************************************************
*函 数 名: GetCopterAccEfLpf
*功能说明: 获取地理系运动加速度的低通滤波值
*形    参: 无
*返 回 值: 加速度
**********************************************************************************************************/
Vector3f_t GetCopterAccEfLpf(void)
{
    return ahrs.accEfLpf;
}

/**********************************************************************************************************
*函 数 名: GetCopterAngle
*功能说明: 获取表示飞行器姿态的欧拉角
*形    参: 无
*返 回 值: 角度值
**********************************************************************************************************/
Vector3f_t GetCopterAngle(void)
{
    return ahrs.angle;
}

/**********************************************************************************************************
*函 数 名: GetCentripetalAcc
*功能说明: 获取向心加速度
*形    参: 无
*返 回 值: 向心加速度
**********************************************************************************************************/
Vector3f_t GetCentripetalAcc(void)
{
    return ahrs.centripetalAcc;
}

/**********************************************************************************************************
*函 数 名: GetCentripetalAccBf
*功能说明: 获取机体系向心加速度误差
*形    参: 无
*返 回 值: 向心加速度
**********************************************************************************************************/
Vector3f_t GetCentripetalAccBf(void)
{
    return ahrs.centripetalAccBf;
}

