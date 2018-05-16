/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     ahrs.c
 * @说明     姿态估计，使用卡尔曼滤波。后续更新中将对运动加速度进行补偿，并使用自适应融合，提升姿态动态精度
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "ahrs.h"
#include "board.h"
#include "kalman3.h"
#include "gps.h"

AHRS_t ahrs;
Kalman_t kalmanRollPitch, kalmanYaw;

static void AttitudeEstimateRollPitch(Vector3f_t deltaAngle, Vector3f_t acc);
static void AttitudeEstimateYaw(Vector3f_t deltaAngle, Vector3f_t mag);
static void kalmanRollPitchInit(void);
static void kalmanYawInit(void);
static void BodyFrameToEarthFrame(Vector3f_t angle, Vector3f_t vector, Vector3f_t* vectorEf);
static void EarthFrameToBodyFrame(Vector3f_t angle, Vector3f_t vector, Vector3f_t* vectorBf);
static void TransAccToEarthFrame(Vector3f_t angle, Vector3f_t acc, Vector3f_t* accEf);

/**********************************************************************************************************
*函 数 名: AHRSInit
*功能说明: 姿态估计参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void AHRSInit(void)
{
    kalmanRollPitchInit();
    kalmanYawInit();
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
		accSum.x += acc.x;
		accSum.y += acc.y;
		accSum.z += acc.z;
		
		magSum.x += mag.x;
		magSum.y += mag.y;
		magSum.z += mag.z;
		
		alignCnt--;
		return 0;
	}
	else
	{
		rollPitch->status.x = accSum.x / 200;
		rollPitch->status.y = accSum.y / 200;
		rollPitch->status.z = accSum.z / 200;

		yaw->status.x = magSum.x / 200;
		yaw->status.y = magSum.y / 200;		
		yaw->status.z = magSum.z / 200;
		
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
    Vector3f_t deltaAngle;	
	static uint32_t previousT;
	float deltaT = (GetSysTimeUs() - previousT) * 1e-6;	
	previousT = GetSysTimeUs();		  

    //姿态初始对准
    if(!AttitudeInitAlignment(&kalmanRollPitch, &kalmanYaw, acc, mag))
        return;
    
	//计算角度变化量，单位为弧度
	deltaAngle.x = Radians(gyro.x * deltaT); 
	deltaAngle.y = Radians(gyro.y * deltaT); 
	deltaAngle.z = Radians(gyro.z * deltaT);	    
    
    //俯仰横滚角估计
    AttitudeEstimateRollPitch(deltaAngle, acc);
    
    //偏航角估计
    AttitudeEstimateYaw(deltaAngle, mag);
    
    //计算飞行器在地理坐标系下的运动加速度
    TransAccToEarthFrame(ahrs.angle, acc, &ahrs.accEf);
}

/**********************************************************************************************************
*函 数 名: kalmanRollPitchInit
*功能说明: 俯仰横滚估计的卡尔曼结构体初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void kalmanRollPitchInit(void)
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
    
    //陀螺仪积分补偿系数
    ahrs.vectorRollPitchKI = 0.00001f;
}

/**********************************************************************************************************
*函 数 名: kalmanYawInit
*功能说明: 航向估计的卡尔曼结构体初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void kalmanYawInit(void)
{
    float qMatInit[9] = {0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001};
    float rMatInit[9] = {3000, 0,  0, 0, 3000, 0, 0, 0, 3000};
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
    
    //陀螺仪积分补偿系数
    ahrs.vectorYawKI = 0.00001f;
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
 	static Vector3f_t vectorError;	
	static float vectorErrorIntRate = 0.0002f;

	//用向量叉积误差积分来补偿陀螺仪零偏噪声
	deltaAngle.x += ahrs.vectorRollPitchErrorInt.x * ahrs.vectorRollPitchKI;
	deltaAngle.y += ahrs.vectorRollPitchErrorInt.y * ahrs.vectorRollPitchKI;	
	deltaAngle.z += ahrs.vectorRollPitchErrorInt.z * ahrs.vectorRollPitchKI;
    
    //角度变化量转换为方向余弦矩阵
    EulerAngleToDCM(deltaAngle, dcMat);
    
    //更新卡尔曼状态转移矩阵
    KalmanStateTransMatSet(&kalmanRollPitch, dcMat);
    
    //卡尔曼滤波器更新
    KalmanUpdate(&kalmanRollPitch, input, acc, true);
    ahrs.vectorRollPitch = kalmanRollPitch.status;
    
	//转换成欧拉角
	ahrs.angle.x = Degrees(atan2f(ahrs.vectorRollPitch.y, Pythagorous2(ahrs.vectorRollPitch.x, ahrs.vectorRollPitch.z)));
	ahrs.angle.y = Degrees(atan2f(-ahrs.vectorRollPitch.x, ahrs.vectorRollPitch.z));
    
	//加速度观测值与姿态估计值进行叉积运算得到旋转误差矢量
	vectorError = VectorCrossProduct(acc, ahrs.vectorRollPitch);
    
	//旋转误差矢量积分
	ahrs.vectorRollPitchErrorInt.x += vectorError.x * vectorErrorIntRate;
	ahrs.vectorRollPitchErrorInt.y += vectorError.y * vectorErrorIntRate;
	ahrs.vectorRollPitchErrorInt.z += vectorError.z * vectorErrorIntRate;
    //积分限幅
	ahrs.vectorRollPitchErrorInt.x = ConstrainFloat(ahrs.vectorRollPitchErrorInt.x, -0.5f, 0.5f);
	ahrs.vectorRollPitchErrorInt.y = ConstrainFloat(ahrs.vectorRollPitchErrorInt.y, -0.5f, 0.5f);
	ahrs.vectorRollPitchErrorInt.z = ConstrainFloat(ahrs.vectorRollPitchErrorInt.z, -0.3f, 0.3f);    
	
    //用于调试观察
	ahrs.vectorRollPitchError.x = ahrs.vectorRollPitchError.x * 0.999f + (acc.x - ahrs.vectorRollPitch.x) * 0.001f;
	ahrs.vectorRollPitchError.y = ahrs.vectorRollPitchError.y * 0.999f + (acc.y - ahrs.vectorRollPitch.y) * 0.001f;
	ahrs.vectorRollPitchError.z = ahrs.vectorRollPitchError.z * 0.999f + (acc.z - ahrs.vectorRollPitch.z) * 0.001f;	    
    
}

/**********************************************************************************************************
*函 数 名: AttitudeEstimateYaw
*功能说明: 航向角估计
*形    参: 描述姿态转动的方向余弦矩阵 磁场强度测量值
*返 回 值: 无
**********************************************************************************************************/
static void AttitudeEstimateYaw(Vector3f_t deltaAngle, Vector3f_t mag)
{
    static Vector3f_t input = {0, 0, 0};
    float dcMat[9];
    static bool fuseFlag = true;
    static uint32_t count = 0;
    Vector3f_t vectorYawEf;
 	static Vector3f_t vectorError;	
	static float vectorErrorIntRate = 0.0002f;
    
    //磁强数据更新频率要低于陀螺仪，因此磁强数据未更新时只进行状态预估计
    //陀螺仪更新频率1KHz，磁力计更新频率100Hz
    if(count % 10 == 0)
    {
        fuseFlag = true;
    }
    else
    {
        fuseFlag = false;
    }
    count++;
    
    //用向量叉积误差积分来补偿陀螺仪零偏噪声
	deltaAngle.x += ahrs.vectorYawErrorInt.x * ahrs.vectorYawKI;
	deltaAngle.y += ahrs.vectorYawErrorInt.y * ahrs.vectorYawKI;	
	deltaAngle.z += ahrs.vectorYawErrorInt.z * ahrs.vectorYawKI;
    
    //角度变化量转换为方向余弦矩阵
    EulerAngleToDCM(deltaAngle, dcMat);
    
    //更新卡尔曼状态转移矩阵    
    KalmanStateTransMatSet(&kalmanYaw, dcMat);

    //卡尔曼滤波器更新    
    KalmanUpdate(&kalmanYaw, input, mag, fuseFlag);   
	ahrs.vectorYaw = kalmanYaw.status;  
    
	//转换磁场向量估计量到地理坐标系
	BodyFrameToEarthFrame(ahrs.angle, ahrs.vectorYaw, &vectorYawEf);
    
	//转换成欧拉角，并减去磁偏角误差
	ahrs.angle.z = Degrees(atan2f(vectorYawEf.y, vectorYawEf.x)) - GetMagDeclination();    
    
    //将航向角限制为0-360°
    ahrs.angle.z = WrapDegree360(ahrs.angle.z);
    
	//磁强观测值与航向估计值进行叉积运算得到旋转误差矢量
	vectorError = VectorCrossProduct(mag, ahrs.vectorYaw);
    
	//旋转误差矢量积分
	ahrs.vectorYawErrorInt.x += vectorError.x * vectorErrorIntRate;
	ahrs.vectorYawErrorInt.y += vectorError.y * vectorErrorIntRate;
	ahrs.vectorYawErrorInt.z += vectorError.z * vectorErrorIntRate;
    //积分限幅
	ahrs.vectorYawErrorInt.x = ConstrainFloat(ahrs.vectorYawErrorInt.x, -0.3f, 0.3f);
	ahrs.vectorYawErrorInt.y = ConstrainFloat(ahrs.vectorYawErrorInt.y, -0.3f, 0.3f);
	ahrs.vectorYawErrorInt.z = ConstrainFloat(ahrs.vectorYawErrorInt.z, -0.3f, 0.3f);    
	
    //用于调试观察
	ahrs.vectorYawError.x = ahrs.vectorYawError.x * 0.999f + (mag.x - ahrs.vectorYaw.x) * 0.001f;
	ahrs.vectorYawError.y = ahrs.vectorYawError.y * 0.999f + (mag.y - ahrs.vectorYaw.y) * 0.001f;
	ahrs.vectorYawError.z = ahrs.vectorYawError.z * 0.999f + (mag.z - ahrs.vectorYaw.z) * 0.001f;	 
}

/**********************************************************************************************************
*函 数 名: BodyFrameToEarthFrame
*功能说明: 转换向量到地理坐标系
*形    参: 转动角度 转动向量 转动后的向量指针
*返 回 值: 无
**********************************************************************************************************/
static void BodyFrameToEarthFrame(Vector3f_t angle, Vector3f_t vector, Vector3f_t* vectorEf)
{
	Vector3f_t anglerad;
	
	anglerad.x = 0;
	anglerad.y = -Radians(angle.y);
	anglerad.z = 0;
	vector = VectorRotate(vector, anglerad);	
	
	anglerad.x = -Radians(angle.x);
	anglerad.y = 0;
	vector = VectorRotate(vector, anglerad);
    
    *vectorEf = vector;
}

/**********************************************************************************************************
*函 数 名: BodyFrameToEarthFrame
*功能说明: 转换向量到机体坐标系
*形    参: 转动角度 转动向量 转动后的向量指针
*返 回 值: 无
**********************************************************************************************************/
static void EarthFrameToBodyFrame(Vector3f_t angle, Vector3f_t vector, Vector3f_t* vectorBf)
{
	Vector3f_t anglerad;
	
	anglerad.x = Radians(angle.x);
	anglerad.y = Radians(angle.y);
	anglerad.z = 0;
    vector = VectorRotate(vector, anglerad);
    *vectorBf = vector;
}

/**********************************************************************************************************
*函 数 名: TransAccToEarthFrame
*功能说明: 转换加速度到地理坐标系，并去除重力加速度
*形    参: 当前飞机姿态 加速度 地理坐标系下的加速度
*返 回 值: 无
**********************************************************************************************************/
static void TransAccToEarthFrame(Vector3f_t angle, Vector3f_t acc, Vector3f_t* accEf)
{
    //后续更新还需要在飞控初始化时计算加速度零偏
    //即使经过校准并对传感器做了恒温处理，加速度的零偏误差还是存在不稳定性，即相隔一定时间后再上电加速度零偏会发生变化
    //由于加速度零偏对导航积分计算影响较大，因此每次上电工作都需要计算零偏并补偿
    
	BodyFrameToEarthFrame(angle, acc, accEf);
	
	ahrs.accEf.y = -ahrs.accEf.y;	//转换坐标系（西北天）到东北天
	ahrs.accEf.z = ahrs.accEf.z - GRAVITY_ACCEL;    //减去重力加速度(0,0,g)    
}

/**********************************************************************************************************
*函 数 名: GetAccEf
*功能说明: 获取地理坐标系下的运动加速度
*形    参: 无
*返 回 值: 加速度
**********************************************************************************************************/
Vector3f_t GetMotionAccEf(void)
{
    return ahrs.accEf;
}













































