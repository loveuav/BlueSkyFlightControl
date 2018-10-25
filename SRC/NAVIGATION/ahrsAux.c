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
#include "quaternion.h"
#include "accelerometer.h"

#include "FreeRTOS.h"
#include "task.h"

//姿态估计算法选择，目前基于四元数的算法还存在问题
#define AHRS_AUX_USE_DCM_KF
//#define AHRS_AUX_USE_QUATERNION_CF
//#define AHRS_AUX_USE_QUATERNION_KF

AHRSAUX_t ahrsAux;
Kalman_t kalmanAux;

static void KalmanAuxInit(void);

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
*形    参: 横滚俯仰的卡尔曼结构体指针 四元数 加速度测量值 磁力计测量值
*返 回 值: 对准完成状态
**********************************************************************************************************/
int8_t AttitudeAuxInitAlignment(Kalman_t* kalman, float q[4],Vector3f_t acc, Vector3f_t mag)
{
    static int16_t alignCnt = 200;
    static Vector3f_t accSum, magSum;
    static uint8_t alignFinishFlag = 0;
    Vector3f_t angle;

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
        //计算平均值
        accSum.x = accSum.x / 200;
        accSum.y = accSum.y / 200;
        accSum.z = accSum.z / 200;

        magSum.x = magSum.x / 200;
        magSum.y = magSum.y / 200;
        magSum.z = magSum.z / 200;

        //卡尔曼状态量初始化
        kalman->state.x = accSum.x;
        kalman->state.y = accSum.y;
        kalman->state.z = accSum.z;

        //计算姿态欧拉角
        AccVectorToRollPitchAngle(&angle, accSum);
        magSum = VectorRotateToEarthFrame(magSum, angle);
        MagVectorToYawAngle(&angle, magSum);

        //欧拉角转四元数
        EulerAngleToQuaternion(angle, q);

        alignFinishFlag = 1;

        return 1;
    }
}

/**********************************************************************************************************
*函 数 名: AttitudeAuxEstimate
*功能说明: 辅助姿态估计
*形    参: 角速度测量值 加速度测量值 磁力计测量值
*返 回 值: 无
**********************************************************************************************************/
void AttitudeAuxEstimate(Vector3f_t gyro, Vector3f_t acc, Vector3f_t mag)
{
    static uint64_t previousT;
    float deltaT = (GetSysTimeUs() - previousT) * 1e-6;
    deltaT = ConstrainFloat(deltaT, 0.0005, 0.002);
    previousT = GetSysTimeUs();

    //姿态初始对准
    if(!AttitudeAuxInitAlignment(&kalmanAux, ahrsAux.q, acc, mag))
        return;

    //补偿向心加速度误差
    acc = Vector3f_Sub(acc, GetCentripetalAccBf());

    //姿态估计
#ifdef AHRS_AUX_USE_DCM_KF
    RollPitchUpdateByKF(&ahrsAux.angle, gyro, acc, deltaT);
#endif
#ifdef AHRS_AUX_USE_QUATERNION_CF
    QuaternionUpdateByCF(ahrsAux.q, &ahrsAux.angle, gyro, acc, mag, deltaT);
#endif

    //转化加速度到地理坐标系
    BodyFrameToEarthFrame(ahrsAux.angle, acc, &ahrsAux.accEf);

    //减去重力加速度(0,0,g)
    ahrsAux.accEf.z = ahrsAux.accEf.z - 1;
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
    float rMatInit[9] = {3500, 0,  0, 0, 3500, 0, 0, 0, 3500};
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

    //状态滑动窗口，用于解决卡尔曼状态估计量与观测量之间的相位差问题
    kalmanAux.slidWindowSize = 1;
    kalmanAux.statusSlidWindow = pvPortMalloc(kalmanAux.slidWindowSize * sizeof(kalmanAux.state));
    kalmanAux.fuseDelay.x = 1;
    kalmanAux.fuseDelay.y = 1;
    kalmanAux.fuseDelay.z = 1;
}

/**********************************************************************************************************
*函 数 名: AttitudeEstimateRollPitch
*功能说明: 俯仰与横滚角估计
*形    参: 姿态角指针 角速度 加速度测量值 时间间隔
*返 回 值: 无
**********************************************************************************************************/
void RollPitchUpdateByKF(Vector3f_t* angle, Vector3f_t gyro, Vector3f_t acc, float deltaT)
{
    Vector3f_t deltaAngle;
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

    //计算角度变化量，单位为弧度
    deltaAngle.x = Radians(gyro.x * deltaT);
    deltaAngle.y = Radians(gyro.y * deltaT);
    deltaAngle.z = Radians(gyro.z * deltaT);

    //角度变化量转换为方向余弦矩阵
    EulerAngleToDCM(deltaAngle, dcMat);

    //更新卡尔曼状态转移矩阵
    KalmanStateTransMatSet(&kalmanAux, dcMat);

    //卡尔曼滤波器更新
    KalmanUpdate(&kalmanAux, input, acc, fuseFlag);
    ahrsAux.vectorRollPitch = kalmanAux.state;

    //转换成欧拉角
    AccVectorToRollPitchAngle(angle, ahrsAux.vectorRollPitch);
    angle->x = Degrees(angle->x);
    angle->y = Degrees(angle->y);

    //表示横滚和俯仰角误差
    Vector3f_t accAngle;
    AccVectorToRollPitchAngle(&accAngle, acc);
    ahrsAux.angleError.x = ahrsAux.angleError.x * 0.999f + (angle->x - Degrees(accAngle.x)) * 0.001f;
    ahrsAux.angleError.y = ahrsAux.angleError.y * 0.999f + (angle->y - Degrees(accAngle.y)) * 0.001f;
}

/**********************************************************************************************************
*函 数 名: QuaternionUpdateByCF
*功能说明: 使用互补滤波更新四元数
*形    参: 四元数 角速度 加速度计数据 磁力计数据 时间间隔
*返 回 值: 无
**********************************************************************************************************/
void QuaternionUpdateByCF(float q[4], Vector3f_t* angle, Vector3f_t gyro, Vector3f_t acc, Vector3f_t mag, float deltaT)
{
    static Vector3f_t vectorG;
    static Vector3f_t gError, mError;
    static Vector3f_t gyro_bias;
    static Vector3f_t hMag;
    static float gGain_P = 0.1f;
    static float mGain_P = 0.0f;

    //转换角速度单位为弧度
    gyro.x = Radians(gyro.x);
    gyro.y = Radians(gyro.y);
    gyro.z = Radians(gyro.z);

    //加速度计与磁力计数据归一化
    Vector3f_Normalize(&acc);
    Vector3f_Normalize(&mag);

    //提取出重力向量在机体系下的投影
    vectorG.x = -2 * (q[1] * q[3] - q[0] * q[2]);
    vectorG.y = -2 * (q[0] * q[1] + q[2] * q[3]);
    vectorG.z = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

    //向量叉积得到旋转误差矢量
    gError = VectorCrossProduct(acc, vectorG);

    //转换磁力计数据到导航系
    hMag = QuaternionRotateToEarthFrame(q, mag);

    //计算偏航误差
    mError.x = 0;
    mError.y = 0;
    mError.z = atan2f(hMag.y, hMag.x);

    //转换偏航误差至机体系
    //mError = QuaternionRotateToBodyFrame(q, mError);
    EarthFrameToBodyFrame(*angle, mError, &mError);

    if(abs(GetAccMag() - 1) < 0.15f)
    {
        //陀螺仪零偏估计
        gyro_bias.x += gError.x * deltaT * 0.001f;
        gyro_bias.y += gError.y * deltaT * 0.001f;
        gyro_bias.z += gError.z * deltaT * 0.001f;

        //加速度旋转误差矢量修正
        gyro.x -= gError.x * gGain_P;
        gyro.y -= gError.y * gGain_P;
        gyro.z -= gError.z * gGain_P;
    }

    //陀螺仪零偏修正
    gyro.x -= gyro_bias.x;
    gyro.y -= gyro_bias.y;
    gyro.z -= gyro_bias.z;

    //偏航误差修正
    gyro.x -= mError.y * mGain_P;
    gyro.y -= mError.x * mGain_P;
    gyro.z -= mError.z * mGain_P;

    //一阶龙格库塔法更新四元数（一阶矩形积分）
    q[0] += 0.5f * deltaT * (-gyro.x * q[1] - gyro.y * q[2] - gyro.z * q[3]);
    q[1] += 0.5f * deltaT * (+gyro.x * q[0] - gyro.y * q[3] + gyro.z * q[2]);
    q[2] += 0.5f * deltaT * (+gyro.x * q[3] + gyro.y * q[0] - gyro.z * q[1]);
    q[3] += 0.5f * deltaT * (-gyro.x * q[2] + gyro.y * q[1] + gyro.z * q[0]);

    //四元数归一化
    QuaternionNormalize(q);

    //四元数转为姿态欧拉角
    QuaternionToEulerAngle(q, angle);

    //姿态角由弧度转为度
    angle->x = Degrees(angle->x);
    angle->y = Degrees(angle->y);
    angle->z = WrapDegree360(Degrees(angle->z));

    //表示横滚和俯仰角误差
    Vector3f_t accAngle;
    AccVectorToRollPitchAngle(&accAngle, acc);
    ahrsAux.angleError.x = ahrsAux.angleError.x * 0.999f + (angle->x - Degrees(accAngle.x)) * 0.001f;
    ahrsAux.angleError.y = ahrsAux.angleError.y * 0.999f + (angle->y - Degrees(accAngle.y)) * 0.001f;
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

