/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     quaternion.c
 * @说明     四元数相关运算，不同坐标系和旋转方向定义，公式的具体形式有所区别
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.08
**********************************************************************************************************/
#include "quaternion.h"

/**********************************************************************************************************
*函 数 名: EulerAngleToQuaternion
*功能说明: 欧拉角转四元数
*形    参: 欧拉角 四元数
*返 回 值: 无
**********************************************************************************************************/
void EulerAngleToQuaternion(Vector3f_t angle, float q[4])
{
    /*方法1*/
    float sinx = sinf(angle.x / 2);
    float cosx = cosf(angle.x / 2);
    float siny = sinf(angle.y / 2);
    float cosy = cosf(angle.y / 2);
    float sinz = sinf(angle.z / 2);
    float cosz = cosf(angle.z / 2);

    q[0] = cosx * cosy * cosz - sinx * siny * sinz;
    q[1] = -sinx * cosy * cosz - cosx * siny * sinz;
    q[2] = -cosx * siny * cosz + sinx * cosy * sinz;
    q[3] = cosx * cosy * sinz + sinx * siny * cosz;

    /*方法2*/
//    float dcM[9];
//    EulerAngleToDCM(angle, dcM);
//
//    q[0] = 0.5f * sqrtf(1 + dcM[0] + dcM[4] + dcM[8]);
//    q[1] = (dcM[7] - dcM[5]) / (4 * q[0]);
//    q[2] = (dcM[2] - dcM[6]) / (4 * q[0]);
//    q[3] = (dcM[3] - dcM[1]) / (4 * q[0]);
}

/**********************************************************************************************************
*函 数 名: QuaternionToDCM
*功能说明: 四元数转方向余弦矩阵(表示从参考系到机体系)
*形    参: 四元数 方向余弦矩阵
*返 回 值: 无
**********************************************************************************************************/
void QuaternionToDCM(float q[4], float dcM[9])
{
    float q0q0 = q[0] * q [0];
    float q0q1 = q[0] * q [1];
    float q0q2 = q[0] * q [2];
    float q0q3 = q[0] * q [3];
    float q1q1 = q[1] * q [1];
    float q1q2 = q[1] * q [2];
    float q1q3 = q[1] * q [3];
    float q2q2 = q[2] * q [2];
    float q2q3 = q[2] * q [3];
    float q3q3 = q[3] * q [3];

    dcM[0] = q0q0 + q1q1 - q2q2 - q3q3;
    dcM[1] = 2 * (q1q2 + q0q3);
    dcM[2] = 2 * (q0q2 + q1q3);
    dcM[3] = 2 * (q1q2 - q0q3);
    dcM[4] = q0q0 - q1q1 + q2q2 - q3q3;
    dcM[5] = 2 * (q2q3 - q0q1);
    dcM[6] = 2 * (q1q3 - q0q2);
    dcM[7] = 2 * (q0q1 + q2q3);
    dcM[8] = q0q0 - q1q1 - q2q2 + q3q3;
}

/**********************************************************************************************************
*函 数 名: QuaternionToDCM_T
*功能说明: 四元数转方向余弦矩阵(表示从机体系到参考系)
*形    参: 四元数 方向余弦矩阵
*返 回 值: 无
**********************************************************************************************************/
void QuaternionToDCM_T(float q[4], float dcM[9])
{
    float q0q0 = q[0] * q [0];
    float q0q1 = q[0] * q [1];
    float q0q2 = q[0] * q [2];
    float q0q3 = q[0] * q [3];
    float q1q1 = q[1] * q [1];
    float q1q2 = q[1] * q [2];
    float q1q3 = q[1] * q [3];
    float q2q2 = q[2] * q [2];
    float q2q3 = q[2] * q [3];
    float q3q3 = q[3] * q [3];

    dcM[0] = q0q0 + q1q1 - q2q2 - q3q3;
    dcM[1] = 2 * (q1q2 - q0q3);
    dcM[2] = 2 * (q1q3 - q0q2);
    dcM[3] = 2 * (q1q2 + q0q3);
    dcM[4] = q0q0 - q1q1 + q2q2 - q3q3;
    dcM[5] = 2 * (q0q1 + q2q3);
    dcM[6] = 2 * (q0q2 + q1q3);
    dcM[7] = 2 * (q2q3 - q0q1);
    dcM[8] = q0q0 - q1q1 - q2q2 + q3q3;
}

/**********************************************************************************************************
*函 数 名: QuaternionRotateToEarthFrame
*功能说明: 使用四元数旋转一个向量（机体系到导航系）
*形    参: 四元数 向量
*返 回 值: 旋转后的向量
**********************************************************************************************************/
Vector3f_t QuaternionRotateToEarthFrame(float q[4], Vector3f_t vector)
{
    static float dcM[9];
    QuaternionToDCM_T(q, dcM);
    return Matrix3MulVector3(dcM, vector);
}

/**********************************************************************************************************
*函 数 名: QuaternionRotateToBodyFrame
*功能说明: 使用四元数旋转一个向量（导航系到机体系）
*形    参: 四元数 向量
*返 回 值: 旋转后的向量
**********************************************************************************************************/
Vector3f_t QuaternionRotateToBodyFrame(float q[4], Vector3f_t vector)
{
    static float dcM[9];
    QuaternionToDCM(q, dcM);
    return Matrix3MulVector3(dcM, vector);
}

/**********************************************************************************************************
*函 数 名: QuaternionToEulerAngle
*功能说明: 四元数转欧拉角
*形    参: 四元数 欧拉角
*返 回 值: 无
**********************************************************************************************************/
void QuaternionToEulerAngle(float q[4], Vector3f_t* angle)
{
    static float dcM[9];

    QuaternionToDCM(q, dcM);

    angle->x = -asinf(-dcM[7]);
    angle->y = atan2f(-dcM[6], dcM[8]);
    angle->z = -atan2f(dcM[3], dcM[4]);
}

/**********************************************************************************************************
*函 数 名: QuaternionNormalize
*功能说明: 四元数归一化
*形    参: 四元数
*返 回 值: 无
**********************************************************************************************************/
void QuaternionNormalize(float q[4])
{
    float qMag = Pythagorous4(q[0], q[1], q[2], q[3]);

    q[0] /= qMag;
    q[1] /= qMag;
    q[2] /= qMag;
    q[3] /= qMag;
}

































