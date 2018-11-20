#ifndef _KALMANVEL_H_
#define _KALMANVEL_H_

#include "mathTool.h"

typedef struct {
    //状态矩阵
    //状态量为：x轴速度 y轴速度 z轴速度 x轴加速度bias y轴加速度bias z轴加速度bias
    float state[6];
    //滑动窗口大小
    int16_t slidWindowSize;
    //状态滑动窗口
    Vector3f_t* stateSlidWindow;
    //观测信号相位补偿值
    int16_t fuseDelay[6];
    //残差矩阵
    float residual[6];
    //状态转移矩阵
    float f[6][6];
    //状态转移矩阵的转置
    float f_t[6][6];
    //观测映射矩阵
    float h[6][6];
    //观测映射矩阵的转置
    float h_t[6][6];
    //控制输入矩阵
    float b[6][6];
    //卡尔曼增益矩阵
    float gain[6][6];
    //误差协方差矩阵
    float covariance[6][6];
    //过程噪声协方差矩阵
    float q[6][6];
    //测量噪声协方差矩阵
    float r[6][6];
} KalmanVel_t;

void KalmanVelUpdate(KalmanVel_t* kalman, Vector3f_t* velocity, Vector3f_t* bias, Vector3f_t accel, 
                     float observe[6], float deltaT, bool fuseFlag);

void KalmanVelStateTransMatSet(KalmanVel_t* kalman, float f[6][6]);
void KalmanVelObserveMapMatSet(KalmanVel_t* kalman, float h[6][6]);
void KalmanVelCovarianceMatSet(KalmanVel_t* kalman, float p[6][6]);
void KalmanVelQMatSet(KalmanVel_t* kalman, float q[6][6]);
void KalmanVelRMatSet(KalmanVel_t* kalman, float r[6][6]);
void KalmanVelBMatSet(KalmanVel_t* kalman, float b[6][6]);

void KalmanVelUseMeasurement(KalmanVel_t* kalman, uint8_t num, bool flag);

#endif







