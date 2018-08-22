#ifndef _KALMAN3_H_
#define _KALMAN3_H_

#include "mathTool.h"

typedef struct{
	//状态矩阵
	Vector3f_t state;
    //滑动窗口大小
    int16_t slidWindowSize;
	//状态滑动窗口
	Vector3f_t* statusSlidWindow;  
    //观测信号相位补偿值
    Vector3i_t fuseDelay;
	//残差矩阵
	Vector3f_t residual;
	//状态转移矩阵
	float f[9];
	//状态转移矩阵的转置
	float f_t[9];
	//观测映射矩阵
	float h[9];
	//观测映射矩阵的转置
	float h_t[9];
	//控制输入矩阵
	float b[9];
	//卡尔曼增益矩阵
	float gain[9];
	//误差协方差矩阵
	float covariance[9];
	//过程噪声协方差矩阵
	float q[9];
	//测量噪声协方差矩阵
	float r[9];	
}Kalman_t;

void KalmanUpdate(Kalman_t* kalman, Vector3f_t input, Vector3f_t observe, bool flag);

void KalmanStateTransMatSet(Kalman_t* kalman, float* f);
void KalmanObserveMapMatSet(Kalman_t* kalman, float* h);
void KalmanCovarianceMatSet(Kalman_t* kalman, float* p);
void KalmanQMatSet(Kalman_t* kalman, float* q);
void KalmanRMatSet(Kalman_t* kalman, float* r);
void KalmanBMatSet(Kalman_t* kalman, float* b);
    
#endif
