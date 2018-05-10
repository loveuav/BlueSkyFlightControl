#ifndef _KALMAN3_H_
#define _KALMAN3_H_

#include "mathTool.h"

typedef struct{
	//状态矩阵
	Vector3f_t status;
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



#endif
