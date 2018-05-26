#ifndef __MPU6500_H
#define	__MPU6500_H

#include "mathTool.h"

bool MPU6500_Detect(void);
void MPU6500_Init(void);

void MPU6500_ReadAcc(Vector3f_t* acc);
void MPU6500_ReadGyro(Vector3f_t* gyro);
void MPU6500_ReadTemp(float* temp);

#endif








