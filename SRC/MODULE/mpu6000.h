#ifndef __MPU6000_H
#define	__MPU6000_H

#include "mathTool.h"

bool MPU6000_Detect(void);
void MPU6000_Init(void);

void MPU6000_ReadAcc(Vector3f_t* acc);
void MPU6000_ReadGyro(Vector3f_t* gyro);
void MPU6000_ReadTemp(float* temp);

#endif








