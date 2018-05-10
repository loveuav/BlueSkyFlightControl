#ifndef __MPU6000_H
#define	__MPU6000_H

#include "mathTool.h"

bool MPU6000_Detect(void);
void MPU6000_Init(void);

void MPU6000_ReadAcc(Vector3i_t* acc);
void MPU6000_ReadGyro(Vector3i_t* gyro);
void MPU6000_ReadTemp(float* temp);
    
Vector3f_t MPU6000_GyroNormalize(Vector3i_t raw);
Vector3f_t MPU6000_AccNormalize(Vector3i_t raw);

#endif








