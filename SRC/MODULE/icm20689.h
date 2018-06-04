#ifndef __ICM20689_H
#define	__ICM20689_H

#include "mathTool.h"

bool ICM20689_Detect(void);
void ICM20689_Init(void);

void ICM20689_ReadAcc(Vector3f_t* acc);
void ICM20689_ReadGyro(Vector3f_t* gyro);
void ICM20689_ReadTemp(float* temp);

#endif








