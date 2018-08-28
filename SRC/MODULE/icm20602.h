#ifndef __ICM20602_H
#define	__ICM20602_H

#include "mathTool.h"

bool ICM20602_Detect(void);
void ICM20602_Init(void);

void ICM20602_ReadAcc(Vector3f_t* acc);
void ICM20602_ReadGyro(Vector3f_t* gyro);
void ICM20602_ReadTemp(float* temp);

#endif








