#ifndef __MODULE_H__
#define __MODULE_H__

#include "mathTool.h"

bool GyroSensorInit(void);
bool MagSensorInit(void);
bool BaroSensorInit(void);

void GyroSensorRead(Vector3i_t* gyro);
void AccSensorRead(Vector3i_t* acc);
void TempSensorRead(float* temp);
void MagSensorRead(Vector3i_t* mag);
void BaroSensorRead(int32_t baroAlt);

Vector3f_t GyroDataNormalize(Vector3i_t raw);
Vector3f_t AccDataNormalize(Vector3i_t raw);

#endif










