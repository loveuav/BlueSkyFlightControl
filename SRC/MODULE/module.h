#ifndef __MODULE_H__
#define __MODULE_H__

#include "mathTool.h"

bool GyroSensorInit(void);
bool MagSensorInit(void);
bool BaroSensorInit(void);
void GPSModuleInit(void);

void GyroSensorRead(Vector3f_t* gyro);
void AccSensorRead(Vector3f_t* acc);
void TempSensorRead(float* temp);

void MagSensorUpdate(void);
void MagSensorRead(Vector3f_t* mag);
void BaroSensorRead(int32_t* baroAlt);
void BaroSensorUpdate(void);

void TempControlSet(int16_t value);

#endif










