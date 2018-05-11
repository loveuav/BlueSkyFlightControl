#ifndef __GYROSCOPE_H
#define	__GYROSCOPE_H

#include "sensor.h"

typedef struct{
	Vector3f_t data;
	SENSOR_CALI_t cali;
}GYROSCOPE_t;

void GyroCaliDataInit(void);
void GyroDataPreTreat(Vector3f_t gyroRaw, Vector3f_t* gyroData);
void GyroCalibration(Vector3f_t gyroRaw);
void GyroCalibrateEnable(void);

#endif



