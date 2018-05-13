#ifndef __GYROSCOPE_H
#define	__GYROSCOPE_H

#include "sensor.h"

typedef struct{
	Vector3f_t data;
	Vector3f_t dataLpf;
    LPF2ndData_t lpf_2nd;
	SENSOR_CALI_t cali;
}GYROSCOPE_t;

void GyroPreTreatInit(void);
void GyroDataPreTreat(Vector3f_t gyroRaw, Vector3f_t* gyroData, Vector3f_t* gyroLpfData);
void GyroCalibration(Vector3f_t gyroRaw);
void GyroCalibrateEnable(void);
Vector3f_t GyroGetData(void);

#endif



