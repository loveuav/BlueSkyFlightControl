#ifndef __ACCELEROMETER_H
#define	__ACCELEROMETER_H

#include "sensor.h"

typedef struct{
	Vector3f_t data;
	float mag;
	float vibraCoef;
	SENSOR_CALI_t cali;
	SENSOR_CALI_t levelCali;
}ACCELEROMETER_t;

void AccPreTreatInit(void);
void AccDataPreTreat(Vector3f_t accRaw, Vector3f_t* accData);
void AccCalibration(Vector3f_t accRaw);
void ImuLevelCalibration(void);
Vector3f_t GetLevelCalibraData(void);
float GetAccMag(void);
Vector3f_t AccGetData(void);

#endif



