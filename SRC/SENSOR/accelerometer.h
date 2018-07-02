#ifndef __ACCELEROMETER_H
#define	__ACCELEROMETER_H

#include "sensor.h"

typedef struct{
	Vector3f_t data;
	Vector3f_t dataLpf;
	float mag;
	float vibraCoef;
    LPF2ndData_t lpf_2nd;
	SENSOR_CALI_t cali;
	SENSOR_CALI_t levelCali;
}ACCELEROMETER_t;

void AccPreTreatInit(void);
void AccDataPreTreat(Vector3f_t accRaw, Vector3f_t* accData);
void AccCalibration(Vector3f_t accRaw);
void AccScaleCalibrate(Vector3f_t* acc);
void ImuLevelCalibration(void);

Vector3f_t GetAccOffsetCaliData(void);
Vector3f_t GetAccScaleCaliData(void);
Vector3f_t GetLevelCalibraData(void);

void AccCalibrateEnable(void);
void LevelCalibrateEnable(void);
    
float GetAccMag(void);
Vector3f_t AccGetData(void);
Vector3f_t AccLpfGetData(void);

#endif



