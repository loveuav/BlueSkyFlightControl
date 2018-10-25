#ifndef __GYROSCOPE_H
#define	__GYROSCOPE_H

#include "sensor.h"

typedef struct {
    Vector3f_t data;
    Vector3f_t dataLpf;
    float temperature;
    LPF2ndData_t lpf_2nd;
    SENSOR_CALI_t cali;
} GYROSCOPE_t;

void GyroPreTreatInit(void);
void GyroDataPreTreat(Vector3f_t gyroRaw, float temperature, Vector3f_t* gyroData, Vector3f_t* gyroLpfData);
void GyroCalibration(Vector3f_t gyroRaw);

uint8_t GetGyroCaliStatus(void);
void GyroCalibrateEnable(void);

Vector3f_t GyroGetData(void);
Vector3f_t GyroLpfGetData(void);
float GyroGetTemp(void);
Vector3f_t GetGyroOffsetCaliData(void);

#endif



