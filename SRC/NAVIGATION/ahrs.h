#ifndef _AHRS_H_
#define _AHRS_H_

#include "mathTool.h"

typedef struct {
    Vector3f_t angle;
    Vector3f_t angleError;
    Vector3f_t angleMeasure;
    
    Vector3f_t accEf;
    Vector3f_t accEfLpf;
    Vector3f_t accBfOffset;

    Vector3f_t gyroEf;

    Vector3f_t centripetalAcc;
    Vector3f_t centripetalAccBf;
} AHRS_t;

void AHRSInit(void);
void AttitudeEstimate(Vector3f_t gyro, Vector3f_t acc, Vector3f_t mag);
void BodyFrameToEarthFrame(Vector3f_t angle, Vector3f_t vector, Vector3f_t* vectorEf);
void EarthFrameToBodyFrame(Vector3f_t angle, Vector3f_t vector, Vector3f_t* vectorBf);

void AttCovarianceSelfAdaptation(void);

Vector3f_t GetCopterAngle(void);
Vector3f_t GetCopterAccEf(void);
Vector3f_t GetCopterAccEfLpf(void);
Vector3f_t GetCentripetalAcc(void);
Vector3f_t GetCentripetalAccBf(void);
Vector3f_t GetAngleMeasure(void);
Vector3f_t GetAngleEstError(void);

uint8_t RollOverDetect(void);

#endif






