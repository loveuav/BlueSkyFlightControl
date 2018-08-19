#ifndef _AHRSAUX_H_
#define _AHRSAUX_H_

#include "mathTool.h"

typedef struct{
	Vector3f_t angle;	
    float      q[4];
    Vector3f_t angleError;	
    
	Vector3f_t vectorRollPitch;

    Vector3f_t accEf;
}AHRSAUX_t;

void AHRSAuxInit(void);
void AttitudeAuxEstimate(Vector3f_t gyro, Vector3f_t acc, Vector3f_t mag);

void RollPitchUpdateByKF(Vector3f_t* angle, Vector3f_t gyro, Vector3f_t acc, float deltaT);
void QuaternionUpdateByCF(float q[4], Vector3f_t* angle, Vector3f_t gyro, Vector3f_t acc, Vector3f_t mag, float deltaT);

Vector3f_t GetSportAccEf(void);
Vector3f_t GetAuxAngle(void);

#endif


