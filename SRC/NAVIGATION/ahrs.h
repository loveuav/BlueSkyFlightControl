#ifndef _AHRS_H_
#define _AHRS_H_

#include "mathTool.h"

typedef struct{
	Vector3f_t angle;	
	Vector3f_t vectorRollPitch;
	Vector3f_t vectorRollPitchError;
	Vector3f_t vectorRollPitchErrorInt;
    float      vectorRollPitchKI;
    
	Vector3f_t vectorYaw; 	
	Vector3f_t vectorYawError; 	
	Vector3f_t vectorYawErrorInt;
    float      vectorYawKI;	
    
    Vector3f_t accEf;
}AHRS_t;

void AHRSInit(void);
void AttitudeEstimate(Vector3f_t gyro, Vector3f_t acc, Vector3f_t mag);
Vector3f_t GetCopterAngle(void);
Vector3f_t GetCopterAccEf(void);

#endif






