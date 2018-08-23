#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#include "mathTool.h"

typedef struct{
	Vector3f_t accel;
	Vector3f_t velocity;
    Vector3f_t velMeasure;
	Vector3f_t position;
    Vector3f_t posMeasure;    
}NAVGATION_t;

void NavigationInit(void);
void VelocityEstimate(void);
void PositionEstimate(void);

void AltCovarianceSelfAdaptation(void);
void PosCovarianceSelfAdaptation(void);

Vector3f_t GetCopterAccel(void);
Vector3f_t GetCopterVelocity(void);
Vector3f_t GetCopterVelMeasure(void);
Vector3f_t GetCopterPosition(void);
Vector3f_t GetCopterPosMeasure(void);

void NavigationReset(void);

#endif







