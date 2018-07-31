#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#include "mathTool.h"

typedef struct{
	Vector3f_t accel;
	Vector3f_t velocity;
    Vector3f_t velMeasure;
	Vector3f_t velocity2;    
	Vector3f_t position;
    Vector3f_t posMeasure;    
    
    Vector3f_t velErrorInt;	
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

float GetDirectionToHome(Vector3f_t position);
float GetDirectionOfTwoPoint(Vector3f_t point1, Vector3f_t point2);

void NavigationReset(void);

#endif







