#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#include "mathTool.h"

typedef struct{
	Vector3f_t accel;
	Vector3f_t velocity;	
	Vector3f_t velocity2;    
	Vector3f_t position;		
	Vector3f_t gpsVel;	
    
    Vector3f_t velErrorInt;	
}NAVGATION_t;

void NavigationInit(void);
void VelocityEstimate(void);
void PositionEstimate(void);

Vector3f_t GetCopterAccel(void);
Vector3f_t GetCopterVelocity(void);
Vector3f_t GetCopterPosition(void);

void NavigationReset(void);

extern NAVGATION_t nav;

#endif







