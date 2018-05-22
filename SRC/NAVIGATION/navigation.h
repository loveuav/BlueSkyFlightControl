#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#include "mathTool.h"

typedef struct{
	Vector3f_t accel;
	Vector3f_t velocity;	
	Vector3f_t position;		
	Vector3f_t gpsVel;	
	
}NAVGATION_t;

void NavigationInit(void);
void VelocityEstimate(void);
void PositionEstimate(void);

Vector3f_t GetCopterVelocity(void);
Vector3f_t GetCopterPosition(void);

#endif







