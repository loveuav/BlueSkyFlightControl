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

#endif







