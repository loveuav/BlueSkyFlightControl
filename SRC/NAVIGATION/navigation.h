#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#include "mathTool.h"

enum
{
    GPS_VEL_X = 0,
    GPS_VEL_Y,
    GPS_VEL_Z,
    BARO_VEL,
    TOF_VEL
};

typedef struct {
    Vector3f_t accel;
    Vector3f_t accel_bias;
    
    Vector3f_t velocity;
    float      velMeasure[6];
    
    Vector3f_t position;
    Vector3f_t posMeasure;
} NAVGATION_t;

void NavigationInit(void);
void VelocityEstimate(void);
void PositionEstimate(void);

void AltCovarianceSelfAdaptation(void);
void PosCovarianceSelfAdaptation(void);

Vector3f_t GetCopterAccel(void);
Vector3f_t GetAccelBias(void);
Vector3f_t GetCopterVelocity(void);
float* GetCopterVelMeasure(void);
Vector3f_t GetCopterPosition(void);
Vector3f_t GetCopterPosMeasure(void);

void NavigationReset(void);

#endif







