#ifndef _LOWPASSFILTER_H_
#define _LOWPASSFILTER_H_

#include "mathTool.h"

typedef struct
{
    float b0;
    float a1;
    float a2;
    Vector3f_t preout;
    Vector3f_t lastout;
} LPF2ndData_t;

void LowPassFilter1st(Vector3f_t* data, Vector3f_t newData, float coff);
void LowPassFilter2ndFactorCal(float deltaT, float Fcut, LPF2ndData_t* lpf_data);
Vector3f_t LowPassFilter2nd(LPF2ndData_t* lpf_2nd, Vector3f_t rawData);

#endif


