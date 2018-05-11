#ifndef _GAUSSNEWTON_H
#define _GAUSSNEWTON_H

#include "mathTool.h"

void GaussNewtonCalibrate(Vector3f_t inputData[6],Vector3f_t* offset, Vector3f_t* scale, 
     float length, int16_t maxIteration);

#endif
