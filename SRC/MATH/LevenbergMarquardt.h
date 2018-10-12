#ifndef _LEVENBERGMARQUARDT_H
#define _LEVENBERGMARQUARDT_H

#include "mathTool.h"

void LevenbergMarquardt(Vector3f_t inputData[6], Vector3f_t* offset, Vector3f_t* scale, float initBeta[6], float length);

#endif
