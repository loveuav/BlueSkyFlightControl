#ifndef __QUATERNION_H__
#define __QUATERNION_H__

#include "mathTool.h"

void EulerAngleToQuaternion(Vector3f_t angle, float q[4]);
void QuaternionToDCM(float q[4], float dcM[9]);
void QuaternionToDCM_T(float q[4], float dcM[9]);
Vector3f_t QuaternionRotateToEarthFrame(float q[4], Vector3f_t vector);
Vector3f_t QuaternionRotateToBodyFrame(float q[4], Vector3f_t vector);
void QuaternionToEulerAngle(float q[4], Vector3f_t* angle);
void QuaternionNormalize(float q[4]);

#endif











