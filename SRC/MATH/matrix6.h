#ifndef _MATRIX6_H_
#define _MATRIX6_H_

#include "mathTool.h"

void Matrix6_Add(float a[6][6], float b[6][6], float c[6][6]);
void Matrix6_Sub(float a[6][6], float b[6][6], float c[6][6]);
void Matrix6_Mul(float a[6][6], float b[6][6], float c[6][6]);
void Matrix6_Copy(float a[6][6], float b[6][6]);
void Matrix6_Tran(float a[6][6], float b[6][6]);
bool Matrix6_Det(float a[6][6], float b[6][6]);

void Vector6f_Add(float v1[6], float v2[6], float v3[6]);
void Vector6f_Sub(float v1[6], float v2[6], float v3[6]);

void Matrix6MulVector6(float m[6][6], float v[6], float result[6]);

#endif

