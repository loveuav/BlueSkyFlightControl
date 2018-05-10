#ifndef _MATRIX3_H_
#define _MATRIX3_H_

#include "mathTool.h"

void Matrix3_Add(float* a,float* b,float* c);
void Matrix3_Sub(float* a,float* b,float* c);
void Matrix3_Mul(float* a,float* b,float* c);
void Matrix3_Copy(float* a, float* b);
void Matrix3_Tran(float* a, float* b);
void Matrix3_Det(float* a,float* b);

#endif
