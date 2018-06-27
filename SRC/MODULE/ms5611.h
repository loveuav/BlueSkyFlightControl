#ifndef __MS5611_H
#define	__MS5611_H

#include "mathTool.h"

bool MS5611_Detect(void);
void MS5611_Init(void);

void MS5611_Update(void);
void MS5611_Read(int32_t* baroAlt);
void MS5611_ReadTemp(float* temp);

#endif
