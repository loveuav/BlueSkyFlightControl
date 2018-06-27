#ifndef __2SMPB_H
#define	__2SMPB_H

#include "mathTool.h"

bool _2SMPB_Detect(void);
void _2SMPB_Init(void);

void _2SMPB_Update(void);
void _2SMPB_Read(int32_t* baroAlt);
void _2SMPB_ReadTemp(float* temp);

#endif
