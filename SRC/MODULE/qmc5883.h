#ifndef __QMC5883_H
#define	__QMC5883_H

#include "mathTool.h"

bool QMC5883_Detect(void);
void QMC5883_Init(void);
void QMC5883_Update(void);
void QMC5883_Read(Vector3i_t* mag);
Vector3f_t QMC5883_MagNormalize(Vector3i_t raw);

#endif


