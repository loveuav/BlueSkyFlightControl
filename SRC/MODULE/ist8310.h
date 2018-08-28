#ifndef __IST8310_H
#define	__IST8310_H

#include "mathTool.h"

bool IST8310_Detect(void);
void IST8310_Init(void);
void IST8310_Update(void);
void IST8310_Read(Vector3f_t* mag);


#endif


