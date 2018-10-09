#ifndef __MMC3630_H
#define	__MMC3630_H

#include "mathTool.h"

bool MMC3630_Detect(void);
void MMC3630_Init(void);
void MMC3630_Update(void);
void MMC3630_Read(Vector3f_t* mag);

#endif


