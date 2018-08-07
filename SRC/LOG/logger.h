#ifndef _LOGGER_H_
#define _LOGGER_H_

#include "mathTool.h"

void LoggerInit(void);
void LoggerLoop(void);
void LoggerWrite(void *data, uint16_t size);

#endif


