#ifndef _LOGGER_H_
#define _LOGGER_H_

#include "mathTool.h"

#define LOG_RATE 100 //日志记录频率 单位:Hz

void LoggerInit(void);
void LoggerLoop(void);
void LoggerWrite(void *data, uint16_t size);

#endif


