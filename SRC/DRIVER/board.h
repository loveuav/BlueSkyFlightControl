#ifndef __BOARD_H__
#define __BOARD_H__

#include "stm32f4xx.h"

/**********************************************************************************************************
*飞控硬件类型
**********************************************************************************************************/
enum{
	BLUESKY_V2,
	BLUESKY_V3,
	MINISKY,
};

/**********************************************************************************************************
*定义硬件类型
**********************************************************************************************************/
#define BOARD_TYPE	BLUESKY_V2
//#define BOARD_TYPE	BLUESKY_V3

#if (BOARD_TYPE == BLUESKY_V2)
	#include "boardConfigBlueSkyV2.h"
#endif
#if (BOARD_TYPE == BLUESKY_V3)
	#include "boardConfigBlueSkyV3.h"
#endif

void BoardInit(void);
void SoftDelayMs(uint32_t ms);
void SoftDelayUs(uint32_t us);

uint32_t GetSysTimeUs(void);
uint32_t GetSysTimeMs(void);

#endif






