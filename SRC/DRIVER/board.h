#ifndef __BOARD_H__
#define __BOARD_H__

#include "stm32f4xx.h"

/**********************************************************************************************************
*飞控软件版本
**********************************************************************************************************/
#define SOFTWARE_VERSION_HIGH   0
#define SOFTWARE_VERSION_LOW    4
#define SOFTWARE_VERSION_MID    9

/**********************************************************************************************************
*飞控硬件类型
**********************************************************************************************************/
enum
{
    BOARD_TEST,
	BOARD_BLUESKY_V2,
    BOARD_BLUESKY_V3,
	BOARD_FUTURESKY
};

/**********************************************************************************************************
*定义硬件类型
**********************************************************************************************************/
//#define BLUESKY_V2
#define BLUESKY_V3
//#define TESTBOARD

#ifdef BLUESKY_V2
	#include "boardConfigBlueSkyV2.h"
    #define BOARD_TYPE BOARD_BLUESKY_V2
#endif
#ifdef BLUESKY_V3
	#include "boardConfigBlueSkyV3.h"
    #define BOARD_TYPE BOARD_BLUESKY_V3
#endif
#ifdef TESTBOARD
	#include "boardConfigTest.h"
    #define BOARD_TYPE BOARD_TEST
#endif

typedef struct
{
    int16_t roll;   
    int16_t pitch;   
    int16_t yaw;      
    int16_t throttle;  
    int16_t aux1;  
    int16_t aux2;  
    int16_t aux3;  
    int16_t aux4;  
    int16_t aux5;  
    int16_t aux6;   
    int16_t aux7;  
    int16_t aux8;  
    int16_t aux9;  
    int16_t aux10;     
    int16_t aux11;  
    int16_t aux12;  
}RCDATA_t;

enum
{
    MPU6000,
    MPU6500,
    ICM20689
};

enum
{
    BMP280,
    MS5611,
    _2SMPB,
    LPS22HB
};

enum
{
    HMC5883,
    QMC5883,
    IST8310
};

enum
{
    SBUS,
    PPM
};

void BoardInit(void);
void SoftDelayMs(uint32_t ms);
void SoftDelayUs(uint32_t us);
void OsDelayMs(uint32_t ms);

uint64_t GetSysTimeUs(void);
uint32_t GetSysTimeMs(void);

#endif






