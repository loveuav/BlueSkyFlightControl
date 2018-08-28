#ifndef __BOARD_H__
#define __BOARD_H__

#include "stm32f4xx.h"

/**********************************************************************************************************
*飞控软件版本
**********************************************************************************************************/
#define SOFTWARE_VERSION_HIGH   0
#define SOFTWARE_VERSION_MID    4
#define SOFTWARE_VERSION_LOW    17

/**********************************************************************************************************
*飞控硬件类型
**********************************************************************************************************/
enum
{
    BOARD_BLUESKY_V3,
	BOARD_FUTURESKY
};

/**********************************************************************************************************
*定义硬件类型
**********************************************************************************************************/
#define BLUESKY_V3

//硬件配置头文件包含
#ifdef BLUESKY_V3
	#include "boardConfigBlueSkyV3.h"
    #define BOARD_TYPE BOARD_BLUESKY_V3
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
    ICM20602,
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






