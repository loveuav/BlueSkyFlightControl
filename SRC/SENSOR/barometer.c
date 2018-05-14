/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     barometer.c
 * @说明     气压计数据预处理
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "barometer.h"
#include "module.h"
#include "board.h"

typedef struct{
	int32_t alt;
	int32_t lastAlt;
	int32_t velocity;
	int32_t alt_offset;
}BAROMETER_t;

BAROMETER_t baro;

/**********************************************************************************************************
*函 数 名: BaroDataPreTreat
*功能说明: 气压高度数据预处理
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void BaroDataPreTreat(void)
{
	static uint32_t lastTime = 0;
	static uint8_t offset_cnt = 50;

	float deltaT = (GetSysTimeUs() - lastTime) * 1e-6;
	lastTime = GetSysTimeUs();
	
    BaroSensorRead(&baro.alt);
    
    if(GetSysTimeMs() > 1500)
    {
        //计算气压高度的初始零偏值
        if(offset_cnt)
        {
            offset_cnt--;
            baro.alt_offset += baro.alt;
            baro.alt_offset *= 0.5f;
        }
    }
    
	//读取气压高度
	baro.alt -= baro.alt_offset;
	//计算气压变化速度
	baro.velocity = (baro.alt - baro.lastAlt) / deltaT;
	baro.lastAlt = baro.alt;	
}

/**********************************************************************************************************
*函 数 名: BaroGetAlt
*功能说明: 获取气压高度数据
*形    参: 无 
*返 回 值: 气压高度
**********************************************************************************************************/
int32_t BaroGetAlt(void)
{
   return baro.alt;
}

/**********************************************************************************************************
*函 数 名: BaroGetVelocity
*功能说明: 获取气压高度变化速度
*形    参: 无 
*返 回 值: 气压高度变化速度
**********************************************************************************************************/
int32_t BaroGetVelocity(void)
{
    return baro.velocity;
}


