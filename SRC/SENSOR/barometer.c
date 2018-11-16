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
#include "faultDetect.h"
#include "flightStatus.h"
#include "navigation.h"
#include "gps.h"

typedef struct {
    int32_t alt;
    int32_t lastAlt;
    float   velocity;
    int32_t alt_offset;
    float temperature;
} BAROMETER_t;

BAROMETER_t baro;

static void BaroCompensate(int32_t* alt);
static void BaroDetectCheck(int32_t baroAlt);

/**********************************************************************************************************
*函 数 名: BaroDataPreTreat
*功能说明: 气压高度数据预处理
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void BaroDataPreTreat(void)
{
    static uint64_t lastTime = 0;
    int32_t baroAltTemp;

    float deltaT = (GetSysTimeUs() - lastTime) * 1e-6;
    lastTime = GetSysTimeUs();

    //读取气压高度
    BaroSensorRead(&baroAltTemp);
    //读取气压温度
    BaroTemperatureRead(&baro.temperature);

    //计算气压高度的初始零偏值
    if(GetInitStatus() == HEAT_FINISH)
    {
        baro.alt_offset += baroAltTemp;
        baro.alt_offset *= 0.5f;
    }
    baroAltTemp -= baro.alt_offset;

    //飞行中的气压高度补偿
    BaroCompensate(&baroAltTemp);

    //气压高度低通滤波
    baro.alt = baro.alt * 0.5f + baroAltTemp * 0.5f;

    //计算气压变化速度，并进行低通滤波
    baro.velocity = baro.velocity * 0.65f + ((baro.alt - baro.lastAlt) / deltaT) * 0.35f;
    baro.lastAlt = baro.alt;

    //检测气压传感器是否工作正常
    BaroDetectCheck(baro.alt);
}

/**********************************************************************************************************
*函 数 名: BaroCompensate
*功能说明: 气压高度补偿（简单处理，用于测试）
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void BaroCompensate(int32_t* alt)
{
    int16_t velocity;

    if(!GpsGetFixStatus())
        return;

    velocity = Pythagorous2(GetCopterVelocity().x, GetCopterVelocity().y);
    velocity = ApplyDeadbandInt(ConstrainInt16(velocity, 0, 500), 20);

    *alt -= velocity * 0.05f;
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
*函 数 名: BaroGetTemp
*功能说明: 获取气压温度数据
*形    参: 无
*返 回 值: 气压温度
**********************************************************************************************************/
float BaroGetTemp(void)
{
    return baro.temperature;
}

/**********************************************************************************************************
*函 数 名: BaroGetVelocity
*功能说明: 获取气压高度变化速度
*形    参: 无
*返 回 值: 气压高度变化速度
**********************************************************************************************************/
float BaroGetVelocity(void)
{
    return baro.velocity;
}


/**********************************************************************************************************
*函 数 名: BaroDetectCheck
*功能说明: 检测气压传感器工作是否正常，通过检测气压高度数据变化来判断
*形    参: 气压高度值
*返 回 值: 无
**********************************************************************************************************/
static void BaroDetectCheck(int32_t baroAlt)
{
    static uint32_t cnt;
    static int32_t lastBaroAlt = 0;

    if(baroAlt == lastBaroAlt)
    {
        cnt++;

        if(cnt > 20)
        {
            //未检测到气压传感器
            FaultDetectSetError(BARO_UNDETECTED);
        }
    }
    else
    {
        cnt = 0;
        FaultDetectResetError(BARO_UNDETECTED);
    }

    lastBaroAlt = baroAlt;
}


