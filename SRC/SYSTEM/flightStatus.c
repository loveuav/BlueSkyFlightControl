/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     flightStatus.c
 * @说明     飞行状态分类与检测
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "flightStatus.h"

typedef struct{
    uint8_t failsafe;
	uint8_t armed;
	uint8_t flight;
	uint8_t altControl;
	uint8_t posControl;
	uint8_t mode;
}FLIGHT_STATUS_t;

FLIGHT_STATUS_t flyStatus;


void SetArmedStatus(uint8_t status)
{
	if(status == DISARMED)	//上锁
	{	
		
	}
	else if(status == ARMED)	//解锁
	{	
		
	}
}












