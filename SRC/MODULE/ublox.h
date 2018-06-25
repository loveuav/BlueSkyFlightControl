#ifndef __UBLOX_H
#define	__UBLOX_H

#include "mathTool.h"

typedef struct{
	float   time;		  //时间
	double	latitude;	  //纬度
	double	longitude;    //经度
	float	altitude;	  //高度	
	int32_t	velN;		  //北向速度
	int32_t	velE;		  //东向速度
	int32_t	velD;		  //天向速度
	float	hAcc;		  //水平定位精度
    float   vAcc;         //垂直定位精度
	uint8_t fixStatus;    //定位状态		
	uint8_t numSV;		  //卫星数量
}UBLOX_t;

void Ublox_Init(void);
UBLOX_t Ublox_GetData(void);

#endif


