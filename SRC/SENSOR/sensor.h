#ifndef __SENSOR_H
#define	__SENSOR_H

#include "mathTool.h"

typedef struct{
	Vector3f_t offset;	    //零偏误差
	Vector3f_t scale;		//比例误差
	bool should_cali;		//传感器校准标志位
} SENSOR_CALI_t;


#endif


