#ifndef __SENSOR_H
#define	__SENSOR_H

#include "mathTool.h"
#include "lowPassFilter.h"

#define GYRO_LPF_CUT 250

typedef struct{
	Vector3f_t offset;	    //零偏误差
	Vector3f_t scale;		//比例误差
	bool should_cali;		//传感器校准标志位
    uint8_t step;
} SENSOR_CALI_t;


#endif


