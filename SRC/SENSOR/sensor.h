#ifndef __SENSOR_H
#define	__SENSOR_H

#include "mathTool.h"
#include "lowPassFilter.h"

//陀螺仪低通滤波截止频率
#define GYRO_LPF_CUT 250

//传感器恒温目标值
#define SENSOR_TEMP_KEPT 55

typedef struct{
	Vector3f_t offset;	    //零偏误差
	Vector3f_t scale;		//比例误差
	bool should_cali;		//传感器校准标志位
    uint8_t step;
} SENSOR_CALI_t;

void ImuTempControlInit(void);
void ImuTempControl(float tempMeasure);

#endif


