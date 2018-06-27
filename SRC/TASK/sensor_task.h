#ifndef __SENSOR_TASK_H__
#define __SENSOR_TASK_H__

void SensorTaskCreate(void);

int16_t	GetImuDataPreTreatTaskStackRemain(void);
int16_t	GetOtherSensorTaskStackRemain(void);

#endif


