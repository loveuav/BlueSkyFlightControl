#ifndef __GPS_H
#define	__GPS_H

#include "sensor.h"

void GpsDataPreTreat(void);
void TransVelToBodyFrame(Vector3f_t velEf, Vector3f_t* velBf, float yaw);
void TransVelToEarthFrame(Vector3f_t velBf, Vector3f_t* velEf, float yaw);
float GetMagDeclination(void);
bool GetGpsFixStatus(void);
Vector3f_t GetGpsVelocity(void);
Vector3f_t GetGpsPosition(void);

#endif















