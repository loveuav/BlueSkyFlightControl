#ifndef __GPS_H
#define	__GPS_H

#include "sensor.h"

void GpsDataPreTreat(void);
void TransVelToBodyFrame(Vector3f_t velEf, Vector3f_t* velBf, float yaw);
void TransVelToEarthFrame(Vector3f_t velBf, Vector3f_t* velEf, float yaw);
float GetMagDeclination(void);
bool GpsGetFixStatus(void);
Vector3f_t GpsGetVelocity(void);
Vector3f_t GpsGetPosition(void);

#endif















