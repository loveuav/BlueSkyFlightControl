#ifndef __GPS_H
#define	__GPS_H

#include "sensor.h"

void GpsDataPreTreat(void);
void TransVelToBodyFrame(Vector3f_t velEf, Vector3f_t* velBf, float yaw);
void TransVelToEarthFrame(Vector3f_t velBf, Vector3f_t* velEf, float yaw);
float GetMagDeclination(void);
bool GpsGetFixStatus(void);
float GpsGetAccuracy(void);
Vector3f_t GpsGetVelocity(void);
Vector3f_t GpsGetPosition(void);

void GpsResetHomePosition(void);
void GpsTransToLocalPosition(Vector3f_t* position, double lat, double lon);

float GetDirectionToHome(Vector3f_t position);
float GetDistanceToHome(Vector3f_t position);
float GetDirectionOfTwoPoint(Vector3f_t point1, Vector3f_t point2);
Vector3f_t GetHomePosition(void);
void GetHomeLatitudeAndLongitude(double* lat, double* lon);

#endif















