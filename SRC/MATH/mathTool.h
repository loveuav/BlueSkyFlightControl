#ifndef __MATHTOOL_H__
#define __MATHTOOL_H__

#include <stdint.h>
#include <stdlib.h>
#include "stdbool.h"
#include <math.h>

#include "vector3.h"

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))

#define M_PI                 3.141592653f	//圆周率
#define DEG_TO_RAD           0.01745329f	//角度转弧度
#define RAD_TO_DEG           57.29577951f	//弧度转角度

#define EARTH_RADIUS         6371.004f      //km
#define GRAVITY_ACCEL        9.8f          //重力加速度 单位：m/s^2

#define HALF_SQRT_2          0.70710678118654757f	//根号2的值

float SafeArcsin(float v);
float ConstrainFloat(float amt, float low, float high);
int16_t ConstrainInt16(int16_t amt, int16_t low, int16_t high);
uint16_t ConstrainUint16(uint16_t amt, uint16_t low, uint16_t high);
int32_t ConstrainInt32(int32_t amt, int32_t low, int32_t high);

int32_t ApplyDeadbandInt(int32_t value, int32_t deadband);
float ApplyDeadbandFloat(float value, float deadband);

float Radians(float deg);
float Degrees(float rad);

float Sq(float v);
float Pythagorous2(float a, float b);
float Pythagorous3(float a, float b, float c);
float Pythagorous4(float a, float b, float c, float d);

float WrapDegree360(float angle);

int32_t GetRandom(void);

#endif







