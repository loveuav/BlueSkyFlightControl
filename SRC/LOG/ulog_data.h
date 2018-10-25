#ifndef _ULOG_DATA_H_
#define _ULOG_DATA_H_

#include "mathTool.h"
#include "ulog.h"

#define ULOG_DATA_FLIGHT_NUM 18
#define ULOG_DATA_GPS_NUM    14

#define ULOG_DATA_FLIGHT_ID  0
#define ULOG_DATA_GPS_ID     1

typedef struct
{
    uint64_t timestamp;
    int16_t  roll_rate;
    int16_t  pitch_rate;
    int16_t  yaw_rate;
    int16_t  roll_rate_sp;
    int16_t  pitch_rate_sp;
    int16_t  yaw_rate_sp;
    int16_t  roll;
    int16_t  pitch;
    int16_t  yaw;
    int16_t  roll_sp;
    int16_t  pitch_sp;
    int16_t  yaw_sp;
    int16_t  accel[3];
    int16_t  velocity[3];
    int16_t  velocity_sp[3];
    int32_t  position[3];
    int32_t  position_sp[3];
} ULOG_DATA_FLIGHT_t;

typedef struct
{
    uint64_t timestamp;
    int32_t  latitude;	  //纬度
    int32_t	 longitude;   //经度
    int32_t	 altitude;	  //高度
    int32_t	 velN;		  //北向速度
    int32_t	 velE;		  //东向速度
    int32_t	 velD;		  //天向速度
    int16_t  heading;     //航向
    int16_t	 hAcc;		  //水平定位精度
    int16_t  vAcc;        //垂直定位精度
    int16_t  sAcc;        //速度精度
    int16_t  cAcc;        //航向精度
    uint8_t fixStatus;    //定位状态
    uint8_t numSV;		  //卫星数量
} ULOG_DATA_GPS_t;

extern ULOG_FORMAT_t ulog_data_flight[ULOG_DATA_FLIGHT_NUM];
extern ULOG_FORMAT_t ulog_data_gps[ULOG_DATA_GPS_NUM];

#endif


