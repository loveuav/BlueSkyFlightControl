#ifndef __UBLOX_H
#define	__UBLOX_H

#include "mathTool.h"

#define UBLOX_SYNC1	    0xB5
#define UBLOX_SYNC2	    0x62

#define UBLOX_NAV_CLASS	    0x01
#define UBLOX_RXM_CLASS	    0x02
#define UBLOX_CFG_CLASS	    0x06
#define UBLOX_MON_CLASS	    0x0a
#define UBLOX_AID_CLASS	    0x0b
#define UBLOX_TIM_CLASS	    0x0d

#define UBLOX_NAV_POSLLH    0x02
#define UBLOX_NAV_DOP	    0x04
#define UBLOX_NAV_SOL       0x06
#define UBLOX_NAV_PVT       0x07
#define UBLOX_NAV_VALNED    0x12
#define UBLOX_NAV_TIMEUTC   0x21
#define UBLOX_NAV_SBAS	    0x32
#define UBLOX_NAV_SVINFO    0x30

#define UBLOX_AID_REQ	    0x00

#define UBLOX_RXM_RAW	    0x10
#define UBLOX_RXM_SFRB	    0x11

#define UBLOX_MON_VER	    0x04
#define UBLOX_MON_HW	    0x09

#define UBLOX_TIM_TP	    0x01

#define UBLOX_CFG_PRT       0x00
#define UBLOX_CFG_MSG	    0x01
#define UBLOX_CFG_TP	    0x07
#define UBLOX_CFG_RTATE	    0x08
#define UBLOX_CFG_CFG	    0x09
#define UBLOX_CFG_SBAS	    0x16
#define UBLOX_CFG_NAV5	    0x24

#define UBLOX_SBAS_AUTO	    0x00000000
#define UBLOX_SBAS_WAAS	    0x0004E004
#define UBLOX_SBAS_EGNOS    0x00000851
#define UBLOX_SBAS_MSAS	    0x00020200
#define UBLOX_SBAS_GAGAN    0x00000108

#define UBLOX_MAX_PAYLOAD   256

typedef struct {
    float   time;		  //时间
    double	latitude;	  //纬度
    double	longitude;    //经度
    float	altitude;	  //高度
    int32_t	velN;		  //北向速度
    int32_t	velE;		  //东向速度
    int32_t	velD;		  //天向速度
    int32_t speed;        //地面速度
    float   heading;      //航向
    float	hAcc;		  //水平定位精度
    float   vAcc;         //垂直定位精度
    float   sAcc;         //速度精度
    float   cAcc;         //航向精度
    uint8_t fixStatus;    //定位状态
    uint8_t numSV;		  //卫星数量
} UBLOX_t;

enum
{
    UBLOX_WAIT_SYNC1 = 0,
    UBLOX_WAIT_SYNC2,
    UBLOX_WAIT_CLASS,
    UBLOX_WAIT_ID,
    UBLOX_WAIT_LEN1,
    UBLOX_WAIT_LEN2,
    UBLOX_PAYLOAD,
    UBLOX_CHECK1,
    UBLOX_CHECK2
};

#pragma pack (1)

typedef struct {
    uint32_t  iTOW;     //GPS Millisecond Time of Week
    int32_t	  lon;      //Longitude
    int32_t	  lat;      //Latitude
    int32_t	  height;   //Height above Ellipsoid
    int32_t	  hMSL;     //Height above mean sea level
    uint32_t  hAcc;     //Horizontal Accuracy Estimate
    uint32_t  vAcc;     //Vertical Accuracy Estimate
} UBLOX_POSLLH_t;

typedef struct {
    uint32_t   iTOW;        //GPS Millisecond Time of Week
    int32_t    fTOW;        //Fractional Nanoseconds remainder of rounded ms above, range -500000 .. 500000
    int16_t    week;        //GPS week (GPS time)
    int8_t     gpsFix;      //GPSfix Type, range 0..4
    int8_t     flags;       //Fix Status Flags
    int32_t    ecefX;       //ECEF X coordinate
    int32_t    ecefY;       //ECEF Y velocity
    int32_t    ecefZ;       //ECEF Z velocity
    uint32_t   pAcc;        //3D Position Accuracy Estimate
    int32_t    ecefVX;      //ECEF X velocity
    int32_t    ecefVY;      //ECEF Y velocity
    int32_t    ecefVZ;      //ECEF Z velocity
    uint32_t   sAcc;        //Speed Accuracy Estimate
    uint16_t   pDOP;        //Position DOP
    uint8_t    res1;        //reserved
    uint8_t    numSV;       //Number of SVs used in Nav Solution
    uint32_t   res2;        //reserved
} UBLOX_SOL_t;

typedef struct {
    uint32_t   iTOW;	    //GPS Millisecond Time of Week	ms
    int32_t    velN;        //NED north velocity	cm/s
    int32_t    velE;        //NED east velocity		cm/s
    int32_t    velD;        //NED down velocity		cm/s
    uint32_t   speed;		//Speed (3-D)			cm/s
    uint32_t   gSpeed;		//Ground Speed (2-D)	cm/s
    int32_t    heading;		//Heading 2-D			deg
    uint32_t   sAcc;		//Speed Accuracy Estimate	cm/s
    uint32_t   cAcc;		//Course / Heading Accuracy Estimate deg
} UBLOX_VELNED_t;

typedef struct {
    uint32_t   iTOW;	    //GPS Millisecond Time of Week	ms
    uint32_t   tAcc;        //Time Accuracy Estimate
    int32_t    nano;        //Nanoseconds of second, range -500000000 .. 500000000 (UTC)
    uint16_t   year;        //Year, range 1999..2099 (UTC)
    uint8_t    month;		//Month, range 1..12 (UTC)
    uint8_t    day;		    //Day of Month, range 1..31 (UTC)
    uint8_t    hour;		//Hour of Day, range 0..23 (UTC)
    uint8_t    min;		    //Minute of Hour, range 0..59 (UTC)
    uint8_t    sec;		    //Seconds of Minute, range 0..59 (UTC)
    uint8_t    valid;       //Validity Flags
} UBLOX_TIMEUTC_t;

typedef struct {
    uint32_t   iTOW;	    //GPS Millisecond Time of Week	ms
    uint16_t   year;        //Year, range 1999..2099 (UTC)
    uint8_t    month;		//Month, range 1..12 (UTC)
    uint8_t    day;		    //Day of Month, range 1..31 (UTC)
    uint8_t    hour;		//Hour of Day, range 0..23 (UTC)
    uint8_t    min;		    //Minute of Hour, range 0..59 (UTC)
    uint8_t    sec;		    //Seconds of Minute, range 0..59 (UTC)
    uint8_t    valid;       //Validity Flags
    uint32_t   tAcc;        //Time Accuracy Estimate
    int32_t    nano;        //Fraction of second, range -1e9 .. 1e9 (UTC)
    uint8_t    gpsFix;      //GPSfix Type, range 0..5 0: no fix 1: dead reckoning only 2: 2D-fix 3: 3D-fix 4: GNSS + dead reckoning combined 5: time only fix
    uint8_t    flags;       //Fix status flags
    uint8_t    flags2;      //Additional flags
    uint8_t    numSV;       //Number of SVs used in Nav Solution
    int32_t	   lon;         //Longitude
    int32_t	   lat;         //Latitude
    int32_t	   height;      //Height above Ellipsoid
    int32_t	   hMSL;        //Height above mean sea level
    uint32_t   hAcc;        //Horizontal Accuracy Estimate
    uint32_t   vAcc;        //Vertical Accuracy Estimate
    int32_t    velN;        //NED north velocity	cm/s
    int32_t    velE;        //NED east velocity		cm/s
    int32_t    velD;        //NED down velocity		cm/s
    int32_t    gSpeed;		//Ground Speed (2-D)	cm/s
    int32_t    heading;		//Heading 2-D			deg
    uint32_t   sAcc;		//Speed Accuracy Estimate	cm/s
    uint32_t   cAcc;		//Course / Heading Accuracy Estimate deg
    uint16_t   pDOP;        //Position DOP
    uint8_t    res1;        //reserved
    int32_t    headVeh;     //Heading of vehicle (2-D)
    uint8_t    res2;        //reserved
} UBLOX_PVT_t;

typedef struct {
    union
    {
        UBLOX_POSLLH_t  posllh;
        UBLOX_VELNED_t  velned;
        UBLOX_SOL_t     sol;
        UBLOX_TIMEUTC_t time;
        UBLOX_PVT_t     pvt;
        char other[UBLOX_MAX_PAYLOAD];
    } payload;

    uint8_t  state;
    uint16_t count;
    uint8_t  class;
    uint8_t  id;
    uint16_t length;
    uint16_t checksumErrors;

    uint8_t  ubloxRxCK_A;
    uint8_t  ubloxRxCK_B;

    uint8_t  ubloxTxCK_A;
    uint8_t  ubloxTxCK_B;
} UBLOX_t_RAW_t;

typedef struct {
    uint16_t   year;
    uint8_t    month;
    uint8_t    day;
    uint8_t    hour;
    uint8_t    min;
    uint8_t    sec;
} UTC_TIME_t;

#pragma pack ()

void Ublox_Init(void);
UBLOX_t Ublox_GetData(void);
UTC_TIME_t GetUTCTime(void);

#endif


