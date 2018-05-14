/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     ublox.c
 * @说明     GPS数据协议解析，目前只支持UBLOX协议，后续增加对NMEA协议的支持        
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "ublox.h"
#include "drv_usart.h"

//只对POSLLH、SOL、VELNED这3帧进行解析
/*
POSLLH: 	B5 62 01 02 1C 00  		头数据，数据长度28，最后2字节校验
SOL	  :		B5 62 01 06 34 00 		数据长度52，最后2字节校验
VELNED:		B5 62 01 12 24 00		数据长度36，最后2字节校验
*/
//UBLOX协议参考：lindi.iki.fi/lindi/gps/ubx.html

enum{
    POSLLH,
    SOL,
    VELNED
};

typedef struct{
	uint32_t  iTOW;     //GPS Millisecond Time of Week
	int32_t	  lon;      //Longitude
	int32_t	  lat;      //Latitude
	int32_t	  height;   //Height above Ellipsoid
	int32_t	  hMSL;     //Height above mean sea level
	uint32_t  hAcc;     //Horizontal Accuracy Estimate
	uint32_t  vAcc;     //Vertical Accuracy Estimate
	uint16_t  check;
}POSLLH_t;

typedef struct{
	uint32_t   timeofweek;  //GPS Millisecond Time of Week
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
	uint16_t   check;
}SOL_t;

typedef struct{
	uint32_t   timeofweek;	//GPS Millisecond Time of Week	ms
	int32_t    velN;        //NED north velocity	cm/s
	int32_t    velE;        //NED east velocity		cm/s
	int32_t    velD;        //NED down velocity		cm/s
	uint32_t   speed;		//Speed (3-D)			cm/s
	uint32_t   gSpeed;		//Ground Speed (2-D)	cm/s
	int32_t    heading;		//Heading 2-D			deg
	uint32_t   sAcc;		//Speed Accuracy Estimate	cm/s
	uint32_t   cAcc;		//Course / Heading Accuracy Estimate deg
	uint16_t   check;
}VELNED_t;

union 
{
	POSLLH_t data;
	uint8_t temp[30];
}PosllhData;

union
{
	SOL_t data;
	uint8_t temp[54];
}SolData;	

union 
{
	VELNED_t data;
	uint8_t temp[38];
}VelnedData;	

UBLOX_t ublox;

static void Ublox_ProtocolParsing(uint8_t data);

/**********************************************************************************************************
*函 数 名: Ublox_Init
*功能说明: ublox数据解析初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Ublox_Init(void)
{
    Usart_SetIRQCallback(GPS_UART, Ublox_ProtocolParsing);
}

/**********************************************************************************************************
*函 数 名: Ublox_ProtocolParsing
*功能说明: ublox协议解析
*形    参: 输入数据
*返 回 值: 无
**********************************************************************************************************/
static void Ublox_ProtocolParsing(uint8_t data)
{
    static uint8_t gps_data_status = 0;
    static uint8_t gps_data_temp_cnt = 0;
    uint8_t i;
    
    switch(gps_data_status)
    {
        case 0:
            if(data == 0xB5)
            gps_data_status = 1;
            break;
        case 1:
            if(data == 0x62)
                gps_data_status = 2;
            else
                gps_data_status = 0;
            break;
        case 2:
            if(data == 0x01)
                gps_data_status = 3;
            else
                gps_data_status = 0;
            break;
        case 3:
            if(data == 0x02)		//POSLLH
                gps_data_status = 11;
            else if(data == 0x06)	//SOL
                gps_data_status = 21;
            else if(data == 0x12)	//VELNED
                gps_data_status = 31;						
            else
                gps_data_status = 0;
            break;
        case 11:
            if(data == 0x1C)
                gps_data_status = 12;
            else
                gps_data_status = 0;
            break;
        case 12:
            if(data == 0x00)
            {
                gps_data_temp_cnt = 0;
                gps_data_status = 13;
            }
            else
                gps_data_status = 0;
            break;
        case 13:		//POSLLH
            PosllhData.temp[gps_data_temp_cnt++] = data;
            if(gps_data_temp_cnt == 30)		//接收完毕
            {
                uint8_t ck_a=0,ck_b=0;
                gps_data_status = 0;
                ck_a += 1;
                ck_b += ck_a;
                ck_a += 2;
                ck_b += ck_a;
                ck_a += 0x1C;
                ck_b += ck_a;
                ck_b += ck_a;
                for(i=0;i<28;i++)			//数据校验
                {
                    ck_a += PosllhData.temp[i];
                    ck_b += ck_a;
                }
                if(ck_a == PosllhData.temp[28] && ck_b == PosllhData.temp[29])	//校验通过
                {
                    ublox.time = PosllhData.data.iTOW / 1000;
                    ublox.longitude = (double)PosllhData.data.lon / 10000000;
                    ublox.latitude = PosllhData.data.lat;
                    ublox.latitude = (double)PosllhData.data.lat / 10000000;
                    ublox.altitude = (float)PosllhData.data.height / 1000;
                    ublox.hAcc = (float)PosllhData.data.hAcc / 1000;
                }
            }
            break;
        case 21:		//SOL
            if(data == 0x34)
                gps_data_status = 22;
            else
                gps_data_status = 0;
            break;
        case 22:
            if(data == 0x00)
            {
                gps_data_temp_cnt = 0;
                gps_data_status = 23;
            }
            else
                gps_data_status = 0;
            break;
        case 23:
            SolData.temp[gps_data_temp_cnt++] = data;
            if(gps_data_temp_cnt==54)		//接收完毕
            {
                uint8_t ck_a=0,ck_b=0;
                gps_data_status = 0;
                ck_a += 1;
                ck_b += ck_a;
                ck_a += 6;
                ck_b += ck_a;
                ck_a += 0x34;
                ck_b += ck_a;
                ck_b += ck_a;
                for(i=0;i<52;i++)			//数据校验
                {
                    ck_a += SolData.temp[i];
                    ck_b += ck_a;
                }
                if(ck_a == SolData.temp[52] && ck_b == SolData.temp[53])	//校验通过
                {
                    ublox.fixStatus = SolData.data.gpsFix;
                    ublox.numSV = SolData.data.numSV;														
                }
            }
            break;
        case 31:		//VELNED
            if(data==0x24)
                gps_data_status = 32;
            else
                gps_data_status = 0;
            break;
        case 32:
            if(data==0x00)
            {
                gps_data_temp_cnt = 0;
                gps_data_status = 33;
            }
            else
                gps_data_status = 0;
            break;
        case 33:
            VelnedData.temp[gps_data_temp_cnt++] = data;
            if(gps_data_temp_cnt == 38)		//接收完毕
            {
                uint8_t ck_a=0,ck_b=0;
                gps_data_status = 0;
                ck_a += 1;
                ck_b += ck_a;
                ck_a += 0x12;
                ck_b += ck_a;
                ck_a += 0x24;
                ck_b += ck_a;
                ck_b += ck_a;
                for(i=0;i<36;i++)			//数据校验
                {
                    ck_a += VelnedData.temp[i];
                    ck_b += ck_a;
                }
                if(ck_a == VelnedData.temp[36] && ck_b == VelnedData.temp[37])	//校验通过
                {
                    ublox.velN = VelnedData.data.velN;
                    ublox.velE = VelnedData.data.velE;
                    ublox.velD = VelnedData.data.velD;
                }
            }
            break;
    }	
}

UBLOX_t Ublox_GetData(void)
{
	return ublox;
}

