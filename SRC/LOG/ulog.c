/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     ulog.c
 * @说明     ulog日志
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.08
**********************************************************************************************************/
#include "ulog.h"
#include "ulog_data.h"
#include "logger.h"
#include <string.h>

#include "board.h"
#include "ahrs.h"
#include "navigation.h"
#include "flightControl.h"
#include "gyroscope.h"
#include "magnetometer.h"
#include "accelerometer.h"
#include "barometer.h"
#include "ublox.h"

/**********************************************************************************************************
*函 数 名: UlogWriteHeader
*功能说明: ulog写入日志固定头部信息
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void UlogWriteHeader(void)
{
	ulog_file_header_s header;
    
	header.magic[0] = 'U';
	header.magic[1] = 'L';
	header.magic[2] = 'o';
	header.magic[3] = 'g';
	header.magic[4] = 0x01;
	header.magic[5] = 0x12;
	header.magic[6] = 0x35;
	header.magic[7] = 0x01; //版本1
	header.timestamp = GetSysTimeUs();

    LoggerWrite(&header, sizeof(header));
}

/**********************************************************************************************************
*函 数 名: UlogWriteFlag
*功能说明: ulog写入Flag
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void UlogWriteFlag(void)
{
	ulog_message_flag_bits_s flag;
    
    memset(&flag, 0, sizeof(flag));
    
    flag.msg_size = 40;
    flag.msg_type = FLAG_BITS;

    LoggerWrite(&flag, sizeof(flag));
}

/**********************************************************************************************************
*函 数 名: UlogWriteFormat
*功能说明: ulog写入数据格式定义
*形    参: 格式类型 格式内容指针 格式内容数量
*返 回 值: 无
**********************************************************************************************************/
void UlogWriteFormat(char* formatType, ULOG_FORMAT_t* ulog_format, int16_t dataNum)
{
	ulog_message_format_s format;
    uint16_t dataCnt = 0;
    
    memset(&format, 0, sizeof(format));
    
    format.msg_size           = 0;
    format.msg_type           = FORMAT;
    
    for(uint8_t i=0; *(formatType + i) != '\0' ;i++)
    {
        format.format[dataCnt++] = *(formatType + i);
    }

    format.format[dataCnt++]  = ':';
    
    for(uint8_t i=0; i<dataNum; i++)
    {
        //写入数据类型
        memcpy(format.format+dataCnt, ulog_format[i].data_type, strlen(ulog_format[i].data_type));
        dataCnt += strlen(ulog_format[i].data_type);
        //数据类型与数据名称之间增加空格
        format.format[dataCnt++]  = ' ';
        //写入数据名称
        memcpy(format.format+dataCnt, ulog_format[i].data_name, strlen(ulog_format[i].data_name));
        dataCnt += strlen(ulog_format[i].data_name);
        //增加分号作为分隔符
        format.format[dataCnt++]  = ';';
    }
    
    format.msg_size = dataCnt;
    
    LoggerWrite(&format, dataCnt+3);
}

/**********************************************************************************************************
*函 数 名: UlogWriteAddLogged
*功能说明: ulog写入AddLogged
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void UlogWriteAddLogged(char* logType, uint8_t msg_id)
{
	ulog_message_add_logged_s add_logged;
    uint16_t dataCnt = 0;
    
    memset(&add_logged, 0, sizeof(add_logged));
    
    add_logged.msg_size = 0;
    add_logged.msg_type = ADD_LOGGED_MSG;
    add_logged.multi_id = 0;
    add_logged.msg_id   = msg_id;
    
    for(uint8_t i=0; *(logType + i) != '\0' ;i++)
    {
        add_logged.message_name[dataCnt++] = *(logType + i);
    }
    
    add_logged.msg_size = 3 + dataCnt;
    
    LoggerWrite(&add_logged, add_logged.msg_size+3);
}

/**********************************************************************************************************
*函 数 名: UlogWriteData_Flight
*功能说明: ulog写入数据
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void UlogWriteData_Flight(void)
{
    ulog_message_data_header_s header;
    ULOG_DATA_FLIGHT_t data;
    
    header.msg_size = sizeof(data) + 2; //消息ID长度包括在内
    header.msg_type = DATA;
    header.msg_id   = ULOG_DATA_FLIGHT_ID;

    data.timestamp      = GetSysTimeUs();
    data.roll_rate      = Radians(GyroGetData().x) * 1000; 
    data.pitch_rate     = Radians(GyroGetData().y) * 1000; 
    data.yaw_rate       = Radians(GyroGetData().z) * 1000;    
    data.roll_rate_sp   = 0; 
    data.pitch_rate_sp  = 0; 
    data.yaw_rate_sp    = 0;        
    data.roll           = GetCopterAngle().x * 10;
    data.pitch          = GetCopterAngle().y * 10;   
    data.yaw            = GetCopterAngle().z * 10;
    data.roll_sp        = 0; 
    data.pitch_sp       = 0; 
    data.yaw_sp         = 0; 
    data.accel[0]       = GetCopterAccel().x * GRAVITY_ACCEL * 100;
    data.accel[1]       = GetCopterAccel().y * GRAVITY_ACCEL * 100; 
    data.accel[2]       = GetCopterAccel().z * GRAVITY_ACCEL * 100;
    data.velocity[0]    = GetCopterVelocity().x;
    data.velocity[1]    = GetCopterVelocity().y;
    data.velocity[2]    = GetCopterVelocity().z;
    data.velocity_sp[0] = 0;
    data.velocity_sp[1] = 0;
    data.velocity_sp[2] = 0;    
    data.position[0]    = GetCopterPosition().x;
    data.position[1]    = GetCopterPosition().y;
    data.position[2]    = GetCopterPosition().z;
    data.position_sp[0] = 0;
    data.position_sp[1] = 0;
    data.position_sp[2] = 0;    
    
    LoggerWrite(&header, sizeof(header));
    LoggerWrite(&data, sizeof(data));
}

/**********************************************************************************************************
*函 数 名: UlogWriteData_GPS
*功能说明: ulog写入数据
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void UlogWriteData_GPS(void)
{
    ulog_message_data_header_s header;
    ULOG_DATA_GPS_t data;
    
    header.msg_size = sizeof(data) + 2; //消息ID长度包括在内
    header.msg_type = DATA;
    header.msg_id   = ULOG_DATA_GPS_ID;

    data.timestamp   = GetSysTimeUs();
    data.latitude    = Ublox_GetData().latitude * 1e7;
    data.longitude   = Ublox_GetData().longitude * 1e7;
    data.altitude    = Ublox_GetData().altitude * 100;
    data.velN        = Ublox_GetData().velN;		 
    data.velE        = Ublox_GetData().velE;		
    data.velD        = Ublox_GetData().velD;		
    data.heading     = Ublox_GetData().heading;	   
    data.hAcc        = Ublox_GetData().hAcc * 100;	 
    data.vAcc        = Ublox_GetData().vAcc * 100;	  
    data.sAcc        = Ublox_GetData().sAcc * 100;	      
    data.cAcc        = Ublox_GetData().cAcc * 100;	
    data.fixStatus   = Ublox_GetData().fixStatus;
    data.numSV       = Ublox_GetData().numSV;	
    
    LoggerWrite(&header, sizeof(header));
    LoggerWrite(&data, sizeof(data));
}


