/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     ulog_data.c
 * @说明     ulog数据格式定义
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.08
**********************************************************************************************************/
#include "ulog_data.h"

ULOG_FORMAT_t ulog_data_flight[ULOG_DATA_FLIGHT_NUM] = 
{
    {
        .data_type = "uint64_t",
        .data_name = "timestamp"
    },    
    {
        .data_type = "int16_t",
        .data_name = "roll_rate"
    },
    {
        .data_type = "int16_t",
        .data_name = "pitch_rate"
    },
    {
        .data_type = "int16_t",
        .data_name = "yaw_rate"
    },
    {
        .data_type = "int16_t",
        .data_name = "roll_rate_sp"
    },
    {
        .data_type = "int16_t",
        .data_name = "pitch_rate_sp"
    },
    {
        .data_type = "int16_t",
        .data_name = "yaw_rate_sp"
    },
    {
        .data_type = "int16_t",
        .data_name = "roll"
    },
    {
        .data_type = "int16_t",
        .data_name = "pitch"
    },
    {
        .data_type = "int16_t",
        .data_name = "yaw"
    },
    {
        .data_type = "int16_t",
        .data_name = "roll_sp"
    },
    {
        .data_type = "int16_t",
        .data_name = "pitch_sp"
    },
    {
        .data_type = "int16_t",
        .data_name = "yaw_sp"
    },
    {
        .data_type = "int16_t[3]",
        .data_name = "accel"
    },
    {
        .data_type = "int16_t[3]",
        .data_name = "velocity"
    },
    {
        .data_type = "int16_t[3]",
        .data_name = "velocity_sp"
    },
    {
        .data_type = "int32_t[3]",
        .data_name = "position"
    },
    {
        .data_type = "int32_t[3]",
        .data_name = "position_sp"
    },
};

ULOG_FORMAT_t ulog_data_gps[ULOG_DATA_GPS_NUM] = 
{
    {
        .data_type = "uint64_t",
        .data_name = "timestamp"
    },    
    {
        .data_type = "int32_t",
        .data_name = "latitude"
    },
    {
        .data_type = "int32_t",
        .data_name = "longitude"
    },
    {
        .data_type = "int32_t",
        .data_name = "altitude"
    },
    {
        .data_type = "int32_t",
        .data_name = "velN"
    },
    {
        .data_type = "int32_t",
        .data_name = "velE"
    },
    {
        .data_type = "int32_t",
        .data_name = "velD"
    },
    {
        .data_type = "int16_t",
        .data_name = "heading"
    },
    {
        .data_type = "int16_t",
        .data_name = "hAcc"
    },
    {
        .data_type = "int16_t",
        .data_name = "vAcc"
    },
    {
        .data_type = "int16_t",
        .data_name = "sAcc"
    },
    {
        .data_type = "int16_t",
        .data_name = "cAcc"
    },
    {
        .data_type = "uint8_t",
        .data_name = "fixState"
    },
    {
        .data_type = "uint8_t",
        .data_name = "numSV"
    },
};








