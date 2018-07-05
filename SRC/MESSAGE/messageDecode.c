/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     messageDecoded.c
 * @说明     接收数据解析
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.06
**********************************************************************************************************/
#include "messageDecode.h"
#include "bsklink.h"
#include <string.h>

#include "sensor.h"
#include "gyroscope.h"
#include "magnetometer.h"
#include "accelerometer.h"
#include "barometer.h"
#include "gps.h"
#include "board.h"
#include "ahrs.h"
#include "navigation.h"
#include "flightControl.h"
#include "motor.h"
#include "rc.h"
#include "ublox.h"
#include "flightStatus.h"

static void BsklinkDecodeSensorCaliCmd(BSKLINK_PAYLOAD_SENSOR_CALI_CMD_t payload);

/**********************************************************************************************************
*函 数 名: MessageDecode
*功能说明: 消息解析
*形    参: 接收数据
*返 回 值: 无
**********************************************************************************************************/
void MessageDecode(uint8_t data)
{
    static BSKLINK_MSG_t msg;
    static uint32_t i=0;
    
    if(BsklinkDecode(&msg, data))
    {
        if(msg.msgid == BSKLINK_MSG_ID_FLIGHT_DATA)
        {
            i++;
        }
        else if(msg.msgid == BSKLINK_MSG_ID_SENSOR_CALI_CMD)
        {
            BSKLINK_PAYLOAD_SENSOR_CALI_CMD_t payload;
            memcpy(&payload, msg.payload, msg.length);
            BsklinkDecodeSensorCaliCmd(payload);
        }
        else if(msg.msgid == BSKLINK_MSG_ID_RC_DATA)
        {
            i++;
        }
    }
}

/**********************************************************************************************************
*函 数 名: BsklinkDecodeSensorCaliCmd
*功能说明: 传感器校准命令解析
*形    参: 消息
*返 回 值: 无
**********************************************************************************************************/
static void BsklinkDecodeSensorCaliCmd(BSKLINK_PAYLOAD_SENSOR_CALI_CMD_t payload)
{
    if(payload.type == GYRO)			//陀螺仪
    {
        if(payload.caliFlag == true)
        {
            GyroCalibrateEnable();
        }
    }
    else if(payload.type == ACC)		//加速度计
    {
		if(payload.caliFlag == true)
        {
			AccCalibrateEnable();
		}
    }
    else if(payload.type == MAG)		//磁力计
    {
        if(payload.caliFlag == true)
        {
            MagCalibrateEnable();
        }
    }
    else if(payload.type == ANGLE)		//水平
    {
		if(payload.caliFlag == true)
        {
			LevelCalibrateEnable();
		}
    }
    else if(payload.type == ESC)		//电调
    {
    }
}


