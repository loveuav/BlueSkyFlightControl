/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     mavlinkDecoded.c
 * @说明     mavlink协议解析
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.07
**********************************************************************************************************/
#include "mavlinkDecode.h"
#include "message.h"
#include "common/mavlink.h"
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
#include "pid.h"
#include "motor.h"
#include "rc.h"
#include "ublox.h"
#include "flightStatus.h"


/**********************************************************************************************************
*函 数 名: MavlinkDecode
*功能说明: 消息解析
*形    参: 接收数据
*返 回 值: 无
**********************************************************************************************************/
void MavlinkDecode(uint8_t data)
{
    static mavlink_message_t msg;
    static mavlink_status_t  status;
    static uint32_t i=0;
    
    //对接收到的字节数据进行帧解析，接收完一帧时再继续对帧数据进行解析   
    if(mavlink_parse_char(0, data, &msg, &status) == false)
        return;
    
    if(msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)
    {
        i++;
    }
}


