/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     mavlinkSend.c
 * @说明     mavlink数据帧发送
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.07
**********************************************************************************************************/
#include "mavlinkSend.h"
#include "message.h"
#include "common/mavlink.h"

#include "board.h"
#include "ahrs.h"
#include "navigation.h"
#include "flightControl.h"
#include "gyroscope.h"
#include "magnetometer.h"
#include "accelerometer.h"
#include "barometer.h"
#include "motor.h"
#include "rc.h"
#include "gps.h"
#include "ublox.h"
#include "flightStatus.h"

/**********************************************************************************************************
*函 数 名: MavlinkSendHeartbeat
*功能说明: 发送心跳包
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void MavlinkSendHeartbeat(uint8_t* sendFlag)
{
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;
    uint8_t msgLength;
    uint8_t msgBuffer[MAVLINK_MAX_PAYLOAD_LEN+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
    
    //消息负载赋值
    heartbeat.type          = MAV_TYPE_QUADROTOR;
    heartbeat.autopilot     = MAV_AUTOPILOT_GENERIC;
    heartbeat.base_mode     = 0;
    heartbeat.custom_mode   = 0;
    heartbeat.system_status = 0;

    //mavlink组帧
    mavlink_msg_heartbeat_encode(1, MAV_COMP_ID_AUTOPILOT1, &msg, &heartbeat);
    //消息帧格式化
    msgLength = mavlink_msg_to_send_buffer(msgBuffer, &msg); 
    //发送消息帧
	DataSend(msgBuffer, msgLength);  
}

/**********************************************************************************************************
*函 数 名: MavlinkSendAttitude
*功能说明: 发送角度及角速度
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void MavlinkSendAttitude(uint8_t* sendFlag)
{
    mavlink_message_t msg;
    mavlink_attitude_t attitude;
    uint8_t msgLength;
    uint8_t msgBuffer[MAVLINK_MAX_PAYLOAD_LEN+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
    
    //消息负载赋值
    attitude.time_boot_ms = GetSysTimeMs();
    attitude.roll         = Radians(GetCopterAngle().x);
    attitude.pitch        = Radians(GetCopterAngle().y);
    attitude.yaw          = Radians(GetCopterAngle().z);
    attitude.rollspeed    = Radians(GyroGetData().x);
    attitude.pitchspeed   = Radians(GyroGetData().y);
    attitude.yawspeed     = Radians(GyroGetData().z);
    
    //mavlink组帧
    mavlink_msg_attitude_encode(1, MAV_COMP_ID_AUTOPILOT1, &msg, &attitude);
    //消息帧格式化
    msgLength = mavlink_msg_to_send_buffer(msgBuffer, &msg); 
    //发送消息帧
	DataSend(msgBuffer, msgLength);  
}






































