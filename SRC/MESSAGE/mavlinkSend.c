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
#include "mavlinkParam.h"

#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
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
#include "battery.h"
#include "flightStatus.h"
#include "parameter.h"

static uint16_t currentParamNum = 0;
static uint8_t sendParamListFlag = 0;

mavlink_command_ack_t commandAck;

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
    heartbeat.autopilot     = MAV_AUTOPILOT_PX4;    //设置飞控类型为PX4，以便能使用QGroudControl地面站
    heartbeat.base_mode     = 0;
    heartbeat.custom_mode   = 0;
    heartbeat.system_status = MAV_STATE_STANDBY;

    //mavlink组帧
    mavlink_msg_heartbeat_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &heartbeat);
    //消息帧格式化
    msgLength = mavlink_msg_to_send_buffer(msgBuffer, &msg); 
    //发送消息帧
	DataSend(msgBuffer, msgLength);  
}

/**********************************************************************************************************
*函 数 名: MavlinkSendSysStatus
*功能说明: 发送系统状态
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void MavlinkSendSysStatus(uint8_t* sendFlag)
{
    mavlink_message_t msg;
    mavlink_sys_status_t status;
    uint8_t msgLength;
    uint8_t msgBuffer[MAVLINK_MAX_PAYLOAD_LEN+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
    
    //消息负载赋值
    status.onboard_control_sensors_present = 0xFFFFFFFF;
    status.onboard_control_sensors_enabled = 0xFFFFFFFF;
    status.onboard_control_sensors_health  = 0xFFFFFFFF;
    status.load = Get_OSCPUusage() * 10;
    status.voltage_battery = GetBatteryVoltage() * 10;
    status.current_battery = GetBatteryCurrent() * 10;
    status.battery_remaining = -1;
    
    //mavlink组帧
    mavlink_msg_sys_status_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &status);
    //消息帧格式化
    msgLength = mavlink_msg_to_send_buffer(msgBuffer, &msg); 
    //发送消息帧
	DataSend(msgBuffer, msgLength);  
}

/**********************************************************************************************************
*函 数 名: MavlinkSendParamValue
*功能说明: 发送飞控参数值
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void MavlinkSendParamValue(uint8_t* sendFlag)
{
    mavlink_message_t msg;
    mavlink_param_value_t param_value;
    uint8_t msgLength;
    uint8_t msgBuffer[MAVLINK_MAX_PAYLOAD_LEN+10];

    if(*sendFlag == DISABLE)
        return;

    if(currentParamNum == MAV_PARAM_NUM)
    {
        *sendFlag = DISABLE;
        sendParamListFlag = 0;
        return;
    }
    
    //消息负载赋值
    param_value.param_value = MavParamGetValue(currentParamNum);
    param_value.param_count = MAV_PARAM_NUM;
    param_value.param_index = currentParamNum;   
    memset(param_value.param_id, 0, 16);
    memcpy(param_value.param_id, MavParamGetString(currentParamNum), strlen(MavParamGetString(currentParamNum)));
    param_value.param_type = MAVLINK_TYPE_FLOAT;
    
    //判断是发送参数列表还是单一参数
    if(sendParamListFlag)
    {
        currentParamNum++;
    }
    else
    {
        currentParamNum = 0;
        *sendFlag = DISABLE;
    }
   
    //mavlink组帧
    mavlink_msg_param_value_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &param_value);
    //消息帧格式化
    msgLength = mavlink_msg_to_send_buffer(msgBuffer, &msg); 
    //发送消息帧
	DataSend(msgBuffer, msgLength);  
}

/**********************************************************************************************************
*函 数 名: MavlinkCurrentParamSet
*功能说明: 设置飞控参数发送计数值
*形    参: 发送参数起始序号 列表发送标志
*返 回 值: 无
**********************************************************************************************************/
void MavlinkCurrentParamSet(uint16_t num, uint8_t flag)
{
    currentParamNum = num;
    sendParamListFlag = flag;
}

/**********************************************************************************************************
*函 数 名: MavlinkSendGpsRawInt
*功能说明: 发送GPS原始数据
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void MavlinkSendGpsRawInt(uint8_t* sendFlag)
{
    mavlink_message_t msg;
    mavlink_gps_raw_int_t gps;
    uint8_t msgLength;
    uint8_t msgBuffer[MAVLINK_MAX_PAYLOAD_LEN+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
    
    //消息负载赋值
    gps.time_usec = Ublox_GetData().time * 1000;
    gps.lat       = Ublox_GetData().latitude * (double)1e7;
    gps.lon       = Ublox_GetData().longitude * (double)1e7;
    gps.alt       = Ublox_GetData().altitude * 1000;
    gps.eph       = Ublox_GetData().hAcc * 100;
    gps.epv       = Ublox_GetData().vAcc * 100;
    gps.vel       = Pythagorous2(Ublox_GetData().velN, Ublox_GetData().velE);
    gps.cog       = 0xFFFF;
    gps.fix_type  = Ublox_GetData().fixStatus;
    gps.satellites_visible = Ublox_GetData().numSV;
    
    //mavlink组帧
    mavlink_msg_gps_raw_int_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &gps);
    //消息帧格式化
    msgLength = mavlink_msg_to_send_buffer(msgBuffer, &msg); 
    //发送消息帧
	DataSend(msgBuffer, msgLength);  
}

/**********************************************************************************************************
*函 数 名: MavlinkSendScaledImu
*功能说明: 发送IMU数据
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void MavlinkSendScaledImu(uint8_t* sendFlag)
{
    mavlink_message_t msg;
    mavlink_scaled_imu_t scaled_imu;
    uint8_t msgLength;
    uint8_t msgBuffer[MAVLINK_MAX_PAYLOAD_LEN+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
    
    //消息负载赋值
    scaled_imu.time_boot_ms = GetSysTimeMs();
    scaled_imu.xacc         = AccGetData().x * 1000;
    scaled_imu.yacc         = AccGetData().y * 1000;
    scaled_imu.zacc         = AccGetData().z * 1000;
    scaled_imu.xgyro        = Radians(GyroGetData().x) * 1000;    
    scaled_imu.ygyro        = Radians(GyroGetData().y) * 1000;         
    scaled_imu.zgyro        = Radians(GyroGetData().z) * 1000;  
    scaled_imu.xmag         = MagGetData().x * 1000;   
    scaled_imu.ymag         = MagGetData().y * 1000;  
    scaled_imu.zmag         = MagGetData().z * 1000;  
    
    //mavlink组帧
    mavlink_msg_scaled_imu_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &scaled_imu);
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
    mavlink_msg_attitude_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &attitude);
    //消息帧格式化
    msgLength = mavlink_msg_to_send_buffer(msgBuffer, &msg); 
    //发送消息帧
	DataSend(msgBuffer, msgLength);  
}

/**********************************************************************************************************
*函 数 名: MavlinkSendCommandAck
*功能说明: 发送命令回应
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void MavlinkSendCommandAck(uint8_t* sendFlag)
{
    mavlink_message_t msg;
    uint8_t msgLength;
    uint8_t msgBuffer[MAVLINK_MAX_PAYLOAD_LEN+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
    
    //mavlink组帧
    mavlink_msg_command_ack_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &commandAck);
    //消息帧格式化
    msgLength = mavlink_msg_to_send_buffer(msgBuffer, &msg); 
    //发送消息帧
	DataSend(msgBuffer, msgLength);  
}

/**********************************************************************************************************
*函 数 名: MavlinkCurrentParamSet
*功能说明: 设置飞控参数发送计数值
*形    参: 发送参数起始序号 列表发送标志
*返 回 值: 无
**********************************************************************************************************/
void MavlinkSetCommandAck(uint16_t command, uint8_t result)
{
    commandAck.command = command;
    commandAck.result  = result;
}

/**********************************************************************************************************
*函 数 名: MavlinkSendAttitude
*功能说明: 发送角度及角速度
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void MavlinkSendStatusText(uint8_t* sendFlag)
{
    mavlink_message_t msg;
    mavlink_statustext_t statustext;
    uint8_t msgLength;
    uint8_t msgBuffer[MAVLINK_MAX_PAYLOAD_LEN+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
    
    //消息负载赋值

    
    //mavlink组帧
    mavlink_msg_statustext_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &statustext);
    //消息帧格式化
    msgLength = mavlink_msg_to_send_buffer(msgBuffer, &msg); 
    //发送消息帧
	DataSend(msgBuffer, msgLength);  
}



