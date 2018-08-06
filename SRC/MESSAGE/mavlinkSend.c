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
#include "mavlinkNotice.h"

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
#include "waypointControl.h"

static int16_t currentParamNum = 0;

mavlink_command_ack_t commandAck;
mavlink_statustext_t  statustext;

uint8_t statusTextSendFlag[MAV_NOTICE_NUM];

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
    heartbeat.base_mode     = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    
    if(GetArmedStatus() == ARMED)
    {
        heartbeat.base_mode |= MAV_MODE_MANUAL_ARMED;
    }
    else
    {
        heartbeat.base_mode |= MAV_MODE_MANUAL_DISARMED;        
    }
    
    switch(GetFlightMode())
    {
        case MANUAL:
            heartbeat.custom_mode   = 0x0001 << 16;
            break;
        
        case SEMIAUTO:
            heartbeat.custom_mode   = 0x0002 << 16;
            break;
        
        case AUTO:
            heartbeat.custom_mode   = 0x0003 << 16;
            break;
        
        case AUTOPILOT:
            heartbeat.custom_mode   = 0x0404 << 16;
            break;
        
        default:
            heartbeat.custom_mode   = 0x0003 << 16;
            break;
    }
    
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
    status.onboard_control_sensors_present = 0;
    status.onboard_control_sensors_enabled = 0;
    status.onboard_control_sensors_health  = 0;
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
    else
       *sendFlag = DISABLE;
    
    //消息负载赋值
    param_value.param_value = MavParamGetValue(currentParamNum);
    param_value.param_count = MAV_PARAM_NUM;
    param_value.param_index = currentParamNum;   
    memset(param_value.param_id, 0, 16);
    memcpy(param_value.param_id, MavParamGetString(currentParamNum), strlen(MavParamGetString(currentParamNum)));
    param_value.param_type = MAVLINK_TYPE_FLOAT;
   
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
*形    参: 发送参数起始序号
*返 回 值: 无
**********************************************************************************************************/
void MavlinkCurrentParamSet(uint16_t num)
{
    currentParamNum = num;
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
    gps.vel       = Ublox_GetData().speed;
    gps.cog       = Ublox_GetData().heading * 100;
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
*函 数 名: MavlinkSendLocalPositionNed
*功能说明: 发送位置信息
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void MavlinkSendLocalPositionNed(uint8_t* sendFlag)
{
    mavlink_message_t msg;
    mavlink_local_position_ned_t position;
    uint8_t msgLength;
    uint8_t msgBuffer[MAVLINK_MAX_PAYLOAD_LEN+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
    
    //消息负载赋值
    position.time_boot_ms = GetSysTimeMs();
    position.x            = GetCopterPosition().x;
    position.y            = GetCopterPosition().y;
    position.z            = -GetCopterPosition().z;
    Vector3f_t velEf;
    TransVelToEarthFrame(GetCopterVelocity(), &velEf, GetCopterAngle().z);
    position.vx           = velEf.x;
    position.vy           = velEf.y;
    position.vz           = -velEf.z;
    
    //mavlink组帧
    mavlink_msg_local_position_ned_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &position);
    //消息帧格式化
    msgLength = mavlink_msg_to_send_buffer(msgBuffer, &msg); 
    //发送消息帧
	DataSend(msgBuffer, msgLength);  
}

/**********************************************************************************************************
*函 数 名: MavlinkSendVfrHud
*功能说明: 发送HUD信息
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void MavlinkSendVfrHud(uint8_t* sendFlag)
{
    mavlink_message_t msg;
    mavlink_vfr_hud_t hud;
    uint8_t msgLength;
    uint8_t msgBuffer[MAVLINK_MAX_PAYLOAD_LEN+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
    
    //消息负载赋值
    hud.airspeed    = 0;
    hud.groundspeed = (float)Pythagorous2(GetCopterVelocity().x, GetCopterVelocity().y) * 0.01f;
    hud.alt         = (float)GetCopterPosition().z * 0.01f;
    hud.climb       = (float)GetCopterVelocity().z * 0.01f;
    hud.heading     = GetCopterAngle().z;
    hud.throttle    = 0xFFFF;

    //mavlink组帧
    mavlink_msg_vfr_hud_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &hud);
    //消息帧格式化
    msgLength = mavlink_msg_to_send_buffer(msgBuffer, &msg); 
    //发送消息帧
	DataSend(msgBuffer, msgLength);  
}

/**********************************************************************************************************
*函 数 名: MavlinkSendRcChannels
*功能说明: 发送遥控通道数据
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void MavlinkSendRcChannels(uint8_t* sendFlag)
{
    mavlink_message_t msg;
    mavlink_rc_channels_t rc;
    uint8_t msgLength;
    uint8_t msgBuffer[MAVLINK_MAX_PAYLOAD_LEN+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
    
    //消息负载赋值
    rc.time_boot_ms = GetSysTimeMs();
    rc.chancount    = 8;
    rc.chan1_raw    = GetRcData().roll;
    rc.chan2_raw    = GetRcData().pitch;
    rc.chan3_raw    = GetRcData().throttle;
    rc.chan4_raw    = GetRcData().yaw;
    rc.chan5_raw    = GetRcData().aux1;
    rc.chan6_raw    = GetRcData().aux2;
    rc.chan7_raw    = GetRcData().aux3;
    rc.chan8_raw    = GetRcData().aux4;
    rc.chan9_raw    = GetRcData().aux5;
    rc.chan10_raw   = GetRcData().aux6;
    rc.chan11_raw   = GetRcData().aux7;
    rc.chan12_raw   = GetRcData().aux8;
    rc.chan13_raw   = GetRcData().aux9;
    rc.chan14_raw   = GetRcData().aux10;
    rc.chan15_raw   = GetRcData().aux11;
    rc.chan16_raw   = GetRcData().aux12;
    rc.chan17_raw   = 0xFFFF;
    rc.chan18_raw   = 0xFFFF;
    rc.rssi         = 0xFF;
    
    //mavlink组帧
    mavlink_msg_rc_channels_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &rc);
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
*函 数 名: MavlinkSendMissionRequest
*功能说明: 发送航点请求
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void MavlinkSendMissionRequest(uint8_t* sendFlag)
{
    mavlink_message_t msg;
    mavlink_mission_request_t request;
    uint8_t msgLength;
    uint8_t msgBuffer[MAVLINK_MAX_PAYLOAD_LEN+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
    
    //消息负载赋值
    request.seq = GetWaypointRecvCount();
    request.target_component = MAVLINK_SYSTEM_ID;
    request.target_system    = MAVLINK_COMPONENT_ID;
    
    //mavlink组帧
    mavlink_msg_mission_request_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &request);
    //消息帧格式化
    msgLength = mavlink_msg_to_send_buffer(msgBuffer, &msg); 
    //发送消息帧
	DataSend(msgBuffer, msgLength);  
}

/**********************************************************************************************************
*函 数 名: MavlinkSendMissionAck
*功能说明: 发送航点接收应答
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void MavlinkSendMissionAck(uint8_t* sendFlag)
{
    mavlink_message_t msg;
    mavlink_mission_ack_t ack;
    uint8_t msgLength;
    uint8_t msgBuffer[MAVLINK_MAX_PAYLOAD_LEN+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
    
    //消息负载赋值
    ack.type = MAV_MISSION_ACCEPTED;
    ack.target_component = MAVLINK_SYSTEM_ID;
    ack.target_system    = MAVLINK_COMPONENT_ID;
    
    //mavlink组帧
    mavlink_msg_mission_ack_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &ack);
    //消息帧格式化
    msgLength = mavlink_msg_to_send_buffer(msgBuffer, &msg); 
    //发送消息帧
	DataSend(msgBuffer, msgLength);  
}

/**********************************************************************************************************
*函 数 名: MavlinkSendMissionCount
*功能说明: 发送航点数量
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void MavlinkSendMissionCount(uint8_t* sendFlag)
{
    mavlink_message_t msg;
    mavlink_mission_count_t count;
    uint8_t msgLength;
    uint8_t msgBuffer[MAVLINK_MAX_PAYLOAD_LEN+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
    
    //消息负载赋值
    count.count = GetWaypointCount();
    count.target_component = MAVLINK_SYSTEM_ID;
    count.target_system    = MAVLINK_COMPONENT_ID;
    
    //mavlink组帧
    mavlink_msg_mission_count_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &count);
    //消息帧格式化
    msgLength = mavlink_msg_to_send_buffer(msgBuffer, &msg); 
    //发送消息帧
	DataSend(msgBuffer, msgLength);  
}

/**********************************************************************************************************
*函 数 名: MavlinkSendMissionItem
*功能说明: 发送航点信息
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void MavlinkSendMissionItem(uint8_t* sendFlag)
{
    mavlink_message_t msg;
    mavlink_mission_item_t item;
    uint8_t msgLength;
    uint8_t msgBuffer[MAVLINK_MAX_PAYLOAD_LEN+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
    
    //消息负载赋值
    item = GetWaypointItem(GetWaypointSendCount());
    
    //mavlink组帧
    mavlink_msg_mission_item_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &item);
    //消息帧格式化
    msgLength = mavlink_msg_to_send_buffer(msgBuffer, &msg); 
    //发送消息帧
	DataSend(msgBuffer, msgLength);  
}

/**********************************************************************************************************
*函 数 名: MavlinkSendHomePosition
*功能说明: 发送Home点位置
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void MavlinkSendHomePosition(uint8_t* sendFlag)
{
    mavlink_message_t msg;
    mavlink_home_position_t home;
    uint8_t msgLength;
    uint8_t msgBuffer[MAVLINK_MAX_PAYLOAD_LEN+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
    
    //消息负载赋值
    double latitude, longitude;
    GetHomeLatitudeAndLongitude(&latitude, &longitude);
    
    home.latitude  = latitude * (double)1e7;
    home.longitude = longitude * (double)1e7;
    home.altitude  = 0;
    home.x         = GetHomePosition().x;
    home.y         = GetHomePosition().y;
    home.z         = 0;
    
    //mavlink组帧
    mavlink_msg_home_position_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &home);
    //消息帧格式化
    msgLength = mavlink_msg_to_send_buffer(msgBuffer, &msg); 
    //发送消息帧
	DataSend(msgBuffer, msgLength);  
}

/**********************************************************************************************************
*函 数 名: MavlinkSendStatusText
*功能说明: 发送状态文本
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void MavlinkSendStatusText(uint8_t* sendFlag)
{
    mavlink_message_t msg;
    uint8_t msgLength;
    uint8_t msgBuffer[MAVLINK_MAX_PAYLOAD_LEN+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
    
    //mavlink组帧
    mavlink_msg_statustext_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &statustext);
    //消息帧格式化
    msgLength = mavlink_msg_to_send_buffer(msgBuffer, &msg); 
    //发送消息帧
	DataSend(msgBuffer, msgLength);  
}

/**********************************************************************************************************
*函 数 名: MavlinkSendNotice
*功能说明: 发送状态文本
*形    参: 文本序号
*返 回 值: 无
**********************************************************************************************************/
void MavlinkSendNotice(uint16_t noticeNum)
{
    MavlinkSendEnable(MAVLINK_MSG_ID_STATUSTEXT);
    statustext.severity = GetMavNoticeValue(noticeNum).severity;
    memset(statustext.text, 0, 50);
    memcpy(statustext.text, GetMavNoticeValue(noticeNum).strings, strlen(GetMavNoticeValue(noticeNum).strings));
}

/**********************************************************************************************************
*函 数 名: MavStatusTextSendCheck
*功能说明: 状态文本发送检查
*形    参: 无
*返 回 值: 状态
**********************************************************************************************************/
bool MavStatusTextSendCheck(void)
{
    static uint32_t i = 0;
    uint8_t flag;
    
    if(statusTextSendFlag[i % MAV_NOTICE_NUM] == 1)
    {
        MavlinkSendNotice(i % MAV_NOTICE_NUM);
        statusTextSendFlag[i % MAV_NOTICE_NUM] = 0;
        flag = true;
    }
    else
    {
        flag = false;
    }
    
    i++;
    
    return flag;
}

/**********************************************************************************************************
*函 数 名: MavlinkSendNoticeEnable
*功能说明: 状态文本发送使能
*形    参: 文本序号
*返 回 值: 无
**********************************************************************************************************/
void MavlinkSendNoticeEnable(uint16_t noticeNum)
{
    statusTextSendFlag[noticeNum] = 1;
}

/**********************************************************************************************************
*函 数 名: MavlinkSendNoticeProgress
*功能说明: 发送进度提示
*形    参: 文本序号
*返 回 值: 无
**********************************************************************************************************/
void MavlinkSendNoticeProgress(uint8_t progress)
{
    switch(progress)
    {
        case 0:
            MavlinkSendNoticeEnable(CAL_PROGRESS_0);
            break;  
        case 1:
            MavlinkSendNoticeEnable(CAL_PROGRESS_10);
            break;          
        case 2:
            MavlinkSendNoticeEnable(CAL_PROGRESS_20);
            break;
        case 3:
            MavlinkSendNoticeEnable(CAL_PROGRESS_30);
            break;         
        case 4:
            MavlinkSendNoticeEnable(CAL_PROGRESS_40);
            break;  
        case 5:
            MavlinkSendNoticeEnable(CAL_PROGRESS_50);
            break;          
        case 6:
            MavlinkSendNoticeEnable(CAL_PROGRESS_60);
            break;   
        case 7:
            MavlinkSendNoticeEnable(CAL_PROGRESS_70);
            break; 
        case 8:
            MavlinkSendNoticeEnable(CAL_PROGRESS_80);
            break;   
        case 9:
            MavlinkSendNoticeEnable(CAL_PROGRESS_90);
            break; 
        default:
            break;
    }
}




