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
#include "mavlinkSend.h"
#include "mavlinkParam.h"
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

static void MavlinkDecodeCommand(mavlink_command_long_t command);
    
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
    
    //对接收到的字节数据进行帧解析，接收完一帧时再继续对帧数据进行解析   
    if(mavlink_parse_char(0, data, &msg, &status) == false)
        return;
    
    switch(msg.msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT:              //心跳包
            break;
        
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:     //请求发送飞控单个参数
            if(MAVLINK_SYSTEM_ID == mavlink_msg_param_request_read_get_target_system(&msg)) 
            {
                static uint16_t paramIndex;
                static mavlink_param_request_read_t request_read;
                //帧解析
                mavlink_msg_param_request_read_decode(&msg, &request_read);
                //根据参数标识符获取参数ID
                paramIndex = MavParamGetIdByName((char*)request_read.param_id);
                if(paramIndex< MAV_PARAM_NUM)
                {
                    //发送单个参数
                    MavParamSendEnable(paramIndex);
                }
            }
            break;
        
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:     //请求发送飞控参数列表
            if(MAVLINK_SYSTEM_ID == mavlink_msg_param_request_list_get_target_system(&msg))
            {
                //发送全部参数
                MavParamSendEnableAll();
            }
            break;
            
        case MAVLINK_MSG_ID_PARAM_SET:              //设置飞控参数           
            if(MAVLINK_SYSTEM_ID == mavlink_msg_param_set_get_target_system(&msg))
            {
                int paramIndex = -1;
                static mavlink_param_set_t param_set;
                //帧解析
                mavlink_msg_param_set_decode(&msg, &param_set);
                //根据参数标识符获取参数ID
                paramIndex = MavParamGetIdByName((char*)param_set.param_id);

                if (paramIndex >= 0 && paramIndex < MAV_PARAM_NUM) 
                {
                    MavParamSetValue(paramIndex, param_set.param_value);  
                    //返回参数给地面站
                    MavParamSendEnable(paramIndex);
                }
            }
            break;
            
        case MAVLINK_MSG_ID_COMMAND_LONG:           //命令
            if (MAVLINK_SYSTEM_ID == mavlink_msg_command_long_get_target_system(&msg))
            {
                static mavlink_command_long_t command;
                //帧解析
                mavlink_msg_command_long_decode(&msg, &command); 
                //命令解析
                MavlinkDecodeCommand(command);           
            }
            break;
            
        default:
            break;
    }
}

/**********************************************************************************************************
*函 数 名: MavlinkDecodeCommand
*功能说明: 命令解析
*形    参: 接收数据
*返 回 值: 无
**********************************************************************************************************/
static void MavlinkDecodeCommand(mavlink_command_long_t command)
{
    switch(command.command)
    {
        /*传感器校准*/
        case MAV_CMD_PREFLIGHT_CALIBRATION:
            if(GetArmedStatus() == DISARMED)
            { 
                if(command.param1 == 1)          //校准陀螺仪
                {
                    MavlinkSendNoticeEnable(CAL_START_GYRO);
                    GyroCalibrateEnable();
                }
                else if(command.param2 == 1)     //校准罗盘
                {
                    MavlinkSendNoticeEnable(CAL_START_MAG);
                    MagCalibrateEnable();
                }
                else if(command.param5 == 1)     //校准加速度计
                {
                    MavlinkSendNoticeEnable(CAL_START_ACC);
                    AccCalibrateEnable();
                }
                else if(command.param5 == 2)     //校准水平
                {
                    MavlinkSendNoticeEnable(CAL_START_LEVEL);
                    LevelCalibrateEnable();
                }
            
                MavlinkSetCommandAck(MAV_CMD_PREFLIGHT_CALIBRATION, MAV_CMD_ACK_OK);
            }
            else
            {
                MavlinkSetCommandAck(MAV_CMD_PREFLIGHT_CALIBRATION, MAV_CMD_ACK_ERR_FAIL);
            }
            
            MavlinkSendEnable(MAVLINK_MSG_ID_COMMAND_ACK);
            break;
            
        /*请求协议版本，如果不支持mavlink2则不回应*/
        case MAV_CMD_REQUEST_PROTOCOL_VERSION:
            break;
        
        case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
            MavlinkSetCommandAck(MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, MAV_CMD_ACK_OK);
            MavlinkSendEnable(MAVLINK_MSG_ID_COMMAND_ACK);            
            break;
        
        default:
            break;
    }
}
