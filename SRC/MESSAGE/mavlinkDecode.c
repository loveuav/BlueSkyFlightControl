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
#include "mavlinkNotice.h"
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
#include "waypointControl.h"

static void MavlinkDecodeCommand(mavlink_command_long_t command);
static void MavlinkSetFlightMode(mavlink_set_mode_t set_mode);  

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
        /*心跳包*/
        case MAVLINK_MSG_ID_HEARTBEAT:      
            break;
        
        /*设置飞行模式*/
        case MAVLINK_MSG_ID_SET_MODE:
        {
            static mavlink_set_mode_t set_mode;
            mavlink_msg_set_mode_decode(&msg, &set_mode);
            MavlinkSetFlightMode(set_mode);  
            break;
        }
                
        /*请求发送飞控单个参数*/
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:     
            if(MAVLINK_SYSTEM_ID == mavlink_msg_param_request_read_get_target_system(&msg)) 
            {
                static uint16_t paramIndex;
                static mavlink_param_request_read_t request_read;
                //帧解析
                mavlink_msg_param_request_read_decode(&msg, &request_read);
                //根据参数标识符获取参数ID
                paramIndex = MavParamGetIdByName((char*)request_read.param_id);
                if(paramIndex < MAV_PARAM_NUM)
                {
                    //发送单个参数
                    MavParamSendEnable(paramIndex);
                }
            }
            break;
            
        /*请求发送飞控参数列表*/
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:     
            if(MAVLINK_SYSTEM_ID == mavlink_msg_param_request_list_get_target_system(&msg))
            {
                //发送全部参数
                MavParamSendEnableAll();
            }
            break;
            
        /*设置飞控参数*/   
        case MAVLINK_MSG_ID_PARAM_SET:                        
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
        
        /*命令*/            
        case MAVLINK_MSG_ID_COMMAND_LONG:          
            if (MAVLINK_SYSTEM_ID == mavlink_msg_command_long_get_target_system(&msg))
            {
                static mavlink_command_long_t command;
                //帧解析
                mavlink_msg_command_long_decode(&msg, &command); 
                //命令解析
                MavlinkDecodeCommand(command);           
            }
            break;
            
        /*航点数量*/    
        case MAVLINK_MSG_ID_MISSION_COUNT:         
            SetWaypointCount(mavlink_msg_mission_count_get_count(&msg));
            SetWaypointRecvCount(0);
            //开始请求航点信息
            MavlinkSendEnable(MAVLINK_MSG_ID_MISSION_REQUEST);
            break;
            
        /*航点信息*/
        case MAVLINK_MSG_ID_MISSION_ITEM:           
        {
            static mavlink_mission_item_t item;
            //帧解析
            mavlink_msg_mission_item_decode(&msg, &item);
            //设置航点信息
            SetWaypointItem(item.seq, item);
            //请求下一个航点信息
            if(item.seq < GetWaypointCount() - 1)
                MavlinkSendEnable(MAVLINK_MSG_ID_MISSION_REQUEST);
            else
            {
                //航点接收完毕，发送应答
                MavlinkSendEnable(MAVLINK_MSG_ID_MISSION_ACK);
            }
            break;
        }
        
        /*请求读取飞控航点信息*/
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:   
            //发送航点数量
            MavlinkSendEnable(MAVLINK_MSG_ID_MISSION_COUNT);
            break;
        
        /*发送航点信息*/
        case MAVLINK_MSG_ID_MISSION_REQUEST:
            SetWaypointSendCount(mavlink_msg_mission_request_get_seq(&msg));
            MavlinkSendEnable(MAVLINK_MSG_ID_MISSION_ITEM);
            break;
        
        /*航点任务响应*/
        case MAVLINK_MSG_ID_MISSION_ACK:
            break;
        
        /*清除所有航点信息*/
        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:      
            ClearAllWaypointItem();
            MavlinkSendEnable(MAVLINK_MSG_ID_MISSION_ACK);
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
        /*返航*/
        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
            MavlinkSetCommandAck(MAV_CMD_NAV_RETURN_TO_LAUNCH, MAV_CMD_ACK_OK);
            MavlinkSendEnable(MAVLINK_MSG_ID_COMMAND_ACK);  
            break;
        
        /*降落*/
        case MAV_CMD_NAV_LAND:
            MavlinkSetCommandAck(MAV_CMD_NAV_LAND, MAV_CMD_ACK_OK);
            MavlinkSendEnable(MAVLINK_MSG_ID_COMMAND_ACK);  
            break;
        
        /*起飞*/
        case MAV_CMD_NAV_TAKEOFF:
            MavlinkSetCommandAck(MAV_CMD_NAV_TAKEOFF, MAV_CMD_ACK_OK | MAV_CMD_ACK_ENUM_END);
            MavlinkSendEnable(MAVLINK_MSG_ID_COMMAND_ACK);  
            break;        
        
        /*开始航点任务*/
        case MAV_CMD_MISSION_START:
            MavlinkSetCommandAck(MAV_CMD_MISSION_START, MAV_CMD_ACK_OK);
            MavlinkSendEnable(MAVLINK_MSG_ID_COMMAND_ACK);  
            break;   
        
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
            
                MavlinkSetCommandAck(MAV_CMD_PREFLIGHT_CALIBRATION, MAV_CMD_ACK_OK | MAV_CMD_ACK_ENUM_END);
            }
            else
            {
                MavlinkSetCommandAck(MAV_CMD_PREFLIGHT_CALIBRATION, MAV_CMD_ACK_ERR_FAIL | MAV_CMD_ACK_ENUM_END);
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
        
        /*电机解锁*/
        case MAV_CMD_COMPONENT_ARM_DISARM:
            if(command.param1 == 1)  
            {                
                if(SetArmedStatus(ARMED))   //解锁
                    MavlinkSetCommandAck(MAV_CMD_COMPONENT_ARM_DISARM, MAV_CMD_ACK_OK | MAV_CMD_ACK_ENUM_END);
                else
                    MavlinkSetCommandAck(MAV_CMD_COMPONENT_ARM_DISARM, MAV_CMD_ACK_ERR_FAIL);
            }
            else if(command.param1 == 0)
            {
                SetArmedStatus(DISARMED);   //上锁
                MavlinkSetCommandAck(MAV_CMD_COMPONENT_ARM_DISARM, MAV_CMD_ACK_OK | MAV_CMD_ACK_ENUM_END);
            }

            MavlinkSendEnable(MAVLINK_MSG_ID_COMMAND_ACK);    
            break;
        
        default:
            break;
    }
}

/**********************************************************************************************************
*函 数 名: MavlinkSetFlightMode
*功能说明: 设置飞行模式
*形    参: 接收数据
*返 回 值: 无
**********************************************************************************************************/
static void MavlinkSetFlightMode(mavlink_set_mode_t set_mode)
{
    switch(set_mode.custom_mode)
    {
        case 0x00010000:
            SetFlightMode(MANUAL);
            break;

        case 0x00020000:
            SetFlightMode(SEMIAUTO);
            break;

        case 0x00030000:
            SetFlightMode(AUTO);
            break;
        
        case 0x04040000:
            SetFlightMode(AUTOPILOT);
            break;            

        case 0x05040000:
            SetFlightMode(RETURNTOHOME);
            break;       
        
        default:
            break;
    }        
}





