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
    else if(msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ)
    {
        i++;
    }
    else if(msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST)        
    {
        //请求发送飞控参数列表
        if(MAVLINK_SYSTEM_ID == mavlink_msg_param_request_list_get_target_system(&msg))
        {
            MavlinkCurrentParamSet(0, 1);
            MavlinkSendEnable(MAVLINK_MSG_ID_PARAM_VALUE);
        }
    }
    else if(msg.msgid == MAVLINK_MSG_ID_PARAM_SET)                 
    {
        //设置飞控参数
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
                MavlinkCurrentParamSet(paramIndex, 0);
                MavlinkSendEnable(MAVLINK_MSG_ID_PARAM_VALUE);
            }
        }
    }
}


