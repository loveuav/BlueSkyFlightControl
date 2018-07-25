/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     mavlinkParam.c
 * @说明     为兼容MP和QGC地面站所需要发送的参数
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.07 
**********************************************************************************************************/
#include "mavlinkParam.h"
#include <string.h>

float mavParam[MAV_PARAM_NUM];

//参数字符标识，不能超过16个字符
const char* mavParamStrings[] = 
{
    "SYS_AUTOSTART",
    "SYS_AUTOCONFIG",
    "MAV_SYS_ID",
    "MAV_COMP_ID",
    "MAV_PROTO_VER",
    "MAV_RADIO_ID",
    "MAV_TYPE",
    "MAV_USEHILGPS",
    "MAV_FWDEXTSP",
    "MAV_BROADCAST"
};

/**********************************************************************************************************
*函 数 名: MavParamGetValue
*功能说明: 获取参数值
*形    参: 参数序号
*返 回 值: 参数值
**********************************************************************************************************/
float MavParamGetValue(uint16_t num)
{
    return mavParam[num];
}

/**********************************************************************************************************
*函 数 名: MavParamSetValue
*功能说明: 设置参数值
*形    参: 参数序号 参数值
*返 回 值: 无
**********************************************************************************************************/
void MavParamSetValue(uint16_t num, float value)
{
    mavParam[num] = value;
}

/**********************************************************************************************************
*函 数 名: MavParamGetString
*功能说明: 获取参数标识符
*形    参: 参数序号
*返 回 值: 字符指针
**********************************************************************************************************/
const char* MavParamGetString(uint8_t num)
{
	return mavParamStrings[num];
}

/**********************************************************************************************************
*函 数 名: MavParamGetIdByName
*功能说明: 根据参数标识符获取参数序号
*形    参: 参数序号
*返 回 值: 字符指针
**********************************************************************************************************/
int MavParamGetIdByName(char *name)
{
    int i;

    for (i=0; i<MAV_PARAM_NUM; i++)
    {
        if (!strncmp(name, mavParamStrings[i], 16))
            break;        
    }

    return i;
}
/**********************************************************************************************************
*函 数 名: MavParamSetDefault
*功能说明: 设置默认参数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MavParamSetDefault(void)
{
    mavParam[SYS_AUTOSTART] = 10020;
    mavParam[SYS_AUTOCONFIG] = 0;   
    mavParam[MAV_SYS_ID] = 1; 
    mavParam[MAV_COMP_ID] = 1; 
    mavParam[MAV_PROTO_VER] = 1; 
    mavParam[MAV_RADIO_ID] = 0; 
    mavParam[MAV_AIRFRAME_TYPE] = 2; 
    mavParam[MAV_USEHILGPS] = 0; 
    mavParam[MAV_FWDEXTSP] = 0;   
    mavParam[MAV_BROADCAST] = 0;     
}




