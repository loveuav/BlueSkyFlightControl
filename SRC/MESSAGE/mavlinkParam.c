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
#include "mavlinkSend.h"
#include <string.h>

float mavParam[MAV_PARAM_NUM];

uint8_t mavParamSendFlag[MAV_PARAM_NUM];

//参数字符标识，不能超过16个字符
const char* mavParamStrings[] = 
{
    "SYS_AUTOSTART",
    "SYS_AUTOCONFIG",
    "SYS_PARAM_VER",
    "SYS_SW_VER",
    "MAV_SYS_ID",
    "MAV_COMP_ID",
    "MAV_PROTO_VER",
    "MAV_RADIO_ID",
    "MAV_TYPE",
    "MAV_USEHILGPS",
    "MAV_FWDEXTSP",
    "MAV_BROADCAST",
    "CAL_BOARD_ID",
    "CAL_GYRO0_ID",
    "CAL_GYRO0_XOFF",
    "CAL_GYRO0_YOFF",
    "CAL_GYRO0_ZOFF",
    "CAL_GYRO0_XSCALE",
    "CAL_GYRO0_YSCALE",
    "CAL_GYRO0_ZSCALE",
    "CAL_MAG0_ID",
    "CAL_MAG0_ROT",
    "CAL_MAG0_XOFF",
    "CAL_MAG0_YOFF",
    "CAL_MAG0_ZOFF",
    "CAL_MAG0_XSCALE",
    "CAL_MAG0_YSCALE",
    "CAL_MAG0_ZSCALE",
    "CAL_ACC0_ID",
    "CAL_ACC0_XOFF",
    "CAL_ACC0_YOFF",
    "CAL_ACC0_ZOFF",
    "CAL_ACC0_XSCALE",
    "CAL_ACC0_YSCALE",
    "CAL_ACC0_ZSCALE",
    "CAL_GYRO1_ID",
    "CAL_GYRO1_XOFF",
    "CAL_GYRO1_YOFF",
    "CAL_GYRO1_ZOFF",
    "CAL_GYRO1_XSCALE",
    "CAL_GYRO1_YSCALE",
    "CAL_GYRO1_ZSCALE",
    "CAL_MAG1_ID",
    "CAL_MAG1_ROT",
    "CAL_MAG1_XOFF",
    "CAL_MAG1_YOFF",
    "CAL_MAG1_ZOFF",
    "CAL_MAG1_XSCALE",
    "CAL_MAG1_YSCALE",
    "CAL_MAG1_ZSCALE",
    "CAL_ACC1_ID",
    "CAL_ACC1_XOFF",
    "CAL_ACC1_YOFF",
    "CAL_ACC1_ZOFF",
    "CAL_ACC1_XSCALE",
    "CAL_ACC1_YSCALE",
    "CAL_ACC1_ZSCALE",
    "CAL_GYRO2_ID",
    "CAL_GYRO2_XOFF",
    "CAL_GYRO2_YOFF",
    "CAL_GYRO2_ZOFF",
    "CAL_GYRO2_XSCALE",
    "CAL_GYRO2_YSCALE",
    "CAL_GYRO2_ZSCALE",
    "CAL_MAG2_ID",
    "CAL_MAG2_ROT",
    "CAL_MAG2_XOFF",
    "CAL_MAG2_YOFF",
    "CAL_MAG2_ZOFF",
    "CAL_MAG2_XSCALE",
    "CAL_MAG2_YSCALE",
    "CAL_MAG2_ZSCALE",
    "CAL_ACC2_ID",
    "CAL_ACC2_XOFF",
    "CAL_ACC2_YOFF",
    "CAL_ACC2_ZOFF",
    "CAL_ACC2_XSCALE",
    "CAL_ACC2_YSCALE",
    "CAL_ACC2_ZSCALE",
    "CAL_ACC_PRIME",
    "CAL_GYRO_PRIME",
    "CAL_MAG_PRIME",
    "CAL_MAG_SIDES",
    "CAL_BARO_PRIME",
    "SENS_DPRES_OFF",
    "SENS_DPRES_ANSC",
    "SENS_BARO_QNH",
    "SENS_BOARD_ROT",
    "SENS_FLOW_ROT",
    "SENS_BOARD_X_OFF",
    "SENS_BOARD_Y_OFF",
    "SENS_BOARD_Z_OFF",
    "SENS_EXT_MAG_ROT",
    "SENS_EXT_MAG",
};

/**********************************************************************************************************
*函 数 名: MavParamSendCheck
*功能说明: 参数发送检查
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
bool MavParamSendCheck(void)
{
    static uint32_t i = 0;
    uint8_t flag;
    
    if(mavParamSendFlag[i % MAV_PARAM_NUM] == 1)
    {
        MavlinkCurrentParamSet(i % MAV_PARAM_NUM);
        mavParamSendFlag[i % MAV_PARAM_NUM] = 0;
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
*函 数 名: MavParamSendEnable
*功能说明: 参数发送使能
*形    参: 参数序号
*返 回 值: 无
**********************************************************************************************************/
void MavParamSendEnable(int16_t num)
{
    mavParamSendFlag[num] = 1;
}

/**********************************************************************************************************
*函 数 名: MavParamSendEnableAll
*功能说明: 发送全部参数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MavParamSendEnableAll(void)
{
    for(uint8_t i=0; i<MAV_PARAM_NUM; i++)
        mavParamSendFlag[i] = 1;
}

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
    mavParam[SYS_AUTOSTART] = 4001;
    mavParam[SYS_AUTOCONFIG] = 0; 
    mavParam[SYS_PARAM_VER] = 0.2;
    mavParam[SYS_SW_VER] = 0.3;
    
    mavParam[MAV_SYS_ID] = 1; 
    mavParam[MAV_COMP_ID] = 1; 
    mavParam[MAV_PROTO_VER] = 1; 
    mavParam[MAV_RADIO_ID] = 0; 
    mavParam[MAV_AIRFRAME_TYPE] = 2; 
    mavParam[MAV_USEHILGPS] = 0; 
    mavParam[MAV_FWDEXTSP] = 0;   
    mavParam[MAV_BROADCAST] = 0;  
    
    mavParam[CAL_BOARD_ID] = 0; 
    mavParam[CAL_GYRO0_ID] = 125; 
    mavParam[CAL_GYRO0_XOFF] = 0; 
    mavParam[CAL_GYRO0_YOFF] = 0; 
    mavParam[CAL_GYRO0_ZOFF] = 0; 
    mavParam[CAL_GYRO0_XSCALE] = 1; 
    mavParam[CAL_GYRO0_YSCALE] = 1; 
    mavParam[CAL_GYRO0_ZSCALE] = 1; 
    mavParam[CAL_MAG0_ID] = 130; 
    mavParam[CAL_MAG0_ROT] = 16; 
    mavParam[CAL_MAG0_XOFF] = 0; 
    mavParam[CAL_MAG0_YOFF] = 0; 
    mavParam[CAL_MAG0_ZOFF] = 0; 
    mavParam[CAL_MAG0_XSCALE] = 1; 
    mavParam[CAL_MAG0_YSCALE] = 1; 
    mavParam[CAL_MAG0_ZSCALE] = 1; 
    mavParam[CAL_ACC0_ID] = 120; 
    mavParam[CAL_ACC0_XOFF] = 0; 
    mavParam[CAL_ACC0_YOFF] = 0; 
    mavParam[CAL_ACC0_ZOFF] = 0; 
    mavParam[CAL_ACC0_XSCALE] = 1; 
    mavParam[CAL_ACC0_YSCALE] = 1; 
    mavParam[CAL_ACC0_ZSCALE] = 1; 
    
    mavParam[CAL_GYRO1_ID] = 0; 
    mavParam[CAL_GYRO1_XOFF] = 0; 
    mavParam[CAL_GYRO1_YOFF] = 0; 
    mavParam[CAL_GYRO1_ZOFF] = 0; 
    mavParam[CAL_GYRO1_XSCALE] = 1; 
    mavParam[CAL_GYRO1_YSCALE] = 1; 
    mavParam[CAL_GYRO1_ZSCALE] = 1; 
    mavParam[CAL_MAG1_ID] = 0; 
    mavParam[CAL_MAG1_ROT] = -1; 
    mavParam[CAL_MAG1_XOFF] = 0; 
    mavParam[CAL_MAG1_YOFF] = 0; 
    mavParam[CAL_MAG1_ZOFF] = 0; 
    mavParam[CAL_MAG1_XSCALE] = 1; 
    mavParam[CAL_MAG1_YSCALE] = 1; 
    mavParam[CAL_MAG1_ZSCALE] = 1; 
    mavParam[CAL_ACC1_ID] = 0; 
    mavParam[CAL_ACC1_XOFF] = 0; 
    mavParam[CAL_ACC1_YOFF] = 0; 
    mavParam[CAL_ACC1_ZOFF] = 0; 
    mavParam[CAL_ACC1_XSCALE] = 1; 
    mavParam[CAL_ACC1_YSCALE] = 1; 
    mavParam[CAL_ACC1_ZSCALE] = 1;   

    mavParam[CAL_GYRO2_ID] = 0; 
    mavParam[CAL_GYRO2_XOFF] = 0; 
    mavParam[CAL_GYRO2_YOFF] = 0; 
    mavParam[CAL_GYRO2_ZOFF] = 0; 
    mavParam[CAL_GYRO2_XSCALE] = 1; 
    mavParam[CAL_GYRO2_YSCALE] = 1; 
    mavParam[CAL_GYRO2_ZSCALE] = 1; 
    mavParam[CAL_MAG2_ID] = 0; 
    mavParam[CAL_MAG2_ROT] = -1; 
    mavParam[CAL_MAG2_XOFF] = 0; 
    mavParam[CAL_MAG2_YOFF] = 0; 
    mavParam[CAL_MAG2_ZOFF] = 0; 
    mavParam[CAL_MAG2_XSCALE] = 1; 
    mavParam[CAL_MAG2_YSCALE] = 1; 
    mavParam[CAL_MAG2_ZSCALE] = 1; 
    mavParam[CAL_ACC2_ID] = 0; 
    mavParam[CAL_ACC2_XOFF] = 0; 
    mavParam[CAL_ACC2_YOFF] = 0; 
    mavParam[CAL_ACC2_ZOFF] = 0; 
    mavParam[CAL_ACC2_XSCALE] = 1; 
    mavParam[CAL_ACC2_YSCALE] = 1; 
    mavParam[CAL_ACC2_ZSCALE] = 1;    
    
    mavParam[CAL_ACC_PRIME] = 120;
    mavParam[CAL_GYRO_PRIME] = 125;
    mavParam[CAL_MAG_PRIME] = 130;
    mavParam[CAL_MAG_SIDES] = 63;
    mavParam[CAL_BARO_PRIME] = 0;
    mavParam[SENS_DPRES_OFF] = 0;
    mavParam[SENS_DPRES_ANSC] = 0;
    mavParam[SENS_BARO_QNH] = 1013.25;
    mavParam[SENS_BOARD_ROT] = 0;
    mavParam[SENS_FLOW_ROT] = 0;
    mavParam[SENS_BOARD_X_OFF] = 0;
    mavParam[SENS_BOARD_Y_OFF] = 0;
    mavParam[SENS_BOARD_Z_OFF] = 0;
    mavParam[SENS_EXT_MAG_ROT] = 0;
    mavParam[SENS_EXT_MAG] = 0;
}




