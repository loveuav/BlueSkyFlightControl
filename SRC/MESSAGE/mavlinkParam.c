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
    "TRIM_ROLL",
    "TRIM_PITCH",
    "TRIM_YAW",
    "RC1_MIN",
    "RC1_TRIM",
    "RC1_MAX",
    "RC1_REV",
    "RC1_DZ",
    "RC2_MIN",
    "RC2_TRIM",
    "RC2_MAX",
    "RC2_REV",
    "RC2_DZ",
    "RC3_MIN",
    "RC3_TRIM",
    "RC3_MAX",
    "RC3_REV",
    "RC3_DZ",
    "RC4_MIN",
    "RC4_TRIM",
    "RC4_MAX",
    "RC4_REV",
    "RC4_DZ",    
    "RC5_MIN",
    "RC5_TRIM",
    "RC5_MAX",
    "RC5_REV",
    "RC5_DZ",
    "RC6_MIN",
    "RC6_TRIM",
    "RC6_MAX",
    "RC6_REV",
    "RC6_DZ",
    "RC7_MIN",
    "RC7_TRIM",
    "RC7_MAX",
    "RC7_REV",
    "RC7_DZ",
    "RC8_MIN",
    "RC8_TRIM",
    "RC8_MAX",
    "RC8_REV",
    "RC8_DZ",
    "RC9_MIN",
    "RC9_TRIM",
    "RC9_MAX",
    "RC9_REV",
    "RC9_DZ",
    "RC10_MIN",
    "RC10_TRIM",
    "RC10_MAX",
    "RC10_REV",
    "RC10_DZ",
    "RC11_MIN",
    "RC11_TRIM",
    "RC11_MAX",
    "RC11_REV",
    "RC11_DZ",
    "RC12_MIN",
    "RC12_TRIM",
    "RC12_MAX",
    "RC12_REV",
    "RC12_DZ",
    "RC13_MIN",
    "RC13_TRIM",
    "RC13_MAX",
    "RC13_REV",
    "RC13_DZ",
    "RC14_MIN",
    "RC14_TRIM",
    "RC14_MAX",
    "RC14_REV",
    "RC14_DZ",
    "RC15_MIN",
    "RC15_TRIM",
    "RC15_MAX",
    "RC15_REV",
    "RC15_DZ",
    "RC16_MIN",
    "RC16_TRIM",
    "RC16_MAX",
    "RC16_REV",
    "RC16_DZ",
    "RC17_MIN",
    "RC17_TRIM",
    "RC17_MAX",
    "RC17_REV",
    "RC17_DZ",
    "RC18_MIN",
    "RC18_TRIM",
    "RC18_MAX",
    "RC18_REV",
    "RC18_DZ",
    "RC_MAP_ROLL",
    "RC_MAP_PITCH",
    "RC_MAP_YAW",
    "RC_MAP_THROTTLE",
    "RC_MAP_FAILSAFE",
    "RC_MAP_MODE_SW",
    "RC_MAP_RETURN_SW",
    "RC_MAP_FLAPS",
    "RC_MAP_RATT_SW",
    "RC_MAP_POSCTL_SW",
    "RC_MAP_LOITER_SW",
    "RC_MAP_ACRO_SW",
    "RC_MAP_OFFB_SW",
    "RC_MAP_KILL_SW",
    "RC_MAP_AUX1",
    "RC_MAP_AUX2",
    "RC_MAP_AUX3",
    "RC_MAP_AUX4",
    "RC_MAP_AUX5",
    "RC_MAP_PARAM1",
    "RC_MAP_PARAM2",
    "RC_MAP_PARAM3",
    "RC_MAP_FLTMODE",
    "RC_FAILS_THR",
    "RC_ASSIST_TH",
    "RC_AUTO_TH",
    "RC_RATT_TH",
    "RC_POSCTL_TH",
    "RC_RETURN_TH",
    "RC_LOITER_TH",
    "RC_ACRO_TH",
    "RC_OFFB_TH",
    "RC_KILLSWITCH_TH",
    "RC_CHAN_CNT",
    "RC_TH_USER",
    "COM_DL_LOSS_T",
    "COM_DL_REG_T",
    "COM_EF_THROT",
    "COM_EF_C2T",
    "COM_EF_TIME",
    "COM_RC_LOSS_T",
    "COM_HOME_H_T",
    "COM_HOME_V_T",
    "COM_AUTOS_PAR",
    "COM_RC_IN_MODE",
    "COM_RC_ARM_HYST",
    "COM_DISARM_LAND",
    "COM_LOW_BAT_ACT",
    "BAT_H_CURR",
    "BAT_V_CURR",
    "BAT_C_SAFE",
    "BAT_C_SAFE_UB",
    "BAT_V_SCALE_IO",
    "BAT_CNT_V_VOLT",
    "BAT_CNT_V_CURR",
    "BAT_V_OFFS_CURR",
    "BAT_V_DIV",
    "BAT_A_PER_V",
    "BAT_V_EMPTY",
    "BAT_V_CHARGED",
    "BAT_LOW_THR",
    "BAT_CRIT_THR",
    "BAT_V_LOAD_DROP",
    "BAT_N_CELLS",
    "BAT_CAPACITY",
    "NAV_DLL_ACT",
    "NAV_RCL_ACT",
    "RTL_RETURN_ALT",
    "RTL_DESCEND_ALT",
    "RTL_LAND_DELAY",
    "RTL_MIN_DIST",
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
    for(uint16_t i=0; i<MAV_PARAM_NUM; i++)
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
    mavParam[CAL_MAG_SIDES] = 34;
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

    mavParam[TRIM_ROLL] = 0;
    mavParam[TRIM_PITCH] = 0;
    mavParam[TRIM_YAW] = 0;
    mavParam[RC1_MIN] = 1000;
    mavParam[RC1_TRIM] = 1500;
    mavParam[RC1_MAX] = 2000;
    mavParam[RC1_REV] = 1;
    mavParam[RC1_DZ] = 50;
    mavParam[RC2_MIN] = 1000;
    mavParam[RC2_TRIM] = 1500;
    mavParam[RC2_MAX] = 2000;
    mavParam[RC2_REV] = 1;
    mavParam[RC2_DZ] = 50;
    mavParam[RC3_MIN] = 1000;
    mavParam[RC3_TRIM] = 1500;
    mavParam[RC3_MAX] = 2000;
    mavParam[RC3_REV] = 1;
    mavParam[RC3_DZ] = 50;
    mavParam[RC4_MIN] = 1000;
    mavParam[RC4_TRIM] = 1500;
    mavParam[RC4_MAX] = 2000;
    mavParam[RC4_REV] = 1;
    mavParam[RC4_DZ] = 50;   
    mavParam[RC5_MIN] = 1000;
    mavParam[RC5_TRIM] = 1500;
    mavParam[RC5_MAX] = 2000;
    mavParam[RC5_REV] = 1;
    mavParam[RC5_DZ] = 50;
    mavParam[RC6_MIN] = 1000;
    mavParam[RC6_TRIM] = 1500;
    mavParam[RC6_MAX] = 2000;
    mavParam[RC6_REV] = 1;
    mavParam[RC6_DZ] = 50;
    mavParam[RC7_MIN] = 1000;
    mavParam[RC7_TRIM] = 1500;
    mavParam[RC7_MAX] = 2000;
    mavParam[RC7_REV] = 1;
    mavParam[RC7_DZ] = 50;
    mavParam[RC8_MIN] = 1000;
    mavParam[RC8_TRIM] = 1500;
    mavParam[RC8_MAX] = 2000;
    mavParam[RC8_REV] = 1;
    mavParam[RC8_DZ] = 50;
    mavParam[RC9_MIN] = 1000;
    mavParam[RC9_TRIM] = 1500;
    mavParam[RC9_MAX] = 2000;
    mavParam[RC9_REV] = 1;
    mavParam[RC9_DZ] = 50;
    mavParam[RC10_MIN] = 1000;
    mavParam[RC10_TRIM] = 1500;
    mavParam[RC10_MAX] = 2000;
    mavParam[RC10_REV] = 1;
    mavParam[RC10_DZ] = 50;
    mavParam[RC11_MIN] = 1000;
    mavParam[RC11_TRIM] = 1500;
    mavParam[RC11_MAX] = 2000;
    mavParam[RC11_REV] = 1;
    mavParam[RC11_DZ] = 50;
    mavParam[RC12_MIN] = 1000;
    mavParam[RC12_TRIM] = 1500;
    mavParam[RC12_MAX] = 2000;
    mavParam[RC12_REV] = 1;
    mavParam[RC12_DZ] = 50;
    mavParam[RC13_MIN] = 1000;
    mavParam[RC13_TRIM] = 1500;
    mavParam[RC13_MAX] = 2000;
    mavParam[RC13_REV] = 1;
    mavParam[RC13_DZ] = 50;
    mavParam[RC14_MIN] = 1000;
    mavParam[RC14_TRIM] = 1500;
    mavParam[RC14_MAX] = 2000;
    mavParam[RC14_REV] = 1;
    mavParam[RC14_DZ] = 50;
    mavParam[RC15_MIN] = 1000;
    mavParam[RC15_TRIM] = 1500;
    mavParam[RC15_MAX] = 2000;
    mavParam[RC15_REV] = 1;
    mavParam[RC15_DZ] = 50;
    mavParam[RC16_MIN] = 1000;
    mavParam[RC16_TRIM] = 1500;
    mavParam[RC16_MAX] = 2000;
    mavParam[RC16_REV] = 1;
    mavParam[RC16_DZ] = 50;
    mavParam[RC17_MIN] = 1000;
    mavParam[RC17_TRIM] = 1500;
    mavParam[RC17_MAX] = 2000;
    mavParam[RC17_REV] = 1;
    mavParam[RC17_DZ] = 50;
    mavParam[RC18_MIN] = 1000;
    mavParam[RC18_TRIM] = 1500;
    mavParam[RC18_MAX] = 2000;
    mavParam[RC18_REV] = 1;
    mavParam[RC18_DZ] = 0;
    
    mavParam[RC_MAP_ROLL] = 1;
    mavParam[RC_MAP_PITCH] = 2;
    mavParam[RC_MAP_YAW] = 4;
    mavParam[RC_MAP_THROTTLE] = 3;
    mavParam[RC_MAP_FAILSAFE] = 0;
    mavParam[RC_MAP_MODE_SW] = 5;
    mavParam[RC_MAP_RETURN_SW] = 7;
    mavParam[RC_MAP_FLAPS] = 0;
    mavParam[RC_MAP_RATT_SW] = 0;
    mavParam[RC_MAP_POSCTL_SW] = 0;
    mavParam[RC_MAP_LOITER_SW] = 0;
    mavParam[RC_MAP_ACRO_SW] = 0;
    mavParam[RC_MAP_OFFB_SW] = 0;
    mavParam[RC_MAP_KILL_SW] = 9;
    mavParam[RC_MAP_AUX1] = 5;
    mavParam[RC_MAP_AUX2] = 6;
    mavParam[RC_MAP_AUX3] = 7;
    mavParam[RC_MAP_AUX4] = 8;
    mavParam[RC_MAP_AUX5] = 0;
    mavParam[RC_MAP_PARAM1] = 0;
    mavParam[RC_MAP_PARAM2] = 0;
    mavParam[RC_MAP_PARAM3] = 0;
    mavParam[RC_MAP_FLTMODE] = 0;
    mavParam[RC_FAILS_THR] = 0;
    mavParam[RC_ASSIST_TH] = 0.25;
    mavParam[RC_AUTO_TH] = 0.75;
    mavParam[RC_RATT_TH] = 0.5;
    mavParam[RC_POSCTL_TH] = 0.5;
    mavParam[RC_RETURN_TH] = 0.5;
    mavParam[RC_LOITER_TH] = 0.5;
    mavParam[RC_ACRO_TH] = 0.5;
    mavParam[RC_OFFB_TH] = 0.5;
    mavParam[RC_KILLSWITCH_TH] = 0.7;
    mavParam[RC_CHAN_CNT] = 0;
    mavParam[RC_TH_USER] = 1;
    
    mavParam[COM_DL_LOSS_T] = 10;
    mavParam[COM_DL_REG_T] = 0;
    mavParam[COM_EF_THROT] = 0.5;
    mavParam[COM_EF_C2T] = 5;
    mavParam[COM_EF_TIME] = 10;
    mavParam[COM_RC_LOSS_T] = 0.5;
    mavParam[COM_HOME_H_T] = 5;
    mavParam[COM_HOME_V_T] = 10;
    mavParam[COM_AUTOS_PAR] = 1;
    mavParam[COM_RC_IN_MODE] = 0;
    mavParam[COM_RC_ARM_HYST] = 1000;
    mavParam[COM_DISARM_LAND] = 2;
    mavParam[COM_LOW_BAT_ACT] = 1;
    
    mavParam[BAT_H_CURR] = 12;
    mavParam[BAT_V_CURR] = 10;
    mavParam[BAT_C_SAFE] = 700;
    mavParam[BAT_C_SAFE_UB] = 1500;
    mavParam[BAT_V_SCALE_IO] = 100;
    mavParam[BAT_CNT_V_VOLT] = -1;
    mavParam[BAT_CNT_V_CURR] = -1;
    mavParam[BAT_V_OFFS_CURR] = 0;
    mavParam[BAT_V_DIV] = 10.19699955;
    mavParam[BAT_A_PER_V] = -1;
    mavParam[BAT_V_EMPTY] = 3.2;
    mavParam[BAT_V_CHARGED] = 4.2;
    mavParam[BAT_LOW_THR] = 0.2;
    mavParam[BAT_CRIT_THR] = 0.1;
    mavParam[BAT_V_LOAD_DROP] = 0.3;
    mavParam[BAT_N_CELLS] = 4;
    mavParam[BAT_CAPACITY] = -1;  

    mavParam[NAV_DLL_ACT] = 0;
    mavParam[NAV_RCL_ACT] = 2;
    
    mavParam[RTL_RETURN_ALT] = 15;
    mavParam[RTL_DESCEND_ALT] = 30;
    mavParam[RTL_LAND_DELAY] = 3;
    mavParam[RTL_MIN_DIST] = 0.5;
}




