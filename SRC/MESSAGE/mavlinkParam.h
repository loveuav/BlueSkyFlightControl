#ifndef __MAVLINKPARAM_H__
#define __MAVLINKPARAM_H__

#include "mathTool.h"

enum
{
    /*System*/
    SYS_AUTOSTART,    
    SYS_AUTOCONFIG,
    /*MAVLink*/
    MAV_SYS_ID,
    MAV_COMP_ID,
    MAV_PROTO_VER,
    MAV_RADIO_ID,
    MAV_AIRFRAME_TYPE,
    MAV_USEHILGPS,
    MAV_FWDEXTSP,
    MAV_BROADCAST,
    
    MAV_PARAM_NUM
};

float MavParamGetValue(uint16_t num);
void MavParamSetValue(uint16_t num, float value);
int MavParamGetIdByName(char *name);
void MavParamSetDefault(void);
const char* MavParamGetString(uint8_t num);

#endif
