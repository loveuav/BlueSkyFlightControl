#ifndef _WAYPOINTCONTROL_H_
#define _WAYPOINTCONTROL_H_

#include "mathTool.h"
#include "common/mavlink.h"

void WaypointControl(void);

uint16_t GetWaypointCount(void);
void SetWaypointCount(uint16_t count);
uint16_t GetWaypointRecvCount(void);
void SetWaypointRecvCount(uint16_t count);
uint16_t GetWaypointSendCount(void);
void SetWaypointSendCount(uint16_t count);

void SetWaypointItem(uint16_t count, mavlink_mission_item_t item);
void ClearAllWaypointItem(void);
mavlink_mission_item_t GetWaypointItem(uint16_t count);

#endif



