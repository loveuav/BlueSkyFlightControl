/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     TaskConfig.h
 * @说明     RTOS的任务配置文件，任务调度模式为抢占式，共有15个优先级（最多可设置为32）
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05
**********************************************************************************************************/
#ifndef __TASKCONFIG_H__
#define __TASKCONFIG_H__

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "board.h"
#include "mathTool.h"

#include "module_task.h"
#include "sensor_task.h"
#include "navigation_task.h"
#include "control_task.h"
#include "message_task.h"
#include "log_task.h"

//任务堆栈大小
#define IMU_SENSOR_READ_TASK_STACK            256
#define SENSOR_UPDATE_TASK_STACK              256
#define IMU_DATA_PRETREAT_TASK_STACK          256
#define OTHER_SENSOR_TASK_STACK               256
#define NAVIGATION_TASK_STACK                 512
#define FLIGHT_STATUS_TASK_STACK              256
#define FLIGHTCONTROL_TASK_STACK              256
#define MESSAGE_TASK_STACK                    512
#define LOG_TASK_STACK                        1024

//任务优先级
#define IMU_SENSOR_READ_TASK_PRIORITY         13
#define IMU_DATA_PRETREAT_TASK_PRIORITY       12
#define FLIGHTCONTROL_TASK_PRIORITY           10
#define NAVIGATION_TASK_PRIORITY              11
#define SENSOR_UPDATE_TASK_PRIORITY           8
#define OTHER_SENSOR_TASK_PRIORITY            7
#define MESSAGE_TASK_PRIORITY                 6
#define FLIGHT_STATUS_TASK_PRIORITY           5
#define LOG_TASK_PRIORITY                     3

enum {
    GYRO_SENSOR_READ,
    ACC_SENSOR_READ,
    TEMP_SENSOR_READ,
    GYRO_DATA_PRETREAT,
    ACC_DATA_PRETREAT,
    GYRO_FOR_CONTROL,
    QUEUE_NUM
};

extern QueueHandle_t messageQueue[QUEUE_NUM];

#endif
