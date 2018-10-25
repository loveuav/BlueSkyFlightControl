/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     control_task.c
 * @说明     飞行控制相关任务
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05
**********************************************************************************************************/
#include "TaskConfig.h"

#include "flightControl.h"
#include "userControl.h"
#include "missionControl.h"
#include "safeControl.h"
#include "rc.h"

xTaskHandle flightControlTask;

/**********************************************************************************************************
*函 数 名: vFlightControlTask
*功能说明: 飞行控制相关任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(vFlightControlTask, pvParameters)
{
    Vector3f_t* gyro;
    static uint32_t count = 0;

    //遥控相关功能初始化
    RcInit();

    //控制参数初始化
    FlightControlInit();

    for(;;)
    {
        //从消息队列中获取数据
        xQueueReceive(messageQueue[GYRO_FOR_CONTROL], &gyro, (3 / portTICK_RATE_MS));

        //100Hz
        if(count % 10 == 0)
        {
            //遥控器各通道数据及失控检测
            RcCheck();

            //安全保护控制
            SafeControl();
        }

        //200Hz
        if(count % 5 == 0)
        {
            //位置外环控制
            PositionOuterControl();
        }

        //500Hz
        if(count % 2 == 0)
        {
            //用户控制模式下的操控逻辑处理
            UserControl();

            //自主控制任务实现（自动起飞、自动降落、自动返航、自动航线等）
            MissionControl();

            //位置内环控制
            PositionInnerControl();

            //姿态外环控制
            AttitudeOuterControl();

            //高度外环控制
            AltitudeOuterControl();
        }

        //飞行内环控制，包括姿态内环和高度内环
        FlightControlInnerLoop(*gyro);

        count++;
    }
}

/**********************************************************************************************************
*函 数 名: ControlTaskCreate
*功能说明: 控制相关任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ControlTaskCreate(void)
{
    xTaskCreate(vFlightControlTask, "flightControl", FLIGHTCONTROL_TASK_STACK, NULL, FLIGHTCONTROL_TASK_PRIORITY, &flightControlTask);
}

/**********************************************************************************************************
*函 数 名: GetFlightControlTaskStackRemain
*功能说明: 获取任务堆栈使用剩余
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
int16_t	GetFlightControlTaskStackRemain(void)
{
    return uxTaskGetStackHighWaterMark(flightControlTask);
}




