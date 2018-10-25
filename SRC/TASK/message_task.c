/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     message_task.c
 * @说明     飞控数据通信相关任务
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.06
**********************************************************************************************************/
#include "TaskConfig.h"

#include "message.h"

xTaskHandle messageTask;

/**********************************************************************************************************
*函 数 名: vMessageTask
*功能说明: 数据通信任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(vMessageTask, pvParameters)
{
    portTickType xLastWakeTime;

    //飞控数据通信初始化
    MessageInit();

    xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
        //发送飞控数据
        MessageSendLoop();

        vTaskDelayUntil(&xLastWakeTime, ((1000 / MAX_SEND_FREQ) / portTICK_RATE_MS));
    }
}

/**********************************************************************************************************
*函 数 名: MessageTaskCreate
*功能说明: 数据通信任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MessageTaskCreate(void)
{
    xTaskCreate(vMessageTask, "message", MESSAGE_TASK_STACK, NULL, MESSAGE_TASK_PRIORITY, &messageTask);
}

/**********************************************************************************************************
*函 数 名: GetMessageTaskStackRemain
*功能说明: 获取任务堆栈使用剩余
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
int16_t	GetMessageTaskStackRemain(void)
{
    return uxTaskGetStackHighWaterMark(messageTask);
}


