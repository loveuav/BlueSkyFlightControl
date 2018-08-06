/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     log_task.c
 * @说明     飞行日志相关任务
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.08 
**********************************************************************************************************/
#include "TaskConfig.h"

#include "ff.h"		
#include "diskio.h"	
#include <stdio.h>

xTaskHandle logTask;

/**********************************************************************************************************
*函 数 名: vLogTask
*功能说明: 飞行日志任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(vLogTask, pvParameters) 
{	
    portTickType xLastWakeTime;
    
    FATFS   	fs;
	FIL     	file;
	static FRESULT		fresult;
    TCHAR		filename[16];
    
 	//挂起调度器
	vTaskSuspendAll();
    
    fresult = f_mount(0, &fs);
	sprintf(filename,"0:bluesky.txt");
	fresult = f_open(&file, filename, FA_OPEN_ALWAYS | FA_WRITE);

    if(fresult == FR_OK)
    {
        f_printf(&file,"Welcome to use the BlueSky FlightControl!\n");
        fresult = f_close(&file);
    }
    
    f_mount(0, NULL);
    
	//唤醒调度器
	xTaskResumeAll();   
    
    xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{

		vTaskDelayUntil(&xLastWakeTime, (10 / portTICK_RATE_MS));
	}
}

/**********************************************************************************************************
*函 数 名: LogTaskCreate
*功能说明: 飞行日志任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void LogTaskCreate(void)
{
	xTaskCreate(vLogTask, "log", LOG_TASK_STACK, NULL, LOG_TASK_PRIORITY, &logTask); 
}

/**********************************************************************************************************
*函 数 名: GetLogTaskStackRemain
*功能说明: 获取任务堆栈使用剩余
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
int16_t	GetLogTaskStackRemain(void)
{
	return uxTaskGetStackHighWaterMark(logTask);
}


