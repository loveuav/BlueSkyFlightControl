/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     navigation_task.c
 * @说明     导航相关任务，包括姿态估计、速度估计和位置估计
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "TaskConfig.h"

#include "ahrs.h"
#include "magnetometer.h"

xTaskHandle navigationTask;


portTASK_FUNCTION(vNavigationTask, pvParameters) 
{	
	Vector3f_t* gyro;
	Vector3f_t* acc;

	vTaskDelay(500);
	
    //姿态估计参数初始化
    AHRSInit();
    
	for(;;)
	{
		//从消息队列中获取数据
		xQueueReceive(messageQueue[GYRO_DATA_PRETREAT], &gyro, (3 / portTICK_RATE_MS)); 
		xQueueReceive(messageQueue[ACC_DATA_PRETREAT], &acc, (3 / portTICK_RATE_MS)); 	

		//姿态估计
		AttitudeEstimate(*gyro, *acc, MagGetData());
	}
}

void NavigationTaskCreate(void)
{
	xTaskCreate(vNavigationTask, "navigationTask", NAVIGATION_TASK_STACK, NULL, NAVIGATION_TASK_PRIORITY, &navigationTask); 
}

int16_t	GetNavigationTaskStackUse(void)
{
	return uxTaskGetStackHighWaterMark(navigationTask);
}




