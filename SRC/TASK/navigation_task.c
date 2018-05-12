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

void NavgationTaskCreate(void)
{
	xTaskCreate(vNavigationTask, "navigationTask", NAVIGATION_TASK_STACK, NULL, NAVIGATION_TASK_PRIORITY, &navigationTask); 
}

int16_t	GetNavgationTaskStackUse(void)
{
	return uxTaskGetStackHighWaterMark(navigationTask);
}




