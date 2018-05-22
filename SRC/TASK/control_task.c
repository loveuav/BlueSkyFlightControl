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

xTaskHandle flightControlTask;

portTASK_FUNCTION(vFlightControlTask, pvParameters) 
{
	Vector3f_t* gyro;
	static uint32_t count = 0;
	
	for(;;)
	{
		//从消息队列中获取数据
		xQueueReceive(messageQueue[GYRO_FOR_CONTROL], &gyro, (3 / portTICK_RATE_MS)); 		

        //500Hz
        if(count % 2 == 0)	
		{
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

void ControlTaskCreate(void)
{
	xTaskCreate(vFlightControlTask, "flightControl", FLIGHTCONTROL_TASK_STACK, NULL, FLIGHTCONTROL_TASK_PRIORITY, &flightControlTask); 
}

int16_t	GetFlightControlTaskStackUse(void)
{
	return uxTaskGetStackHighWaterMark(flightControlTask);
}




