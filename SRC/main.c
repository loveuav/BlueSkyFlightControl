/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
                                
 * @版本：	 V0.0
 * @作者：   BlueSky
 * @QQ:      352707983
 * @论坛:    爱无人机 bbs.loveuav.com
 * @编译：   Keil ARM MDK 5.24@Win10 64位  
 
 * 1缩进等于4空格!
 
 * @文件     main.c
 * @说明     程序入口文件
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/

#include "FreeRTOS.h"
#include "task.h"

#include "board.h"
#include "messageQueue.h"
#include "parameter.h"

xTaskHandle startTask;

//启动任务
portTASK_FUNCTION(vStartTask, pvParameters) 
{	
    //CPU使用率统计功能初始化
    uTaskCPUUsageInit();

    //硬件初始化
    BoardInit();
    //飞控参数初始化
    ParamInit();
    
    //消息队列创建
    MessageQueueCreate();
        
    //创建用户任务
    ModuleTaskCreate();
    
    for(;;)
    {
        vTaskDelay(5000);
    }
}

int main(void)
{	
    //创建启动任务
    xTaskCreate(vStartTask, "startTask", 128, NULL, 0, &startTask); 
    //OS调度器启动
    vTaskStartScheduler();

    while(1)
    {
    }
}

//空闲钩子函数
void vApplicationIdleHook(void)
{
	
}



