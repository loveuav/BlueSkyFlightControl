/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html

 * @作者：   BlueSky
 * @QQ:      352707983
 * @论坛:    爱无人机 bbs.loveuav.com
 * @Q群:     472648354
 * @编译：   Keil ARM MDK 5.25

 * 1缩进等于4空格!
 * 文件编码格式为UTF-8

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
#include "faultDetect.h"

xTaskHandle startTask;

/**********************************************************************************************************
*函 数 名: TaskStackUseUpdate
*功能说明: 获取全部任务堆栈剩余情况
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void TaskStackUseUpdate(void)
{
    static int16_t stackUse[10];

    stackUse[0] = IMU_SENSOR_READ_TASK_STACK - GetImuSensorReadTaskStackRemain();
    stackUse[1] = SENSOR_UPDATE_TASK_STACK - GetSensorUpdateTaskStackRemain();
    stackUse[2] = IMU_DATA_PRETREAT_TASK_STACK - GetImuDataPreTreatTaskStackRemain();
    stackUse[3] = OTHER_SENSOR_TASK_STACK -GetOtherSensorTaskStackRemain();
    stackUse[4] = NAVIGATION_TASK_STACK - GetNavigationTaskStackRemain();
    stackUse[5] = FLIGHT_STATUS_TASK_STACK - GetFlightStatusTaskStackRemain();
    stackUse[6] = FLIGHTCONTROL_TASK_STACK - GetFlightControlTaskStackRemain();
    stackUse[7] = MESSAGE_TASK_STACK - GetMessageTaskStackRemain();
    stackUse[8] = LOG_TASK_STACK - GetLogTaskStackRemain();
    stackUse[9] = stackUse[9];
}

/**********************************************************************************************************
*函 数 名: CpuUseUpdate
*功能说明: 获取CPU使用率
*形    参: 无
*返 回 值: 使用率
**********************************************************************************************************/
float CpuUsageUpdate(void)
{
    static float cpuUsage;

    cpuUsage = Get_OSCPUusage();

    return cpuUsage;
}

/**********************************************************************************************************
*函 数 名: vStartTask
*功能说明: 系统启动任务，调用各类初始化函数，并创建消息队列和要运行的用户任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(vStartTask, pvParameters)
{
    //CPU使用率统计功能初始化
    uTaskCPUUsageInit();

    //硬件初始化
    BoardInit();
    //飞控参数初始化
    ParamInit();
    //系统故障检测初始化
    FaultDetectInit();

    //消息队列创建
    MessageQueueCreate();

    /*****************************用户任务创建********************************/
    //传感器及模块数据读取任务创建
    ModuleTaskCreate();

    //传感器数据预处理任务创建
    SensorTaskCreate();

    //导航计算任务创建
    NavigationTaskCreate();

    //飞行控制任务创建
    ControlTaskCreate();

    //数据通信任务创建
    MessageTaskCreate();

    //飞行日志任务创建
    LogTaskCreate();
    /*************************************************************************/

    //删除本任务
    //vTaskDelete(NULL);
    for(;;)
    {
        TaskStackUseUpdate();
        CpuUsageUpdate();
        vTaskDelay(5000);
    }
}

/**********************************************************************************************************
*函 数 名: main
*功能说明: 系统程序入口
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
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



