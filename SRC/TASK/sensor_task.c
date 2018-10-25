/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     sensor_task.c
 * @说明     传感器校准及数据预处理相关任务
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05
**********************************************************************************************************/
#include "TaskConfig.h"

#include "gyroscope.h"
#include "accelerometer.h"
#include "magnetometer.h"
#include "barometer.h"
#include "gps.h"

xTaskHandle imuDataPreTreatTask;
xTaskHandle otherSensorTask;

/**********************************************************************************************************
*函 数 名: vImuDataPreTreatTask
*功能说明: IMU传感器数据预处理任务，任务优先级仅次于IMU传感器读取
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(vImuDataPreTreatTask, pvParameters)
{
    Vector3f_t* gyroRawData;
    Vector3f_t* accRawData;
    float*      tempRawData;
    Vector3f_t* accData  = pvPortMalloc(sizeof(Vector3f_t));
    Vector3f_t* gyroData = pvPortMalloc(sizeof(Vector3f_t));
    Vector3f_t* gyroLpfData = pvPortMalloc(sizeof(Vector3f_t));

    //挂起调度器
    vTaskSuspendAll();

    //陀螺仪预处理初始化
    GyroPreTreatInit();
    //加速度预处理初始化
    AccPreTreatInit();
    //IMU传感器恒温参数初始化
    ImuTempControlInit();

    //唤醒调度器
    xTaskResumeAll();

    for(;;)
    {
        //从消息队列中获取数据
        xQueueReceive(messageQueue[GYRO_SENSOR_READ], &gyroRawData, (3 / portTICK_RATE_MS));
        xQueueReceive(messageQueue[ACC_SENSOR_READ], &accRawData, (3 / portTICK_RATE_MS));
        xQueueReceive(messageQueue[TEMP_SENSOR_READ], &tempRawData, (3 / portTICK_RATE_MS));

        //陀螺仪校准
        GyroCalibration(*gyroRawData);
        //加速度校准
        AccCalibration(*accRawData);

        //陀螺仪数据预处理
        GyroDataPreTreat(*gyroRawData, *tempRawData, gyroData, gyroLpfData);
        //加速度数据预处理
        AccDataPreTreat(*accRawData, accData);

        //IMU安装误差校准
        ImuLevelCalibration();

        //IMU传感器恒温控制
        ImuTempControl(*tempRawData);

        //往下一级消息队列中填充数据
        xQueueSendToBack(messageQueue[ACC_DATA_PRETREAT], (void *)&accData, 0);
        xQueueSendToBack(messageQueue[GYRO_DATA_PRETREAT], (void *)&gyroData, 0);
        xQueueSendToBack(messageQueue[GYRO_FOR_CONTROL], (void *)&gyroLpfData, 0);
    }
}

/**********************************************************************************************************
*函 数 名: vOtherSensorTask
*功能说明: 其它传感器数据预处理任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(vOtherSensorTask, pvParameters)
{
    portTickType xLastWakeTime;
    static uint16_t count = 0;

    //挂起调度器
    vTaskSuspendAll();

    //磁力计校准参数初始化
    MagCaliDataInit();

    //唤醒调度器
    xTaskResumeAll();

    xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
        //100Hz
        if(count % 2 == 0)
        {
            //磁力计校准
            MagCalibration();

            //磁力计数据预处理
            MagDataPreTreat();
        }

        //25Hz
        if(count % 8 == 0)
        {
            //气压高度数据预处理
            BaroDataPreTreat();
        }

        //10Hz
        if(count % 20 == 0)
        {
            //GPS数据预处理
            GpsDataPreTreat();
        }

        //传感器健康状态检测
        SensorHealthCheck();

        count++;

        //睡眠5ms
        vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS));
    }
}

/**********************************************************************************************************
*函 数 名: SensorTaskCreate
*功能说明: 传感器数据预处理相关任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void SensorTaskCreate(void)
{
    xTaskCreate(vImuDataPreTreatTask, "imuDataPreTreat", IMU_DATA_PRETREAT_TASK_STACK, NULL, IMU_DATA_PRETREAT_TASK_PRIORITY, &imuDataPreTreatTask);
    xTaskCreate(vOtherSensorTask, "otherSensor", OTHER_SENSOR_TASK_STACK, NULL, OTHER_SENSOR_TASK_PRIORITY, &otherSensorTask);
}

/**********************************************************************************************************
*函 数 名: GetImuDataPreTreatTaskStackRemain
*功能说明: 获取任务堆栈使用剩余
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
int16_t	GetImuDataPreTreatTaskStackRemain(void)
{
    return uxTaskGetStackHighWaterMark(imuDataPreTreatTask);
}

/**********************************************************************************************************
*函 数 名: GetOtherSensorTaskStackRemain
*功能说明: 获取任务堆栈使用剩余
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
int16_t	GetOtherSensorTaskStackRemain(void)
{
    return uxTaskGetStackHighWaterMark(otherSensorTask);
}

