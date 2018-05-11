/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     messageQueue.c
 * @说明     消息队列，主要用于对数据传递实时性要求较高的任务间通信
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "messageQueue.h"

//声明消息队列句柄
QueueHandle_t messageQueue[QUEUE_NUM];

/**********************************************************************************************************
*函 数 名: MessageQueueCreate
*功能说明: 消息队列创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MessageQueueCreate(void)
{
	messageQueue[ACC_SENSOR_READ] =  xQueueCreate(2, sizeof(Vector3f_t *));
	messageQueue[GYRO_SENSOR_READ] =  xQueueCreate(2, sizeof(Vector3f_t *));
	messageQueue[TEMP_SENSOR_READ] =  xQueueCreate(2, sizeof(float *));

	messageQueue[GYRO_DATA_PRETREAT] =  xQueueCreate(2, sizeof(Vector3f_t *));
	messageQueue[ACC_DATA_PRETREAT] =  xQueueCreate(2, sizeof(Vector3f_t *));
	messageQueue[GYRO_FOR_CONTROL] =  xQueueCreate(2, sizeof(Vector3f_t *));
}






