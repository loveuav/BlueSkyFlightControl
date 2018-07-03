/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     message.c
 * @说明     飞控数据通信，所有通信采用小端模式
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "message.h"
#include "messageSend.h"
#include "messageDecode.h"
#include "drv_usart.h"
#include "drv_usbhid.h"
#include "bsklink.h"

#include "ahrs.h"
#include "flightControl.h"
#include "gyroscope.h"
#include "navigation.h"
#include "accelerometer.h"
#include "barometer.h"
#include "motor.h"
#include "gps.h"

uint8_t sendFlag[0xFF];	            //发送标志位
uint8_t sendFreq[0xFF];	            //发送频率
uint8_t sortResult[0xFF];           
uint8_t sendList[MAX_SEND_FREQ];    //发送列表

BSKLINK_PAYLOAD_SENSOR_CALI_CMD_t sensorCali;

//static void DataSendDebug(void);
void SendFreqSort(void);
void SendListCreate(void);

/**********************************************************************************************************
*函 数 名: MessageInit
*功能说明: 飞控数据通信初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MessageInit(void)
{
    //设置数据通信串口接收中断回调函数
    Usart_SetIRQCallback(DATA_UART, MessageDecode);
    
    //初始化各帧的发送频率，各帧频率和不能超过MAX_SEND_FREQ
    sendFreq[BSKLINK_MSG_ID_FLIGHT_DATA]        = 30;
    sendFreq[BSKLINK_MSG_ID_SENSOR]             = 5; 
    sendFreq[BSKLINK_MSG_ID_SENSOR_CALI_DATA]   = 1; 
    sendFreq[BSKLINK_MSG_ID_RC_DATA]            = 5; 
	sendFreq[BSKLINK_MSG_ID_MOTOR]              = 0; 
    sendFreq[BSKLINK_MSG_ID_FLIGHT_STATUS]      = 1;
    sendFreq[BSKLINK_MSG_ID_GPS]                = 2; 
    sendFreq[BSKLINK_MSG_ID_BATTERY]            = 1;  
    sendFreq[BSKLINK_MSG_ID_HEARTBEAT]          = 1;     //心跳包发送频率为固定1Hz
    
    //生成发送列表
    SendFreqSort();
    SendListCreate();
}

/**********************************************************************************************************
*函 数 名: MessageSendLoop
*功能说明: 检测是否有需要发送的数据帧
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MessageSendLoop(void)
{    
    static uint32_t i = 0;
    
    //根据需求发送的数据帧
	if(sendFlag[BSKLINK_MSG_ID_SENSOR_CALI_CMD] == ENABLE) 						//传感器校准反馈 
        BsklinkSendSensorCaliCmd(&sendFlag[BSKLINK_MSG_ID_SENSOR_CALI_CMD], sensorCali.type, sensorCali.step, sensorCali.successFlag);                  
    if(sendFlag[BSKLINK_MSG_ID_PID_ATT] == ENABLE)
        BsklinkSendPidAtt(&sendFlag[BSKLINK_MSG_ID_PID_ATT]);                   //姿态PID    
    else if(sendFlag[BSKLINK_MSG_ID_PID_POS] == ENABLE)
        BsklinkSendPidPos(&sendFlag[BSKLINK_MSG_ID_PID_POS]);                   //位置PID
    //循环发送的数据
    else
    {
        //根据发送列表来使能对应的数据帧发送标志位
        sendFlag[sendList[(i++) % MAX_SEND_FREQ]] = ENABLE; 
        
        BsklinkSendFlightData(&sendFlag[BSKLINK_MSG_ID_FLIGHT_DATA]);          //基本飞行数据
        BsklinkSendFlightStatus(&sendFlag[BSKLINK_MSG_ID_FLIGHT_STATUS]);      //飞行状态信息
        BsklinkSendSensor(&sendFlag[BSKLINK_MSG_ID_SENSOR]);                   //传感器数据
        BsklinkSendSensorCaliData(&sendFlag[BSKLINK_MSG_ID_SENSOR_CALI_DATA]); //传感器校准数据
        BsklinkSendRcData(&sendFlag[BSKLINK_MSG_ID_RC_DATA]);                  //遥控通道数据
		BsklinkSendMotor(&sendFlag[BSKLINK_MSG_ID_MOTOR]);					   //电机输出
        BsklinkSendGps(&sendFlag[BSKLINK_MSG_ID_GPS]);                         //GPS数据
        BsklinkSendHeartBeat(&sendFlag[BSKLINK_MSG_ID_HEARTBEAT]);             //心跳包
    }
}

/**********************************************************************************************************
*函 数 名: MessageSetSensorCaliFeedback
*功能说明: 传感器校准反馈消息发送使能
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MessageSensorCaliFeedbackEnable(uint8_t type, uint8_t step, uint8_t success)
{
	sendFlag[BSKLINK_MSG_ID_SENSOR_CALI_CMD] = ENABLE;
	
	sensorCali.type = type;
	sensorCali.successFlag = success;
	sensorCali.step = step;
}

//static void DataSendDebug(void)
//{
//    static DATA_TYPE_t dataTemp;  
//    static uint8_t dataToSend[50];
//	uint8_t _cnt=0;

//	dataToSend[_cnt++] = 0xAA;
//	dataToSend[_cnt++] = 0xAA;
//	dataToSend[_cnt++] = 0x02;
//	dataToSend[_cnt++] = 0;
//	
//	dataTemp.i16 = GetCopterAngle().x * 10;
//	dataToSend[_cnt++] = dataTemp.byte[1];
//	dataToSend[_cnt++] = dataTemp.byte[0];
//	dataTemp.i16 = GetCopterAngle().y * 10;
//	dataToSend[_cnt++] = dataTemp.byte[1];
//	dataToSend[_cnt++] = dataTemp.byte[0];
//	dataTemp.i16 = GetCopterAngle().z * 10;
//	dataToSend[_cnt++] = dataTemp.byte[1];
//	dataToSend[_cnt++] = dataTemp.byte[0];
//	dataTemp.i16 = GetPosInnerCtlError().z;
//	dataToSend[_cnt++] = dataTemp.byte[1];
//	dataToSend[_cnt++] = dataTemp.byte[0];
//	dataTemp.i16 = GetPosOuterCtlError().z;
//	dataToSend[_cnt++] = dataTemp.byte[1];
//	dataToSend[_cnt++] = dataTemp.byte[0];
//	dataTemp.i16 = GetCopterVelocity().z;
//	dataToSend[_cnt++] = dataTemp.byte[1];
//	dataToSend[_cnt++] = dataTemp.byte[0];
//	dataTemp.i16 = nav.accelLpf.z * 4000;
//	dataToSend[_cnt++] = dataTemp.byte[1];
//	dataToSend[_cnt++] = dataTemp.byte[0];
//	dataTemp.i16 = nav.accel.x * 500;
//	dataToSend[_cnt++] = dataTemp.byte[1];
//	dataToSend[_cnt++] = dataTemp.byte[0];
//	dataTemp.i16 = GetCopterVelocity().x;
//	dataToSend[_cnt++] = dataTemp.byte[1];
//	dataToSend[_cnt++] = dataTemp.byte[0];

//	dataToSend[3] = _cnt-4;
//	
//	uint8_t sum = 0;
//	for(u8 i=0;i<_cnt;i++)
//		sum += dataToSend[i];
//	
//	dataToSend[_cnt++]=sum;
//	
//	DataSend(dataToSend, _cnt);    
//}


/**********************************************************************************************************
*函 数 名: SendFreqSort
*功能说明: 根据消息发送频率来给消息ID排序
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void SendFreqSort(void)
{
    uint8_t i = 0, j = 0;
    uint8_t temp;
    
    //先初始化消息ID排序序列
    for(i = 0; i<0xFF; i++)
        sortResult[i] = i;

    //开始按照发送频率来给消息ID排序
    for(i=0; i<0xFF; i++)
    {    
        for(j=i+1; j<0xFF; j++)
        {
            if(sendFreq[sortResult[j]] > sendFreq[sortResult[i]])
            {
                temp = sortResult[i];
                sortResult[i] = sortResult[j];
                sortResult[j] = temp;
            }
        }
    }
    
    //除了第一个，其它改为倒序，目的是为了让所有数据帧发送尽可能均匀
    uint8_t validNum = 0;
    for(i=0; i<0xFF; i++)
    {
        if(sendFreq[sortResult[i]] != 0)
            validNum++;        
    }   
    
    validNum -= 1;
    
    for(i=1; i<=validNum/2; i++)
    {
        temp = sortResult[i];
        sortResult[i] = sortResult[validNum + 1 - i];
        sortResult[validNum + 1 - i] = temp;
    }
}

/**********************************************************************************************************
*函 数 名: SendListCreate
*功能说明: 根据各消息帧的发送频率自动生成发送列表
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void SendListCreate(void)
{
    uint8_t sendNum = 0;
    uint8_t i, j;
    static float interval;
    uint8_t random;
    
    //判断总发送量是否超出最大发送频率，若超过则退出该函数
    for(i=0; i<0xFF; i++)
    {
        if(sendFreq[sortResult[i]] == 0)
            break;
        
        sendNum += sendFreq[sortResult[i]];
    }
    if(sendNum > MAX_SEND_FREQ)
        return;
    
    //开始生成发送列表
    for(i=0; i<0xFF; i++)
    {
        if(sendFreq[sortResult[i]] == 0)
            return;

        //发送间隔
        interval = (float)MAX_SEND_FREQ / sendFreq[sortResult[i]];
		//生成随机数，作为该帧数据在列表中的排序起始点，这样可以尽量使各帧数据分布均匀
        random   = GetRandom() % MAX_SEND_FREQ;
		
        for(j=0; j<sendFreq[sortResult[i]]; j++)
        {
            for(uint8_t k=0; k<MAX_SEND_FREQ-j*interval; k++)
            {
                if(sendList[(int16_t)(j*interval+k+random) % MAX_SEND_FREQ] == 0)
                {
                    sendList[(int16_t)(j*interval+k+random) % MAX_SEND_FREQ] = sortResult[i];
                    break;
                }
            }
        }
    }
}

/**********************************************************************************************************
*函 数 名: DataSend
*功能说明: 数据发送接口
*形    参: 数据指针 长度
*返 回 值: 无
**********************************************************************************************************/
void DataSend(uint8_t *data , uint8_t length)
{
    //串口发送
    Usart_SendData(DATA_UART, data, length);
    //USB HID发送
    UsbHid_Send(data, length);
}



