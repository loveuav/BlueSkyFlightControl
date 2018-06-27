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

#include "ahrs.h"
#include "flightControl.h"
#include "gyroscope.h"
#include "navigation.h"
#include "accelerometer.h"
#include "barometer.h"
#include "motor.h"
#include "gps.h"

uint8_t sendFlag[MSG_NUM];	//发送标志位
uint8_t sendFreq[MSG_NUM];	//发送频率

//static void DataSendDebug(void);

/**********************************************************************************************************
*函 数 名: MessageInit
*功能说明: 飞控数据通信初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MessageInit(void)
{
    //设置数据通信串口接收中断回调函数
    Usart_SetIRQCallback(DATA_UART, dataDecode);
}

/**********************************************************************************************************
*函 数 名: DataSendLoop
*功能说明: 飞控数据发送循环
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MessageSendLoop(void)
{
    static uint32_t cnt;
    
    cnt++;
    
    //循环发送的数据帧
    if(cnt % 3 == 0)   
        sendFlag[MSG_FLIGHT_DATA] = ENABLE;
    if(cnt % 7 == 0)  
        sendFlag[MSG_SENSOR] = ENABLE;         
    if(cnt % 11 == 0)     
        sendFlag[MSG_RC] = ENABLE;   
    if(cnt % 29 == 0)     
        sendFlag[MSG_GPS] = ENABLE;   
    
    //检测是否有需要发送的数据帧
    if(sendFlag[MSG_FLIGHT_DATA])          //基本飞行数据
    {
        BsklinkSendFlightData();
        sendFlag[MSG_FLIGHT_DATA] = DISABLE;
    }
	else if(sendFlag[MSG_FLIGHT_STATUS])   //飞行状态信息
    {
        BsklinkSendFlightStatus();
        sendFlag[MSG_FLIGHT_STATUS] = DISABLE;
    }
    else if(sendFlag[MSG_SENSOR])          //传感器数据
    {
        BsklinkSendSensor();   
        sendFlag[MSG_SENSOR] = DISABLE;
    }    
    else if(sendFlag[MSG_RC])              //遥控通道数据
    {
        BsklinkSendRcData();
        sendFlag[MSG_RC] = DISABLE;
    }
    else if(sendFlag[MSG_GPS])             //GPS数据
    {
        BsklinkSendGps(); 
        sendFlag[MSG_GPS] = DISABLE;
    }
    else if(sendFlag[MSG_PID_ATT])         //姿态PID
    {
        BsklinkSendPidAtt();
        sendFlag[MSG_PID_ATT] = DISABLE;
    }        
    else if(sendFlag[MSG_PID_POS])         //位置PID
    {
        BsklinkSendPidPos();
        sendFlag[MSG_PID_POS] = DISABLE;
    }        
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
//	dataTemp.i16 = GetCopterAngle().z * 10;//fc.posOuterTarget.z;//
//	dataToSend[_cnt++] = dataTemp.byte[1];
//	dataToSend[_cnt++] = dataTemp.byte[0];
//	dataTemp.i16 = GetPosInnerCtlError().z;//GetAttOuterCtlError().y * 10;//
//	dataToSend[_cnt++] = dataTemp.byte[1];
//	dataToSend[_cnt++] = dataTemp.byte[0];
//	dataTemp.i16 = GetPosOuterCtlError().z;//GetAttOuterCtlError().z * 10;//
//	dataToSend[_cnt++] = dataTemp.byte[1];
//	dataToSend[_cnt++] = dataTemp.byte[0];
//	dataTemp.i16 = GetCopterVelocity().z;//fc.attInnerCtlValue.x;//BaroGetAlt();
//	dataToSend[_cnt++] = dataTemp.byte[1];
//	dataToSend[_cnt++] = dataTemp.byte[0];
//	dataTemp.i16 = nav.accelLpf.z * 4000;//fc.attInnerCtlValue.y;//nav.velocity2.x;//
//	dataToSend[_cnt++] = dataTemp.byte[1];
//	dataToSend[_cnt++] = dataTemp.byte[0];
//	dataTemp.i16 = nav.accel.x * 500;//fc.attInnerCtlValue.z;//nav.velocity2.y;//nav.velocity2.z;//
//	dataToSend[_cnt++] = dataTemp.byte[1];
//	dataToSend[_cnt++] = dataTemp.byte[0];
//	dataTemp.i16 = GetCopterVelocity().x;//nav.velocity2.z;//fc.altInnerCtlValue;//ahrs.vectorRollPitchError.z * 1000;
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



