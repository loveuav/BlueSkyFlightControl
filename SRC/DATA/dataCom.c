/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     dataCom.c
 * @说明     飞控数据通信，所有通信采用小端模式
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "dataCom.h"
#include "dataSend.h"
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

uint8_t sendFlag[FRAME_NUM];

//static void DataSendDebug(void);

/**********************************************************************************************************
*函 数 名: DataSendLoop
*功能说明: 飞控数据发送循环
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void DataSendLoop(void)
{
    static uint32_t cnt;
    
    cnt++;
    
    //循环发送的数据帧
    if(cnt % 3 == 0)        //33Hz
    {
        sendFlag[FRAME_FLIGHT] = ENABLE;      
    }
    if(cnt % 10 == 0)       //10Hz
    {
        sendFlag[FRAME_RC] = ENABLE;   
    }
    if(cnt % 30 == 0)       //3Hz
    {
        sendFlag[FRAME_GPS] = ENABLE;   
    }
    
    //检测是否有需要发送的数据帧
    if(sendFlag[FRAME_FLIGHT])              //基本飞行数据
    {
        SendFlightData();   
        sendFlag[FRAME_FLIGHT] = DISABLE;
    }
    if(sendFlag[FRAME_RC])                  //遥控通道数据
    {
        SendRcData();  
        sendFlag[FRAME_RC] = DISABLE;
    }
    if(sendFlag[FRAME_GPS])                 //GPS数据
    {
        SendGpsData(); 
        sendFlag[FRAME_GPS] = DISABLE;
    }
    if(sendFlag[FRAME_PID_ATTINNER])        //姿态内环PID
    {
        SendPidAttInner();
        sendFlag[FRAME_PID_ATTINNER] = DISABLE;
    }   
    if(sendFlag[FRAME_PID_ATTOUTER])        //姿态外环PID
    {
        SendPidAttOuter();
        sendFlag[FRAME_PID_ATTOUTER] = DISABLE;
    }       
    if(sendFlag[FRAME_PID_POSINNER])        //位置内环PID
    {
        SendPidPosInner();
        sendFlag[FRAME_PID_POSINNER] = DISABLE;
    }        
    if(sendFlag[FRAME_PID_POSOUTER])        //位置外环PID
    {
        SendPidPosOuter();
        sendFlag[FRAME_PID_POSOUTER] = DISABLE;
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



