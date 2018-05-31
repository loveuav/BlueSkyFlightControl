/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     dataCom.c
 * @说明     飞控数据通信
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "dataCom.h"
#include "drv_usart.h"
#include "drv_usbhid.h"

#include "ahrs.h"
#include "flightControl.h"
#include "gyroscope.h"
#include "navigation.h"
#include "accelerometer.h"
#include "barometer.h"

DATA_TYPE_t dataTemp;  
uint8_t dataToSend[50];

static void DataSend(uint8_t *data , uint8_t length);
static void DataSendDebug(void);

void DataSendLoop(void)
{
    static uint32_t cnt;
    
    if(cnt % 2 == 0)
    {
        DataSendDebug();        
    }
    
    cnt++;
}

static void DataSendDebug(void)
{
	uint8_t _cnt=0;

	dataToSend[_cnt++] = 0xAA;
	dataToSend[_cnt++] = 0xAA;
	dataToSend[_cnt++] = 0x02;
	dataToSend[_cnt++] = 0;
	
	dataTemp.i16 = GetAttOuterCtlError().x * 100;
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataTemp.i16 = GetAttOuterCtlError().y * 100;
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataTemp.i16 = GetAttOuterCtlError().z * 100;
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataTemp.i16 = GetCopterVelocity().z;
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataTemp.i16 = GetCopterPosition().z;
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataTemp.i16 = BaroGetAlt();
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataTemp.i16 = GetCopterAccel().z * 4000;//ahrs.vectorRollPitchError.x * 1000;
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataTemp.i16 = nav.velocity2.z;//ahrs.vectorRollPitchError.y * 1000;
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataTemp.i16 = nav.velErrorInt.z;//ahrs.vectorRollPitchError.z * 1000;
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[0];

	dataToSend[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += dataToSend[i];
	
	dataToSend[_cnt++]=sum;
	
	DataSend(dataToSend, _cnt);    
}

/**********************************************************************************************************
*函 数 名: DataSend
*功能说明: 数据发送接口
*形    参: 数据指针 长度
*返 回 值: 无
**********************************************************************************************************/
static void DataSend(uint8_t *data , uint8_t length)
{
    //串口发送
    Usart_SendData(DATA_UART, data, length);
    //USB HID发送
    UsbHid_Send(data, length);
}



