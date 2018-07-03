/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     drv_usbhid.c
 * @说明     usb hid驱动
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "drv_usbhid.h"
#include "RTL.h"
#include <rl_usb.h>
#define __NO_USB_LIB_C
#include "usb_config.c"
#include "FreeRTOS.h"
#include "task.h"

static UsbHidCallback usbHidCallbackFunc;

/**********************************************************************************************************
*函 数 名: UsbHid_Init
*功能说明: USB HID驱动初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void UsbHid_Init(void) 
{
    usbd_init();
    usbd_connect(__TRUE);
}

/**********************************************************************************************************
*函 数 名: UsbHid_Send
*功能说明: USB HID发送数据
*形    参: 发送数据缓冲区指针 数据长度
*返 回 值: 无
**********************************************************************************************************/
void UsbHid_Send(uint8_t *dataToSend, uint8_t length)
{
    static uint8_t data2send[64];
    static uint8_t data2send2[64];
    uint8_t datacnt = 0;
    
	if(length <= 64)
	{
        data2send[0] = length;
		for(datacnt=0; datacnt<length; datacnt++)
		{
			data2send[datacnt+1] = dataToSend[datacnt];
		}

        usbd_hid_get_report_trigger(0, data2send, 64);
        for(u8 i=0;i<64;i++)
            data2send[i] = 0;
	}
	else
    {
        data2send[0] = 63;
		for(datacnt=0; datacnt<64; datacnt++)
		{
			data2send[datacnt+1] = dataToSend[datacnt];
		}
        data2send2[0] = length - 63;
        for(datacnt=0; datacnt<length-63; datacnt++)
		{
			data2send2[datacnt+1] = dataToSend[datacnt+63];
		}
        
        usbd_hid_get_report_trigger(0, data2send, 64);
        for(u8 i=0;i<64;i++)
            data2send[i] = 0;

        vTaskDelay(1);
        
        usbd_hid_get_report_trigger(0, data2send2, 64);
        for(u8 i=0;i<64;i++)
            data2send2[i] = 0;

    } 
}

/**********************************************************************************************************
*函 数 名: UsbHid_SetRecvCallback
*功能说明: 设置USB HID数据接收回调函数
*形    参: 回调函数
*返 回 值: 无
**********************************************************************************************************/
void UsbHid_SetRecvCallback(UsbHidCallback usbHidCallback)
{
    usbHidCallbackFunc = usbHidCallback;
}

/**********************************************************************************************************
*函 数 名: usbd_hid_set_report
*功能说明: USB HID接收驱动回调函数
*形    参: 
*返 回 值: 无
**********************************************************************************************************/
void usbd_hid_set_report (U8 rtype, U8 rid, U8 *buf, int len, U8 req)
{
    switch (rtype) 
    {
        case HID_REPORT_OUTPUT:
            for(u8 i = 0; i<len; i++)
            {
                if(usbHidCallbackFunc != 0)
                    (*usbHidCallbackFunc)(*(buf+i));
            }
            break;
    }
}


