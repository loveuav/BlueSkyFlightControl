/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     drv_usb.c
 * @说明     usb转串口驱动
 * @版本  	 V1.2
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.07
**********************************************************************************************************/
#include "drv_usb.h"
#include "usb_dcd_int.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usb_conf.h"
#include "usb_bsp.h"
#include "usbd_cdc_core.h"

uint32_t connectCheckTime = 0;
uint8_t  connectFlag = 0;
uint32_t errorCheckCnt = 0;

USB_OTG_CORE_HANDLE  USB_OTG_dev;

USBD_Usr_cb_TypeDef USR_cb =
{
    USBD_USR_Init,
    USBD_USR_DeviceReset,
    USBD_USR_DeviceConfigured,
    USBD_USR_DeviceSuspended,
    USBD_USR_DeviceResumed,
    USBD_USR_DeviceConnected,
    USBD_USR_DeviceDisconnected,
};

static UsbCallback usbCallbackFunc;

/**********************************************************************************************************
*函 数 名: Usb_Init
*功能说明: USB驱动初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Usb_Init(void)
{
    USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
}

/**********************************************************************************************************
*函 数 名: Usb_Send
*功能说明: USB发送数据
*形    参: 发送数据缓冲区指针 数据长度
*返 回 值: 无
**********************************************************************************************************/
void Usb_Send(uint8_t *dataToSend, uint8_t length)
{
    if(connectFlag)
    {
        DCD_EP_Tx(&USB_OTG_dev, 1, dataToSend, length);
        DCD_SetEPStatus(&USB_OTG_dev,1, USB_OTG_EP_TX_VALID);
    }

    errorCheckCnt = 0;
}

/**********************************************************************************************************
*函 数 名: Usb_SetRecvCallback
*功能说明: 设置USB数据接收回调函数
*形    参: 回调函数
*返 回 值: 无
**********************************************************************************************************/
void Usb_SetRecvCallback(UsbCallback usbCallback)
{
    usbCallbackFunc = usbCallback;
}

/**********************************************************************************************************
*函 数 名: Usb_Receive
*功能说明: USB接收回调函数
*形    参: 数据接收buff指针 接收长度
*返 回 值: 无
**********************************************************************************************************/
void Usb_Receive(uint8_t *buf, uint8_t len)
{
    connectFlag = 1;
    errorCheckCnt = 0;

    for(u8 i = 0; i<len; i++)
    {
        if(usbCallbackFunc != 0)
            (*usbCallbackFunc)(*(buf+i));
    }
}

/**********************************************************************************************************
*函 数 名: Usb_ClearInterrupt
*功能说明: USB清除中断标志
*形    参: USB句柄
*返 回 值: 无
**********************************************************************************************************/
void Usb_ClearInterrupt(USB_OTG_CORE_HANDLE *pdev)
{
    USB_OTG_WRITE_REG32(&pdev->regs.DREGS->DIEPEMPMSK, 0);
}

/**********************************************************************************************************
*函 数 名: Usb_Receive
*功能说明: USB OTG中断服务函数，处理所有USB中断
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void OTG_FS_IRQHandler(void)
{
    if(connectFlag && errorCheckCnt > 200000)
    {
        Usb_ClearInterrupt(&USB_OTG_dev);
        connectFlag = 0;
    }
    errorCheckCnt++;

    USBD_OTG_ISR_Handler(&USB_OTG_dev);
}

/**********************************************************************************************************
*函 数 名:
*功能说明: USB用户自定义函数
*形    参:
*返 回 值: 无
**********************************************************************************************************/
void USBD_USR_Init(void)
{
}

void USBD_USR_DeviceReset(uint8_t speed)
{
}

void USBD_USR_DeviceConfigured(void)
{
}

void USBD_USR_DeviceSuspended(void)
{
}

void USBD_USR_DeviceResumed(void)
{
}

void USBD_USR_DeviceConnected(void)
{
}

void USBD_USR_DeviceDisconnected(void)
{
}


