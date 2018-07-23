/**
  ******************************************************************************
  * @file    usb_dcd.c
  * @author  MCD Application Team
  * @version V2.1.0
  * @date    19-March-2012
  * @brief   Peripheral Device Interface Layer
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usb_dcd.h"
#include "usb_bsp.h"


/** @addtogroup USB_OTG_DRIVER
* @{
*/

/** @defgroup USB_DCD 
* @brief This file is the interface between EFSL ans Host mass-storage class
* @{
*/


/** @defgroup USB_DCD_Private_Defines
* @{
*/ 
/**
* @}
*/ 


/** @defgroup USB_DCD_Private_TypesDefinitions
* @{
*/ 
/**
* @}
*/ 



/** @defgroup USB_DCD_Private_Macros
* @{
*/ 
/**
* @}
*/ 


/** @defgroup USB_DCD_Private_Variables
* @{
*/ 
/**
* @}
*/ 


/** @defgroup USB_DCD_Private_FunctionPrototypes
* @{
*/ 

/**
* @}
*/ 


/** @defgroup USB_DCD_Private_Functions
* @{
*/ 



void DCD_Init(USB_OTG_CORE_HANDLE *pdev , 
              USB_OTG_CORE_ID_TypeDef coreID)
{
  uint32_t i;
  USB_OTG_EP *ep;
  
  USB_OTG_SelectCore (pdev , coreID);	//初始化核心寄存器地址，选择设备--全速/高速
  
  pdev->dev.device_status = USB_OTG_DEFAULT;
  pdev->dev.device_address = 0;				//设备初始地址为0
  
  /* Init ep structure */
  for (i = 0; i < pdev->cfg.dev_endpoints ; i++)	//依次配置设备输入端点
  {
    ep = &pdev->dev.in_ep[i];								//读设备输入端点i
    /* Init ep structure */
    ep->is_in = 1;													//端点为输入
    ep->num = i;														//端点编号
    ep->tx_fifo_num = i;										//端点发送fifo编号
    /* Control until ep is actvated */
    ep->type = EP_TYPE_CTRL;								//端点为控制端点
    ep->maxpacket =  USB_OTG_MAX_EP0_SIZE;	//端点最大包大小
    ep->xfer_buff = 0;											//端点交易缓冲区
    ep->xfer_len = 0;												//端点交易数据长度
  }
  
  for (i = 0; i < pdev->cfg.dev_endpoints; i++)		//依次配置设备输出端点
  {
    ep = &pdev->dev.out_ep[i];							//读设备输出端点i
    /* Init ep structure */
    ep->is_in = 0;													//端点为输出
    ep->num = i;														//端点编号
    ep->tx_fifo_num = i;										//端点发送fifo编号
    /* Control until ep is activated */
    ep->type = EP_TYPE_CTRL;								//端点为控制端点
    ep->maxpacket = USB_OTG_MAX_EP0_SIZE;		//端点最大包大小
    ep->xfer_buff = 0;											//端点交易缓冲区
    ep->xfer_len = 0;												//端点交易数据长度
  }
  
  USB_OTG_DisableGlobalInt(pdev);	//使能设备应用程序触发的中断
  
  /*Init the Core (common init.) */
  USB_OTG_CoreInit(pdev);					//初始化模块USB、模块、AHB配置寄存器


  /* Force Device Mode*/
  USB_OTG_SetCurrentMode(pdev, DEVICE_MODE);	//强制设置当前模式为设备模式
  
  /* Init Device */
  USB_OTG_CoreInitDev(pdev);
  
  
  /* Enable USB Global interrupt */
  USB_OTG_EnableGlobalInt(pdev);
}


/**
* @brief  Configure an EP
* @param pdev : Device instance
* @param epdesc : Endpoint Descriptor
* @retval : status
*/
uint32_t DCD_EP_Open(USB_OTG_CORE_HANDLE *pdev , 
                     uint8_t ep_addr,
                     uint16_t ep_mps,
                     uint8_t ep_type)
{
  USB_OTG_EP *ep;
  
  if ((ep_addr & 0x80) == 0x80)		//是否为输入端点
  {
    ep = &pdev->dev.in_ep[ep_addr & 0x7F];	//将设备输入端点[ep_addr & 0x7F]赋值给ep
  }
  else
  {
    ep = &pdev->dev.out_ep[ep_addr & 0x7F];	//将设备输出端点[ep_addr & 0x7F]赋值给ep
  }
	
  ep->num   = ep_addr & 0x7F;								//端点号为ep_addr & 0x7F
  
  ep->is_in = (0x80 & ep_addr) != 0;				//端点是否为输入端点
  ep->maxpacket = ep_mps;										//端点最大包
  ep->type = ep_type;												//端点类型
	
  if (ep->is_in)			//如果为输入端点
  {
    /* Assign a Tx FIFO */
    ep->tx_fifo_num = ep->num;				//发送fifo为端点号
  }
  /* Set initial data PID. */
  if (ep_type == USB_OTG_EP_BULK )		//如果为批量端点
  {
    ep->data_pid_start = 0;						//起始包PID为0
  }
  USB_OTG_EPActivate(pdev , ep );			//激活端点ep
  return 0;
}
/**
* @brief  called when an EP is disabled
* @param pdev: device instance
* @param ep_addr: endpoint address
* @retval : status
*/
uint32_t DCD_EP_Close(USB_OTG_CORE_HANDLE *pdev , uint8_t  ep_addr)	//关闭设备端点ep_addr
{
  USB_OTG_EP *ep;
  
  if ((ep_addr&0x80) == 0x80)		//如果为输入端点
  {
    ep = &pdev->dev.in_ep[ep_addr & 0x7F];	//读输入端点[ep_addr & 0x7F]的值
  }
  else
  {
    ep = &pdev->dev.out_ep[ep_addr & 0x7F];	//读输出端点[ep_addr & 0x7F]的值
  }
  ep->num   = ep_addr & 0x7F;								//设置端点号
  ep->is_in = (0x80 & ep_addr) != 0;				//设置端点为输入或者输出
  USB_OTG_EPDeactivate(pdev , ep );					//断开端点ep
  return 0;
}


/**
* @brief  DCD_EP_PrepareRx
* @param pdev: device instance
* @param ep_addr: endpoint address
* @param pbuf: pointer to Rx buffer
* @param buf_len: data length
* @retval : status
*/
uint32_t   DCD_EP_PrepareRx( USB_OTG_CORE_HANDLE *pdev,	//设备端点ep_addr接收buf_len个数据到pbuf
                            uint8_t   ep_addr,
                            uint8_t *pbuf,                        
                            uint16_t  buf_len)
{
  USB_OTG_EP *ep;
  
  ep = &pdev->dev.out_ep[ep_addr & 0x7F];	//读取输出端点[ep_addr & 0x7F]的值
  
  /*setup and start the Xfer */
  ep->xfer_buff = pbuf;  			//设置接收地址为pbuf
  ep->xfer_len = buf_len;			//设置接收长度为buf_len
  ep->xfer_count = 0;					//设置接收计数为xfer_count
  ep->is_in = 0;							//输出端点
  ep->num = ep_addr & 0x7F;		//端点编号为ep_addr & 0x7F
  
  if (pdev->cfg.dma_enable == 1)			//如果允许dma传输
  {
    ep->dma_addr = (uint32_t)pbuf;  
  }
  
  if ( ep->num == 0 )									//如果端点为0
  {
    USB_OTG_EP0StartXfer(pdev , ep);	//端点0数据传输并开始传输
  }
  else
  {
    USB_OTG_EPStartXfer(pdev, ep );		//处理端点数据传输并开始传输
  }
  return 0;
}

/**
* @brief  Transmit data over USB
* @param pdev: device instance
* @param ep_addr: endpoint address
* @param pbuf: pointer to Tx buffer
* @param buf_len: data length
* @retval : status
*/
uint32_t  DCD_EP_Tx ( USB_OTG_CORE_HANDLE *pdev,	//端点ep_addr发送buf_len个pbuf数据到主机 
                     uint8_t   ep_addr,
                     uint8_t   *pbuf,
                     uint32_t   buf_len)
{
  USB_OTG_EP *ep;
  
  ep = &pdev->dev.in_ep[ep_addr & 0x7F];	//读输入端点[ep_addr & 0x7F]
  
  /* Setup and start the Transfer */
  ep->is_in = 1;													//端点为输入端点
  ep->num = ep_addr & 0x7F;  							//端点地址
  ep->xfer_buff = pbuf;										//传输缓冲区
  ep->dma_addr = (uint32_t)pbuf;  				//dma地址
  ep->xfer_count = 0;											//输入数据计数
  ep->xfer_len  = buf_len;								//将要输入的数据长度
  
  if ( ep->num == 0 )											//如果为端点0
  {
    USB_OTG_EP0StartXfer(pdev , ep);			//端点0数据传输并开始传输
  }
  else																		//不为端点0
  {
    USB_OTG_EPStartXfer(pdev, ep );				//处理端点数据传输并开始传输
  }
  return 0;
}


/**
* @brief  Stall an endpoint.
* @param pdev: device instance
* @param epnum: endpoint address
* @retval : status
*/
uint32_t  DCD_EP_Stall (USB_OTG_CORE_HANDLE *pdev, uint8_t   epnum)	//端点epnum挂起
{
  USB_OTG_EP *ep;
  if ((0x80 & epnum) == 0x80)							//如果端点为输入端点
  {
    ep = &pdev->dev.in_ep[epnum & 0x7F];	//读输入端点[epnum & 0x7F]
  }
  else																		//端点为输出端点
  {
    ep = &pdev->dev.out_ep[epnum];				//读输出端点[epnum & 0x7F]
  }

  ep->is_stall = 1;												//端点挂起
  ep->num   = epnum & 0x7F;								//端点编号
  ep->is_in = ((epnum & 0x80) == 0x80);		//输入或输出
  
  USB_OTG_EPSetStall(pdev , ep);					//设置端点ep挂起
  return (0);
}


/**
* @brief  Clear stall condition on endpoints.
* @param pdev: device instance
* @param epnum: endpoint address
* @retval : status
*/
uint32_t  DCD_EP_ClrStall (USB_OTG_CORE_HANDLE *pdev, uint8_t epnum)	//清除端点epnum挂起状态
{
  USB_OTG_EP *ep;
  if ((0x80 & epnum) == 0x80)								//如果端点为输入端点
  {
    ep = &pdev->dev.in_ep[epnum & 0x7F];    //读输入端点[epnum & 0x7F]
  }
  else																			//端点为输出端点
  {
    ep = &pdev->dev.out_ep[epnum];					//读输出端点
  }
  
  ep->is_stall = 0;  												//清除挂起位
  ep->num   = epnum & 0x7F;									//端点号
  ep->is_in = ((epnum & 0x80) == 0x80);			//输入或者输出
  
  USB_OTG_EPClearStall(pdev , ep);					//清除端点ep挂起状态
  return (0);
}


/**
* @brief  This Function flushes the FIFOs.
* @param pdev: device instance
* @param epnum: endpoint address
* @retval : status
*/
uint32_t  DCD_EP_Flush (USB_OTG_CORE_HANDLE *pdev , uint8_t epnum)	//刷新FIFO
{

  if ((epnum & 0x80) == 0x80)		//如果为输入端点
  {
    USB_OTG_FlushTxFifo(pdev, epnum & 0x7F);	//刷新TxFIFO epnum & 0x7F
  }
  else													//输出端点
  {
    USB_OTG_FlushRxFifo(pdev);								//刷新RxFIFO
  }

  return (0);
}


/**
* @brief  This Function set USB device address
* @param pdev: device instance
* @param address: new device address
* @retval : status
*/
void  DCD_EP_SetAddress (USB_OTG_CORE_HANDLE *pdev, uint8_t address)	//设置设备地址为address
{
  USB_OTG_DCFG_TypeDef  dcfg;	//设备配置寄存器
  dcfg.d32 = 0;
  dcfg.b.devaddr = address;		//设置设备地址为address
  USB_OTG_MODIFY_REG32( &pdev->regs.DREGS->DCFG, 0, dcfg.d32);	//修改设备配置寄存器
}

/**
* @brief  Connect device (enable internal pull-up)
* @param pdev: device instance
* @retval : None
*/
void  DCD_DevConnect (USB_OTG_CORE_HANDLE *pdev)		//设备连接
{
#ifndef USE_OTG_MODE
  USB_OTG_DCTL_TypeDef  dctl;		//设备控制寄存器
  dctl.d32 = USB_OTG_READ_REG32(&pdev->regs.DREGS->DCTL);	//读设备控制寄存器
  /* Connect device */
  dctl.b.sftdiscon  = 0;				//0: 正常工作	1: 使主机收到设备断开连接的事件
  USB_OTG_WRITE_REG32(&pdev->regs.DREGS->DCTL, dctl.d32);	//写设备控制寄存器
  USB_OTG_BSP_mDelay(3);				//延时3ms
#endif
}


/**
* @brief  Disconnect device (disable internal pull-up)
* @param pdev: device instance
* @retval : None
*/
void  DCD_DevDisconnect (USB_OTG_CORE_HANDLE *pdev)	//设备断开
{
#ifndef USE_OTG_MODE
  USB_OTG_DCTL_TypeDef  dctl;		//设备控制寄存器
  dctl.d32 = USB_OTG_READ_REG32(&pdev->regs.DREGS->DCTL);	//读设备控制寄存器
  /* Disconnect device for 3ms */
  dctl.b.sftdiscon  = 1;				//0: 正常工作	1: 使主机收到设备断开连接的事件
  USB_OTG_WRITE_REG32(&pdev->regs.DREGS->DCTL, dctl.d32);	//写设备控制寄存器
  USB_OTG_BSP_mDelay(3);				//延时3ms
#endif
}


/**
* @brief  returns the EP Status
* @param  pdev : Selected device
*         epnum : endpoint address
* @retval : EP status
*/

uint32_t DCD_GetEPStatus(USB_OTG_CORE_HANDLE *pdev ,uint8_t epnum)	//获取端点epnum的状态
{
  USB_OTG_EP *ep;
  uint32_t Status = 0;  
  
  if ((0x80 & epnum) == 0x80)	//如果为输入端点
  {
    ep = &pdev->dev.in_ep[epnum & 0x7F];    //输入端点[epnum & 0x7F]的地址
  }
  else												//输出端点
  {
    ep = &pdev->dev.out_ep[epnum];					//输出端点[epnum & 0x7F]的地址
  }
  
  Status = USB_OTG_GetEPStatus(pdev ,ep);		//读取端点ep的状态

  /* Return the current status */
  return Status;														//函数返回端点ep的状态
}

/**
* @brief  Set the EP Status
* @param  pdev : Selected device
*         Status : new Status
*         epnum : EP address
* @retval : None
*/
void DCD_SetEPStatus (USB_OTG_CORE_HANDLE *pdev , uint8_t epnum , uint32_t Status)	//设置端点epnum的状态为Status
{
  USB_OTG_EP *ep;
  
  if ((0x80 & epnum) == 0x80)								//如果端点为输入端点
  {
    ep = &pdev->dev.in_ep[epnum & 0x7F];    //输入端点[epnum & 0x7F]的地址
  }
  else																			//输出端点
  {
    ep = &pdev->dev.out_ep[epnum];					//输出端点[epnum & 0x7F]的地址
  }
  
   USB_OTG_SetEPStatus(pdev ,ep , Status);	//设置端点ep的状态为Status
}

/**
* @}
*/ 

/**
* @}
*/ 

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
