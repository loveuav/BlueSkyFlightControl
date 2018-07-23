/**
  ******************************************************************************
  * @file    usb_regs.h
  * @author  MCD Application Team
  * @version V2.1.0
  * @date    19-March-2012
  * @brief   hardware registers
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_OTG_REGS_H__
#define __USB_OTG_REGS_H__

/* Includes ------------------------------------------------------------------*/
#include "usb_conf.h"


/** @addtogroup USB_OTG_DRIVER
  * @{
  */
  
/** @defgroup USB_REGS
  * @brief This file is the 
  * @{
  */ 


/** @defgroup USB_REGS_Exported_Defines
  * @{
  */ 

#define USB_OTG_HS_BASE_ADDR                 0x40040000
#define USB_OTG_FS_BASE_ADDR                 0x50000000

#define USB_OTG_CORE_GLOBAL_REGS_OFFSET      0x000	//核心全局寄存器偏移地址
#define USB_OTG_DEV_GLOBAL_REG_OFFSET        0x800	//设备全局寄存器偏移地址
#define USB_OTG_DEV_IN_EP_REG_OFFSET         0x900	//设备输入端点寄存器偏移地址
#define USB_OTG_EP_REG_OFFSET                0x20		//端点寄存器偏移地址
#define USB_OTG_DEV_OUT_EP_REG_OFFSET        0xB00	//设备输出端点寄存器偏移地址
#define USB_OTG_HOST_GLOBAL_REG_OFFSET       0x400	//主机全局寄存器偏移地址
#define USB_OTG_HOST_PORT_REGS_OFFSET        0x440	//主机端口寄存器偏移地址
#define USB_OTG_HOST_CHAN_REGS_OFFSET        0x500	//主机通道寄存器偏移地址
#define USB_OTG_CHAN_REGS_OFFSET             0x20		//通道寄存器偏移地址
#define USB_OTG_PCGCCTL_OFFSET               0xE00	//电源和时钟门控控制寄存器偏移地址
#define USB_OTG_DATA_FIFO_OFFSET             0x1000	//数据FIFO偏移地址
#define USB_OTG_DATA_FIFO_SIZE               0x1000	//数据FIFO大小


#define USB_OTG_MAX_TX_FIFOS                 15			//最大发送FIFO数量

#define USB_OTG_HS_MAX_PACKET_SIZE           512		//HS最大包大小
#define USB_OTG_FS_MAX_PACKET_SIZE           64			//FS最大包大小
#define USB_OTG_MAX_EP0_SIZE                 64			//端点最大包大小
/**
  * @}
  */ 

/** @defgroup USB_REGS_Exported_Types
  * @{
  */ 

/** @defgroup __USB_OTG_Core_register
  * @{
  */
typedef struct _USB_OTG_GREGS  //000h
{
  __IO uint32_t GOTGCTL;      /* USB_OTG Control and Status Register    000h*/
  __IO uint32_t GOTGINT;      /* USB_OTG Interrupt Register             004h*/
  __IO uint32_t GAHBCFG;      /* Core AHB Configuration Register    		008h*/
  __IO uint32_t GUSBCFG;      /* Core USB Configuration Register    		00Ch*/
  __IO uint32_t GRSTCTL;      /* Core Reset Register                		010h*/
  __IO uint32_t GINTSTS;      /* Core Interrupt Register            		014h*/
  __IO uint32_t GINTMSK;      /* Core Interrupt Mask Register       		018h*/
  __IO uint32_t GRXSTSR;      /* Receive Sts Q Read Register        		01Ch*/
  __IO uint32_t GRXSTSP;      /* Receive Sts Q Read & POP Register  		020h*/
  __IO uint32_t GRXFSIZ;      /* Receive FIFO Size Register         		024h*/
  __IO uint32_t DIEPTXF0_HNPTXFSIZ;   /* EP0 / Non Periodic Tx FIFO Size Register 028h*/
  __IO uint32_t HNPTXSTS;     /* Non Periodic Tx FIFO/Queue Sts reg 		02Ch*/
  uint32_t Reserved30[2];     /* Reserved                           		030h*/
  __IO uint32_t GCCFG;        /* General Purpose IO Register        		038h*/
  __IO uint32_t CID;          /* User ID Register                   		03Ch*/
  uint32_t  Reserved40[48];   /* Reserved                      					040h-0FFh*/
  __IO uint32_t HPTXFSIZ; /* Host Periodic Tx FIFO Size Reg     				100h*/
  __IO uint32_t DIEPTXF[USB_OTG_MAX_TX_FIFOS];/* dev Periodic Transmit FIFO */
}
USB_OTG_GREGS;
/**
  * @}
  */


/** @defgroup __device_Registers
  * @{
  */
typedef struct _USB_OTG_DREGS // 800h
{
  __IO uint32_t DCFG;         /* dev Configuration Register   800h*/
  __IO uint32_t DCTL;         /* dev Control Register         804h*/
  __IO uint32_t DSTS;         /* dev Status Register (RO)     808h*/
  uint32_t Reserved0C;        /* Reserved                     80Ch*/
  __IO uint32_t DIEPMSK;   		/* dev IN Endpoint Mask         810h*/
  __IO uint32_t DOEPMSK;  		/* dev OUT Endpoint Mask        814h*/
  __IO uint32_t DAINT;     		/* dev All Endpoints Itr Reg    818h*/
  __IO uint32_t DAINTMSK; 		/* dev All Endpoints Itr Mask   81Ch*/
  uint32_t  Reserved20;     	/* Reserved                     820h*/
  uint32_t Reserved9;       	/* Reserved                     824h*/
  __IO uint32_t DVBUSDIS;    	/* dev VBUS discharge Register  828h*/
  __IO uint32_t DVBUSPULSE;  	/* dev VBUS Pulse Register      82Ch*/
  __IO uint32_t DTHRCTL;     	/* dev thr                      830h*/
  __IO uint32_t DIEPEMPMSK; 	/* dev empty msk             834h*/
  __IO uint32_t DEACHINT;    	/* dedicated EP interrupt       838h*/
  __IO uint32_t DEACHMSK;    	/* dedicated EP msk             83Ch*/  
  uint32_t Reserved40;      	/* dedicated EP mask           840h*/
  __IO uint32_t DINEP1MSK;  	/* dedicated EP mask           844h*/
  uint32_t  Reserved44[15]; 	/* Reserved                 844-87Ch*/
  __IO uint32_t DOUTEP1MSK; 	/* dedicated EP msk            884h*/   
}
USB_OTG_DREGS;
/**
  * @}
  */


/** @defgroup __IN_Endpoint-Specific_Register
  * @{
  */
typedef struct _USB_OTG_INEPREGS
{
  __IO uint32_t DIEPCTL; /* dev IN Endpoint Control Reg 900h + (ep_num * 20h) + 00h*/
  uint32_t Reserved04;             /* Reserved                       900h + (ep_num * 20h) + 04h*/
  __IO uint32_t DIEPINT; /* dev IN Endpoint Itr Reg     900h + (ep_num * 20h) + 08h*/
  uint32_t Reserved0C;             /* Reserved                       900h + (ep_num * 20h) + 0Ch*/
  __IO uint32_t DIEPTSIZ; /* IN Endpoint Txfer Size   900h + (ep_num * 20h) + 10h*/
  __IO uint32_t DIEPDMA; /* IN Endpoint DMA Address Reg    900h + (ep_num * 20h) + 14h*/
  __IO uint32_t DTXFSTS;/*IN Endpoint Tx FIFO Status Reg 900h + (ep_num * 20h) + 18h*/
  uint32_t Reserved18;             /* Reserved  900h+(ep_num*20h)+1Ch-900h+ (ep_num * 20h) + 1Ch*/
}
USB_OTG_INEPREGS;
/**
  * @}
  */


/** @defgroup __OUT_Endpoint-Specific_Registers
  * @{
  */
typedef struct _USB_OTG_OUTEPREGS
{
  __IO uint32_t DOEPCTL;       /* dev OUT Endpoint Control Reg  B00h + (ep_num * 20h) + 00h*/
  uint32_t Reserved04;         /* Reserved                      B00h + (ep_num * 20h) + 04h*/
  __IO uint32_t DOEPINT;       /* dev OUT Endpoint Itr Reg      B00h + (ep_num * 20h) + 08h*/
  uint32_t Reserved0C;         /* Reserved                      B00h + (ep_num * 20h) + 0Ch*/
  __IO uint32_t DOEPTSIZ;      /* dev OUT Endpoint Txfer Size   B00h + (ep_num * 20h) + 10h*/
  __IO uint32_t DOEPDMA;       /* dev OUT Endpoint DMA Address  B00h + (ep_num * 20h) + 14h*/
  uint32_t Reserved18[2];      /* Reserved B00h + (ep_num * 20h) + 18h - B00h + (ep_num * 20h) + 1Ch*/
}
USB_OTG_OUTEPREGS;
/**
  * @}
  */


/** @defgroup __Host_Mode_Register_Structures
  * @{
  */
typedef struct _USB_OTG_HREGS
{
  __IO uint32_t HCFG;             /* Host Configuration Register    400h*/
  __IO uint32_t HFIR;      /* Host Frame Interval Register   404h*/
  __IO uint32_t HFNUM;         /* Host Frame Nbr/Frame Remaining 408h*/
  uint32_t Reserved40C;                   /* Reserved                       40Ch*/
  __IO uint32_t HPTXSTS;   /* Host Periodic Tx FIFO/ Queue Status 410h*/
  __IO uint32_t HAINT;   /* Host All Channels Interrupt Register 414h*/
  __IO uint32_t HAINTMSK;   /* Host All Channels Interrupt Mask 418h*/
}
USB_OTG_HREGS;
/**
  * @}
  */


/** @defgroup __Host_Channel_Specific_Registers
  * @{
  */
typedef struct _USB_OTG_HC_REGS
{
  __IO uint32_t HCCHAR;
  __IO uint32_t HCSPLT;
  __IO uint32_t HCINT;
  __IO uint32_t HCINTMSK;
  __IO uint32_t HCTSIZ;
  __IO uint32_t HCDMA;
  uint32_t Reserved[2];
}
USB_OTG_HC_REGS;
/**
  * @}
  */


/** @defgroup __otg_Core_registers
  * @{
  */
typedef struct USB_OTG_core_regs //000h
{
  USB_OTG_GREGS         *GREGS;
  USB_OTG_DREGS         *DREGS;
  USB_OTG_HREGS         *HREGS;
  USB_OTG_INEPREGS      *INEP_REGS[USB_OTG_MAX_TX_FIFOS];
  USB_OTG_OUTEPREGS     *OUTEP_REGS[USB_OTG_MAX_TX_FIFOS];
  USB_OTG_HC_REGS       *HC_REGS[USB_OTG_MAX_TX_FIFOS];
  __IO uint32_t         *HPRT0;
  __IO uint32_t         *DFIFO[USB_OTG_MAX_TX_FIFOS];
  __IO uint32_t         *PCGCCTL;
}
USB_OTG_CORE_REGS , *PUSB_OTG_CORE_REGS;

typedef union _USB_OTG_GOTGCTL_TypeDef //OTG_FS控制状态寄存器OTG_FS_GOTGCTL 0x000
{
  uint32_t d32;
  struct
  {
uint32_t sesreqscs :			//会话请求成功session request success
    1;
uint32_t sesreq :					//会话请求session request
    1;
uint32_t Reserved2_7 :		//
    6;
uint32_t hstnegscs :			//主机协商成功host negotiation success
    1;
uint32_t hnpreq :					//HNP 请求HNP request
    1;
uint32_t hstsethnpen :		//主机设置HNP使能host set HNP enable
    1;
uint32_t devhnpen :				//使能设备HNP特性device HNP enabled
    1;
uint32_t Reserved12_15 :	//
    4;
uint32_t conidsts :				//连接器ID状态connector ID status
    1;
uint32_t dbct :						//长/短去抖动时间long/short debounce time
    1;
uint32_t asesvld :				//A 会话有效A-session valid
    1;
uint32_t bsesvld :				//B会话有效B-session valid
    1;
uint32_t Reserved20_31 :	//
    12;
  }
  b;
} USB_OTG_GOTGCTL_TypeDef ;

typedef union _USB_OTG_GOTGINT_TypeDef //中断寄存器OTG_FS_GOTGINT 0x004
{
  uint32_t d32;
  struct
  {
uint32_t Reserved0_1 :			//
    2;
uint32_t sesenddet :				//检测到会话结束session end detected
    1;
uint32_t Reserved3_7 :			//
    5;
uint32_t sesreqsucstschng :	//会话请求成功状态更改session request success status change
    1;
uint32_t hstnegsucstschng :	//主机协商成功状态更改host negotiation success status change
    1;
uint32_t reserver10_16 :		//
    7;
uint32_t hstnegdet :				//检测到主机协商host negotiation detected
    1;
uint32_t adevtoutchng :			//A器件超时更改A-device timeout change
    1;
uint32_t debdone :					//去抖动完成debounce done
    1;
uint32_t Reserved31_20 :		//
    12;
  }
  b;
} USB_OTG_GOTGINT_TypeDef ;

typedef union _USB_OTG_GAHBCFG_TypeDef //配置寄存器OTG_FS_GAHBCFG  0x008
{
  uint32_t d32;
  struct
  {
uint32_t glblintrmsk :						//全局中断屏蔽global interrupt mask
    1;										//0: 屏蔽应用程序触发的中断 1: 取消对应用程序触发的中断的屏蔽
uint32_t hburstlen :			
    4;
uint32_t dmaenable :
    1;
uint32_t Reserved :
    1;
uint32_t nptxfemplvl_txfemplvl :	//TxFIFO空级别TxFIFO empty level
    1;										//0: 半空状态  1: 全空状态
uint32_t ptxfemplvl :							//周期性TxFIFO空门限periodic TxFIFO enmty level
    1;										//0: 半空状态  1: 全空状态
uint32_t Reserved9_31 :
    23;
  }
  b;
} USB_OTG_GAHBCFG_TypeDef ;

typedef union _USB_OTG_GUSBCFG_TypeDef //配置寄存器OTG_FS_GUSBCFG	0x00C
{
  uint32_t d32;
  struct
  {
uint32_t toutcal :					//FS超时校准FS timeout calibration
    3;
uint32_t Reserved3_5 :			//
    3;
uint32_t physel :						//全速系列收发器选择full speed serial transceiver select
    1;
uint32_t Reserved7 :				//
    1;
uint32_t srpcap :						//SRP使能SRP-capable
    1;						//0: 不使能SRP功能	1: 使能SRP功能
uint32_t hnpcap :						//HNP使能HNP-capable
    1;						//0: 不使能HNP功能  1: 使能HNP功能
uint32_t usbtrdtim :				//USB周转时间USB turnaround time
    4;
uint32_t Reserved14 :				//
    1;
uint32_t phylpwrclksel :
    1;
uint32_t Reserved16 :
    1;
uint32_t ulpi_fsls :
    1;
uint32_t ulpi_auto_res :
    1;
uint32_t ulpi_clk_sus_m :
    1;
uint32_t ulpi_ext_vbus_drv :
    1;
uint32_t ulpi_int_vbus_ind :
    1;
uint32_t term_sel_dl_pulse :
    1;
uint32_t ulpi_ind_cpl :
    1;
uint32_t ulpi_passthrough :
    1;       
uint32_t ulpi_protect_disable :
    1; 
uint32_t Reserved26_28 :	//
    3;     
uint32_t force_host :			//强制主机模式force host mode
1;								//0: 正常模式  1: 强制主机模式
uint32_t force_dev :			//强制设备模式force device mode 
1;								//0: 正常模式  1: 强制设备模式
uint32_t corrupt_tx :			//损坏的发送数据包corrupt Tx packet
    1;
  }
  b;
} USB_OTG_GUSBCFG_TypeDef ;

typedef union _USB_OTG_GRSTCTL_TypeDef //复位寄存器OTG_FS_GRSTCTL 0x010
{
  uint32_t d32;
  struct
  {
uint32_t csftrst :			//模块软复位core soft reset
    1;
uint32_t hsftrst :			//HCLK软复位HCLK soft reset
    1;
uint32_t hstfrm :				//主机帧计数器复位host frame counter reset
    1;
uint32_t Reserved3 :		//
    1;
uint32_t rxfflsh :			//RxFIFO刷新RxFIFO flush
    1;
uint32_t txfflsh :			//TxFIFO刷新TxFIFO flush
    1;
uint32_t txfnum :				//TxFIFO编号TxFIFO number
    5;
uint32_t Reserved11_29 ://
    19;
uint32_t dmareq :				//
    1;
uint32_t ahbidle :			//AHB主器件空闲AHB master idle
    1;						//0: 主器件不空闲  1: 主器件空闲
  }
  b;
} USB_OTG_GRSTCTL_TypeDef ;

typedef union _USB_OTG_GINTMSK_TypeDef //中断屏蔽寄存器OTG_FS_GINTMSK	0x18
{
  uint32_t d32;
  struct
  {
uint32_t Reserved0 :
    1;
uint32_t modemismatch :	//模式不匹配中断屏蔽mode mismatch interrupt mask
    1;								//0: 屏蔽中断 1: 使能中断
uint32_t otgintr :			//OTG中断屏蔽OTG interrupt mask 
    1;
uint32_t sofintr :			//帧起始中断屏蔽start of frame mask
    1;
uint32_t rxstsqlvl :		//接收FIFO非中断屏蔽receive FIFO nonempty mask
    1;
uint32_t nptxfempty :		//非周期性FIFO非空中断屏蔽non-periodic TxFIFO empty mask
    1;
uint32_t ginnakeff :		//全局非周期性IN NAK 生效中断屏蔽Global non-periodic IN NAK effective mask
    1;
uint32_t goutnakeff :		//全局OUT NAK 生效中断屏蔽global OUT NAK effective mask
    1;
uint32_t Reserved8_9 :	//保留
    2;
uint32_t erlysuspend :	//早期挂起中断屏蔽early suspend mask
    1;
uint32_t usbsuspend :		//USB挂起中断屏蔽USB suspend mask
    1;
uint32_t usbreset :			//USB 复位中断屏蔽USB reset mask
    1;
uint32_t enumdone :			//枚举完成中断屏蔽enumeration done mask
    1;
uint32_t isooutdrop :		//丢弃同步OUT数据包中断屏蔽 isochronous OUT packet dropped interrupt mask
    1;
uint32_t eopframe :			//周期性帧结束中断屏蔽end of periodic frame interrupt mask
    1;
uint32_t Reserved16 :		
    1;
uint32_t epmismatch :		//端点不匹配中断屏蔽endpoint mismatch interrupt mask
    1;
uint32_t inepintr :			//IN 端点中断屏蔽IN endpoint interrupt mask
    1;
uint32_t outepintr :		//OUT端点中断屏蔽OUT endpoint interrupt mask
    1;
uint32_t incomplisoin :	//未完成IN同步传输中断屏蔽incomplete isochronous IN transfer mask 
    1;
uint32_t incomplisoout ://(设备)未完成OUT同步传输中断屏蔽incomplete isochronousOUT transfer mask; （主机）未完成周期性传输中断屏蔽incomplete periodic transfer mask
    1;
uint32_t Reserved22_23 ://保留
    2;
uint32_t portintr :			//主机端口中断屏蔽host port interrupt mask
    1;
uint32_t hcintr :				//主机通道中断屏蔽host port interrupt mask	
    1;
uint32_t ptxfempty :		//周期性TxFIFO空屏蔽periodic TxFIFO empty mask
    1;
uint32_t Reserved27 :		//
    1;
uint32_t conidstschng :	//连接器ID状态更改屏蔽connector ID status change mask
    1;
uint32_t disconnect :		//检测到断开连接中断屏蔽disconnect detected interrupt mask
    1;
uint32_t sessreqintr :	//检测到会话请求/新会话中断屏蔽session request/new session detected interrupt mask
    1;
uint32_t wkupintr :			//检测到恢复/远程唤醒中断屏蔽resume/remote wakeup detected interrupt mask
    1;
  }
  b;
} USB_OTG_GINTMSK_TypeDef ;
typedef union _USB_OTG_GINTSTS_TypeDef //模块中断寄存器OTG_FS_GINTSTS 0x014
{
  uint32_t d32;
  struct
  {
uint32_t curmode :				//当前工作模式current mode of operation
    1;								//0: 设备模式	1: 主机模式
uint32_t modemismatch :		//模式不匹配中断mode mismatch interrupt
    1;
uint32_t otgintr :				//OTG中断OTG interrupt
    1;
uint32_t sofintr :				//帧起始start of frame
    1;
uint32_t rxstsqlvl :			//RxFIFO非空RxFIFO non-empty
    1;
uint32_t nptxfempty :			//非周期性TxFIFO 空non-periodic TxFIFO empty
    1;
uint32_t ginnakeff :			//全局非周期性IN NAK 有效global IN nonperiodic NAK effective
    1;
uint32_t goutnakeff :			//全局OUT NAK 有效gloabl OUT NAK effective
    1;
uint32_t Reserved8_9 :		//
    2;
uint32_t erlysuspend :		//早期挂起early suspend
    1;
uint32_t usbsuspend :			//USB挂起USB suspend
    1;
uint32_t usbreset :				//USB复位USB reset
    1;
uint32_t enumdone :				//枚举完成enumeration done
    1;
uint32_t isooutdrop :			//丢弃同步OUT 数据包中断isochronous OUT packet dropped interrupt
    1;
uint32_t eopframe :				//周期性帧结束中断end od periodic frame interrupt
    1;
uint32_t Reserved16_17 :	//
    2;
uint32_t inepint:					//IN 端点中断IN endpoint interrupt
    1;
uint32_t outepintr :			//OUT端点中断OUT endpoint interrupt
    1;
uint32_t incomplisoin :		//未完成IN同步传输incomplete isochronous IN transfer
    1;
uint32_t incomplisoout :	//未完成周期性传输incomplete periodic transfer
    1;
uint32_t Reserved22_23 :	//
    2;
uint32_t portintr :				//主机端口中断host port interrupt 
    1;
uint32_t hcintr :					//主机通道中断host channels interrupt
    1;
uint32_t ptxfempty :			//周期性TxFIFO空periodic TxFIFO empty
    1;
uint32_t Reserved27 :			//
    1;
uint32_t conidstschng :		//连接器ID线状态更改connector ID status change
    1;
uint32_t disconnect :			//检测到断开连接中断disconnect detected interrupt
    1;
uint32_t sessreqintr :		//检测到会话请求/新会话中断session request/new session detected interrupt
    1;
uint32_t wkupintr :				//检测到恢复/远程唤醒中断resume/remote wakeup detected interrupt
    1;
  }
  b;
} USB_OTG_GINTSTS_TypeDef ;

typedef union _USB_OTG_DRXSTS_TypeDef 	//OTG接收状态调试读取/OTG状态读取和出栈寄存器（设备模式）读取偏移地址：0x01C		出栈偏移地址：0x020
{
  uint32_t d32;
  struct
  {
uint32_t epnum :		//端点编号endpoint number
    4;
uint32_t bcnt :			//字节计数byte count
    11;
uint32_t dpid :			//数据PID data PID
    2;
uint32_t pktsts :		//数据包状态packet status
    4;
uint32_t fn :				//帧编号frame number
    4;
uint32_t Reserved :	//
    7;
  }
  b;
} USB_OTG_DRXSTS_TypeDef ;

typedef union _USB_OTG_GRXSTS_TypeDef //OTG接收状态调试读取/OTG状态读取和出栈寄存器（主机模式）读取偏移地址：0x01C		出栈偏移地址：0x020
{
  uint32_t d32;
  struct
  {
uint32_t chnum :		//通道编号channel number
    4;
uint32_t bcnt :			//字节计数byte count
    11;
uint32_t dpid :			//数据PID  data PID
    2;
uint32_t pktsts :		//数据包状态packet status
    4;
uint32_t Reserved :	//
    11;
  }
  b;
} USB_OTG_GRXFSTS_TypeDef ;

typedef union _USB_OTG_FSIZ_TypeDef //OTG_FS接收FIFO大小寄存器OTG_FS_GRXFSIZ	0x024
{
  uint32_t d32;
  struct
  {
uint32_t startaddr :
    16;
uint32_t depth :		
    16;
  }
  b;
} USB_OTG_FSIZ_TypeDef ;

typedef union _USB_OTG_HNPTXSTS_TypeDef //OTG_FS主机非周期性发送FIFO大小寄存器	0x028
{
  uint32_t d32;
  struct
  {
    uint32_t nptxfspcavail :
      16;
    uint32_t nptxqspcavail :
      8;
      struct
        {
          uint32_t terminate :
            1;
          uint32_t token :
            2;
          uint32_t chnum :
            4; 
         } nptxqtop;
     uint32_t Reserved :
        1;
  }
  b;
} USB_OTG_HNPTXSTS_TypeDef ;

typedef union _USB_OTG_DTXFSTSn_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t txfspcavail :
    16;
uint32_t Reserved :
    16;
  }
  b;
} USB_OTG_DTXFSTSn_TypeDef ;

typedef union _USB_OTG_GCCFG_TypeDef //通用模块配置寄存器OTG_FS_GCCFG	0x038
{
  uint32_t d32;
  struct
  {
uint32_t Reserved_in :					//
    16;
uint32_t pwdn :									//掉电power down
    1;
uint32_t Reserved_17 :					//
    1;
uint32_t vbussensingA :					//使能A器件的Vbus感应功能enable the Vbus sensing A device
    1;						//0: 禁止"A"器件的VBUS感应功能  1: 使能"A"器件的VBUS感应功能
uint32_t vbussensingB :					//使能B器件的Vbus感应功能enable the Vbus sensing B device
    1;						//0: 禁止"B"器件的VBUS感应功能  1: 使能"B"器件的VBUS感应功能
uint32_t sofouten :							//SOF输出使能SOF output enable
    1;						//0: SOF脉冲不从引脚输出  1: SOF脉冲可从引脚输出
uint32_t disablevbussensing :		//Vbus 感应禁止选项Vbus sensing disable option
    1;						//0: 硬件支持VBUS感应  1: 硬件不支持VBUS感应
uint32_t Reserved_out :					//
    10;
  }
  b;
} USB_OTG_GCCFG_TypeDef ;

typedef union _USB_OTG_DCFG_TypeDef //设备配置寄存器OTG_FS_DCFG0x800
{
  uint32_t d32;
  struct
  {
uint32_t devspd :					//设备速度
    2;
uint32_t nzstsouthshk :		//非零长度状态OUT握手信号non-zero-length status OUT handshake
    1;
uint32_t Reserved3 :			//
    1;
uint32_t devaddr :				//设备地址device address
    7;
uint32_t perfrint :				//周期性帧间隔periodic frame interval
    2;
uint32_t Reserved13_31 :	//
    19;
  }
  b;
} USB_OTG_DCFG_TypeDef ;

typedef union _USB_OTG_DCTL_TypeDef //设备控制寄存器OTG_FS_DCTL 0x804
{
  uint32_t d32;
  struct
  {
uint32_t rmtwkupsig :		//发送远程唤醒信号remote wakeup signaling
    1;
uint32_t sftdiscon :		//软断开soft disconnect
    1;						//0: 正常工作	1: 使主机收到设备断开连接的事件
uint32_t gnpinnaksts :	//全局IN NAK状态global IN NAK status
    1;
uint32_t goutnaksts :		//全局OUT NAK 状态global OUT NAK status
    1;
uint32_t tstctl :				//测试控制test control
    3;
uint32_t sgnpinnak :		//将全局IN NAK 置1 set global IN NAK
    1;
uint32_t cgnpinnak :		//将全局IN NAK 清零clear global IN NAK
    1;
uint32_t sgoutnak :			//将全局OUT NAK置1 set global OUT NAK
    1;
uint32_t cgoutnak :			//将全局OUT NAK清零clear global OUT NAK
    1;
uint32_t poprg_done :		//上电编程完成power-on programming done
    1;    
uint32_t Reserved :			//
    20;
  }
  b;
} USB_OTG_DCTL_TypeDef ;

typedef union _USB_OTG_DSTS_TypeDef //设备状态寄存器 0x808
{
  uint32_t d32;
  struct
  {
uint32_t suspsts :			//挂起状态suspend status
    1;
uint32_t enumspd :			//枚举速度enumerated speed
    2;
uint32_t errticerr :		//不定错误erratic error
    1;
uint32_t Reserved4_7:		//
    4;
uint32_t soffn :				//接收SOF的帧编号frame number of the received SOF
    14;
uint32_t Reserved22_31 ://
    10;
  }
  b;
} USB_OTG_DSTS_TypeDef ;

typedef union _USB_OTG_DIEPINTn_TypeDef //OTG_FS设备IN端点通用中断屏蔽寄存器 0x810
{
  uint32_t d32;
  struct
  {
uint32_t xfercompl :		//传输完成中断屏蔽 transfer completed interrupt mask
    1;
uint32_t epdisabled :		//端点禁止中断屏蔽	endpoint disabled interrupt mask
    1;
uint32_t Reserved2 :		//
    1;
uint32_t timeout :			//超时中断屏蔽	timeout condition mask 
    1;
uint32_t intktxfemp :		//TxFIFO为空时接收到IN令牌中断屏蔽
    1;
uint32_t Reserved5 :		//
    1;
uint32_t inepnakeff :
    1;
uint32_t emptyintr :
    1;
uint32_t txfifoundrn :
    1;
uint32_t Reserved14_31 :
    23;
  }
  b;
} USB_OTG_DIEPINTn_TypeDef ;
typedef union _USB_OTG_DIEPINTn_TypeDef   USB_OTG_DIEPMSK_TypeDef ;

typedef union _USB_OTG_DOEPINTn_TypeDef //设备OUT端点通用中断屏蔽寄存器 0x814
{
  uint32_t d32;
  struct
  {
uint32_t xfercompl :		//传输完成中断屏蔽transfer completed interrupt mask
    1;
uint32_t epdisabled :		//端点禁止中断屏蔽endpoint disabled interrupt mask
    1;
uint32_t Reserved2 :		//
    1;
uint32_t setup :				//SETUP阶段完成中断屏蔽SETUP phase done mask
    1;
uint32_t Reserved04_31 :
    28;
  }
  b;
} USB_OTG_DOEPINTn_TypeDef ;
typedef union _USB_OTG_DOEPINTn_TypeDef   USB_OTG_DOEPMSK_TypeDef ;

typedef union _USB_OTG_DAINT_TypeDef //设备全体端点中断寄存器0x818
{
  uint32_t d32;
  struct
  {
uint32_t in :		//IN端点中断位IN endpoint interrupt bits
    16;
uint32_t out :	//OUT端点中断位OUT endpoint interrrupt bits
    16;
  }
  ep;
} USB_OTG_DAINT_TypeDef ;

typedef union _USB_OTG_DTHRCTL_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t non_iso_thr_en :
    1;
uint32_t iso_thr_en :
    1;
uint32_t tx_thr_len :
    9;
uint32_t Reserved11_15 :
    5;
uint32_t rx_thr_en :
    1;
uint32_t rx_thr_len :
    9;
uint32_t Reserved26 : 
    1;
uint32_t arp_en :  
    1;
uint32_t Reserved28_31 :
    4;   
  }
  b;
} USB_OTG_DTHRCTL_TypeDef ;
typedef union _USB_OTG_DEPCTL_TypeDef //设备端点x控制寄存器 0xB00 + (端点编号 x 0x20)
{
  uint32_t d32;
  struct
  {
uint32_t mps :				//最大数据包大小maximum packet size 
    11;
uint32_t reserved :		//
    4;
uint32_t usbactep :		//USB活动端点USB active endpoint
    1;
uint32_t dpid :				//端点数据PID endpoint data PID
    1;							//0:DATA0  1:DATA1
uint32_t naksts :			//NAK 状态NAK status
    1;
uint32_t eptype :			//端点类型endpoint type
    2;
uint32_t snp :				//监听模式snoop mode
    1;
uint32_t stall :			//STALL握手STALL handshake
    1;
uint32_t txfnum :			//TxFIFO编号TxFIFO number
    4;
uint32_t cnak :				//将NAK清零clear NAK
    1;
uint32_t snak :				//将NAK置1 set NAK
    1;
uint32_t setd0pid :		//设置DATA0PID
    1;
uint32_t setd1pid :		//设置DATA1PID
    1;
uint32_t epdis :			//端点禁止endpoint disable
    1;
uint32_t epena :			//端点使能endpoint enable,该位置1将启动数据发送
    1;
  }
  b;
} USB_OTG_DEPCTL_TypeDef ;
typedef union _USB_OTG_DEPXFRSIZ_TypeDef //设备端点x传输大小寄存器 0x910 + (端点编号 x 0x20)
{
  uint32_t d32;
  struct
  {
uint32_t xfersize :	//传输大小transfer size
    19;
uint32_t pktcnt :		//数据包计数packet count
    10;
uint32_t mc :				//多重计数multi count
    2;
uint32_t Reserved :
    1;
  }
  b;
} USB_OTG_DEPXFRSIZ_TypeDef ;
typedef union _USB_OTG_DEP0XFRSIZ_TypeDef //设备端点0传输大小寄存器 0xB10
{
  uint32_t d32;
  struct
  {
uint32_t xfersize :				//传输大小transfer size
    7;
uint32_t Reserved7_18 :
    12;
uint32_t pktcnt :					//数据包计数
    2;
uint32_t Reserved20_28 :
    9;
uint32_t supcnt :					//STEUP数据包计数SETUP packet count
    2;								//01:1个数据包 10:2个数据包 11:3个数据包
    uint32_t Reserved31;
  }
  b;
} USB_OTG_DEP0XFRSIZ_TypeDef ;

typedef union _USB_OTG_HCFG_TypeDef //主机配置寄存器 0x400
{
  uint32_t d32;
  struct
  {
uint32_t fslspclksel :	//FS/LS PHY时钟选择FS/LS PHY clock select
    2;
uint32_t fslssupp :			//仅支持FS和LS  FS- and LS-only support
    1;
  }
  b;
} USB_OTG_HCFG_TypeDef ;

typedef union _USB_OTG_HFRMINTRVL_TypeDef //主机帧时间间隔寄存器OTG_FS_HFIR 0x404
{
  uint32_t d32;
  struct
  {
uint32_t frint :		//帧间隔 frame interval
    16;
uint32_t Reserved :
    16;
  }
  b;
} USB_OTG_HFRMINTRVL_TypeDef ;

typedef union _USB_OTG_HFNUM_TypeDef //主机帧编号/帧剩余时间寄存器 0x408
{
  uint32_t d32;
  struct
  {
uint32_t frnum :	//帧编号frame number
    16;
uint32_t frrem :	//帧剩余时间	frame time remaining
    16;
  }
  b;
} USB_OTG_HFNUM_TypeDef ;
typedef union _USB_OTG_HPTXSTS_TypeDef //主机周期性发送FIFO/队列状态寄存器 0x410
{
  uint32_t d32;
  struct
  {
uint32_t ptxfspcavail :			//周期性发送数据FIFO 可用空间periodic transfer data FIFO space available
    16;
uint32_t ptxqspcavail :			//周期性发送请求队列可用空间periodic transmit request queue space available
    8;
      struct
        {
          uint32_t terminate :	//结束(所选通道/端点的最后一个条目)terminate(last entry for the selected channel/endpoint)
            1;
          uint32_t token :			//类型type
						2;							//00: 输入/输出 01: 零长度数据包 11: 禁止通道命令
          uint32_t chnum :			//通道/端点编号channel/endpoint number
            4; 
          uint32_t odd_even :		//奇数/偶数帧odd/even frame
						1;            	//0: 以偶数帧发送 1: 以奇数帧发送
         } ptxqtop;  				// 周期性发送请求队列顶部top of the periodic transmit request queue
  }
  b;
} USB_OTG_HPTXSTS_TypeDef ;
typedef union _USB_OTG_HPRT0_TypeDef //主机端口控制和状态寄存器 0x440
{
  uint32_t d32;
  struct
  {
uint32_t prtconnsts :			//端口连接状态port connect status
    1;
uint32_t prtconndet :			//检测到端口连接port connect detected
    1;
uint32_t prtena :					//端口使能port enable
    1;
uint32_t prtenchng :			//端口使能/禁止变化port enable/disable change
    1;
uint32_t prtovrcurract :	//端口过流激活port overcurrent active
    1;
uint32_t prtovrcurrchng :	//端口过流变化port overcurrent change
    1;		
uint32_t prtres :					//端口恢复port resume
    1;
uint32_t prtsusp :				//端口挂起port suspend
    1;
uint32_t prtrst :					//端口复位port reset
    1;
uint32_t Reserved9 :			//
    1;
uint32_t prtlnsts :				//端口线状态port line status
2;										//位10:OTG_FS_FS_DP的逻辑电平  位11:OTG_FS_FS_DM的逻辑电平
uint32_t prtpwr :					//端口电源port power
    1;
uint32_t prttstctl :			//端口测试控制port test control
    4;
uint32_t prtspd :					//端口速度port speed
2;										//01:全速  10:低速  11:保留
uint32_t Reserved19_31 :	//
    13;
  }
  b;
} USB_OTG_HPRT0_TypeDef ;
typedef union _USB_OTG_HAINT_TypeDef //主机全体通道中断寄存器 0x414
{
  uint32_t d32;
  struct
  {
uint32_t chint :		//通道中断channel interrupt
    16;
uint32_t Reserved :
    16;
  }
  b;
} USB_OTG_HAINT_TypeDef ;
typedef union _USB_OTG_HAINTMSK_TypeDef //主机全体通道中断屏蔽寄存器 0x418
{
  uint32_t d32;
  struct
  {
uint32_t chint :			//通道中断屏蔽channel interrupt mask
    16;
uint32_t Reserved :
    16;
  }
  b;
} USB_OTG_HAINTMSK_TypeDef ;
typedef union _USB_OTG_HCCHAR_TypeDef //主机通道x特性寄存器0x500+（通道编号 x 0x20）
{
  uint32_t d32;
  struct
  {
uint32_t mps :				//最大数据包大小maximum packet size
    11;
uint32_t epnum :			//端点编号endpoint number
    4;
uint32_t epdir :			//端点方向endpoint direction
    1;
uint32_t Reserved :		//
    1;
uint32_t lspddev :		//低速设备low-speed device
    1;
uint32_t eptype :			//端点类型endpoint type
    2;
uint32_t multicnt :		//多重计数multicount
    2;
uint32_t devaddr :		//设备地址device address
    7;
uint32_t oddfrm :			//奇数帧odd frame
    1;
uint32_t chdis :			//通道禁止channel disable
    1;
uint32_t chen :				//通道使能channel enable
    1;
  }
  b;
} USB_OTG_HCCHAR_TypeDef ;

typedef union _USB_OTG_HCSPLT_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t prtaddr :
    7;
uint32_t hubaddr :
    7;
uint32_t xactpos :
    2;
uint32_t compsplt :
    1;
uint32_t Reserved :
    14;
uint32_t spltena :
    1;
  }
  b;
} USB_OTG_HCSPLT_TypeDef ;
typedef union _USB_OTG_HCINTn_TypeDef //主机通道x中断寄存器host channel_x interrupt register 0x508+（通道编号 x 0x20）
{
  uint32_t d32;
  struct
  {
uint32_t xfercompl :		//传输完成transfer completed
    1;
uint32_t chhltd :				//通道停止channel halted
    1;
uint32_t ahberr :				//
    1;
uint32_t stall :				//收到STALL响应STALL response received interrupt 
    1;
uint32_t nak :					//收到NA响应NAK response received interrupt
    1;
uint32_t ack :					//收到/发送ACK响应ACK response received/transmitted interrupt
    1;
uint32_t nyet :					//
    1;
uint32_t xacterr :			//通信事务错误transaction error
    1;
uint32_t bblerr :				//串扰错误babble error
    1;
uint32_t frmovrun :			//帧溢出错误frame overrun
    1;
uint32_t datatglerr :		//数据同步错误data toggle error
    1;
uint32_t Reserved :			//
    21;
  }
  b;
} USB_OTG_HCINTn_TypeDef ;
typedef union _USB_OTG_HCTSIZn_TypeDef //主机通道x传输大小寄存器0x510+（通道编号 想0x20）
{
  uint32_t d32;
  struct
  {
uint32_t xfersize :	//传输大小transfer size
    19;
uint32_t pktcnt :		//数据包计数packet count
    10;
uint32_t pid :			//数据PID data PID
    2;
uint32_t dopng :		//
    1;
  }
  b;
} USB_OTG_HCTSIZn_TypeDef ;
typedef union _USB_OTG_HCINTMSK_TypeDef //主机通道x中断屏蔽寄存器0x50C + （通道编号 x 0x20）
{
  uint32_t d32;
  struct
  {
uint32_t xfercompl :		//传输完成中断屏蔽transfer completed mask
    1;
uint32_t chhltd :				//通道停止中断屏蔽chnanel halted mask
    1;
uint32_t ahberr :				//
    1;
uint32_t stall :				//STALL 响应接收中断屏蔽STALL response received interrupt mask
    1;
uint32_t nak :					//NAK响应接收中断屏蔽NAK response received interrupt mask
    1;
uint32_t ack :					//ACK响应接收/发送中断屏蔽ACK responce received/transmitted interrupt mask
    1;
uint32_t nyet :					//NYET响应接收中断屏蔽response received interrupt mask
    1;
uint32_t xacterr :			//通信事务错误屏蔽transaction error mask
    1;
uint32_t bblerr :				//串扰错误屏蔽babble error mask
    1;
uint32_t frmovrun :			//帧溢出屏蔽frame overrun mask
    1;
uint32_t datatglerr :		//数据同步错误屏蔽data toggle error mask
    1;
uint32_t Reserved :			//
    21;
  }
  b;
} USB_OTG_HCINTMSK_TypeDef ;

typedef union _USB_OTG_PCGCCTL_TypeDef //电源和时钟门控控制寄存器0xE00
{
  uint32_t d32;
  struct
  {
uint32_t stoppclk :			//通知PHY时钟stop PHY clock
    1;
uint32_t gatehclk :			//门控HCLK gate HCLK
    1;
uint32_t Reserved2_3 :	//
    2;
uint32_t phy_susp :			//PHY挂起 PHY suspended
    1;    
uint32_t Reserved5_31 :	//
    27;
  }
  b;
} USB_OTG_PCGCCTL_TypeDef ;

/**
  * @}
  */ 


/** @defgroup USB_REGS_Exported_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USB_REGS_Exported_Variables
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USB_REGS_Exported_FunctionsPrototype
  * @{
  */ 
/**
  * @}
  */ 


#endif //__USB_OTG_REGS_H__


/**
  * @}
  */ 

/**
  * @}
  */ 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

