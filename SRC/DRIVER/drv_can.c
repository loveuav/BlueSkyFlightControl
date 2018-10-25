/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     drv_can.c
 * @说明     can驱动
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.08
**********************************************************************************************************/
#include "drv_can.h"

/**********************************************************************************************************
*函 数 名: Can_Open
*功能说明: can初始化
*形    参: can序号
*返 回 值: 无
**********************************************************************************************************/
void Can_Open(uint8_t deviceNum)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    CAN_StructInit(&can);

    can.CAN_TTCM        = DISABLE;
    can.CAN_ABOM        = DISABLE;
    can.CAN_AWUM        = DISABLE;
    can.CAN_NART        = DISABLE;
    can.CAN_RFLM        = DISABLE;
    can.CAN_TXFP        = ENABLE;
    can.CAN_Mode        = CAN_Mode_Normal;
    can.CAN_SJW         = CAN_SJW_1tq;
    can.CAN_BS1         = CAN_BS1_9tq;
    can.CAN_BS2         = CAN_BS2_4tq;
    can.CAN_Prescaler   = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps

    can_filter.CAN_FilterNumber         = 0;
    can_filter.CAN_FilterMode           = CAN_FilterMode_IdMask;
    can_filter.CAN_FilterScale          = CAN_FilterScale_32bit;
    can_filter.CAN_FilterIdHigh         = 0x0000;
    can_filter.CAN_FilterIdLow          = 0x0000;
    can_filter.CAN_FilterMaskIdHigh     = 0x0000;
    can_filter.CAN_FilterMaskIdLow      = 0x0000;
    can_filter.CAN_FilterFIFOAssignment = 0;    //the message which pass the filter save in fifo0
    can_filter.CAN_FilterActivation     = ENABLE;

    if(deviceNum == 1)
    {
        GPIO_PinAFConfig(CAN1_GPIO, CAN1_PINSOURCE_RX, GPIO_AF_CAN1);
        GPIO_PinAFConfig(CAN1_GPIO, CAN1_PINSOURCE_TX, GPIO_AF_CAN1);

        gpio.GPIO_Pin  = CAN1_PIN_RX | CAN1_PIN_TX;
        gpio.GPIO_Mode = GPIO_Mode_AF;
        GPIO_Init(CAN1_GPIO, &gpio);

        nvic.NVIC_IRQChannel                    = CAN1_RX0_IRQn;
        nvic.NVIC_IRQChannelPreemptionPriority  = 3;
        nvic.NVIC_IRQChannelSubPriority         = 0;
        nvic.NVIC_IRQChannelCmd                 = ENABLE;
        NVIC_Init(&nvic);

        nvic.NVIC_IRQChannel                    = CAN1_TX_IRQn;
        nvic.NVIC_IRQChannelPreemptionPriority  = 3;
        nvic.NVIC_IRQChannelSubPriority         = 0;
        nvic.NVIC_IRQChannelCmd                 = ENABLE;
        NVIC_Init(&nvic);

        CAN_DeInit(CAN1);
        CAN_Init(CAN1, &can);
        CAN_FilterInit(&can_filter);

        CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
        CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);
    }
    else if(deviceNum == 2)
    {
        GPIO_PinAFConfig(CAN2_GPIO, CAN2_PINSOURCE_RX, GPIO_AF_CAN2);
        GPIO_PinAFConfig(CAN2_GPIO, CAN2_PINSOURCE_TX, GPIO_AF_CAN2);

        gpio.GPIO_Pin  = CAN2_PIN_RX | CAN2_PIN_TX;
        gpio.GPIO_Mode = GPIO_Mode_AF;
        GPIO_Init(CAN2_GPIO, &gpio);

        nvic.NVIC_IRQChannel                    = CAN2_RX0_IRQn;
        nvic.NVIC_IRQChannelPreemptionPriority  = 3;
        nvic.NVIC_IRQChannelSubPriority         = 0;
        nvic.NVIC_IRQChannelCmd                 = ENABLE;
        NVIC_Init(&nvic);

        nvic.NVIC_IRQChannel                    = CAN2_TX_IRQn;
        nvic.NVIC_IRQChannelPreemptionPriority  = 3;
        nvic.NVIC_IRQChannelSubPriority         = 0;
        nvic.NVIC_IRQChannelCmd                 = ENABLE;
        NVIC_Init(&nvic);

        CAN_DeInit(CAN2);
        CAN_Init(CAN2, &can);
        CAN_FilterInit(&can_filter);

        CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
        CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);
    }
}

/**********************************************************************************************************
*函 数 名: CAN1_TX_IRQHandler
*功能说明: CAN1发送中断
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void CAN1_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET)
    {
        CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
    }
}

/**********************************************************************************************************
*函 数 名: CAN1_RX0_IRQHandler
*功能说明: CAN1接收中断
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg rx_message;

    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
    }
}

/**********************************************************************************************************
*函 数 名: CAN2_TX_IRQHandler
*功能说明: CAN2发送中断
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void CAN2_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET)
    {
        CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
    }
}

/**********************************************************************************************************
*函 数 名: CAN2_RX0_IRQHandler
*功能说明: CAN2接收中断
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void CAN2_RX0_IRQHandler(void)
{
    CanRxMsg rx_message;
    if(CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx_message);
    }
}

void CAN1_SendTest(void)
{
    CanTxMsg tx_message;

    tx_message.StdId = 0x085;
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.DLC = 0x08;          //帧长度

    tx_message.Data[0] = 1;
    tx_message.Data[1] = 2;
    tx_message.Data[2] = 3;
    tx_message.Data[3] = 4;

    CAN_Transmit(CAN1,&tx_message);
}

