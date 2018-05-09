/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     drv_usart.c
 * @说明     串口驱动
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "drv_usart.h"

uint8_t usartTxBuf[6][256];
uint8_t usartTxCounter[6];
uint8_t usartTxDataCnt[6];

/**********************************************************************************************************
*函 数 名: Usart_Open
*功能说明: 串口初始化
*形    参: 串口号
*返 回 值: 无
**********************************************************************************************************/
void Usart_Open(uint8_t deviceNum)
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;   

    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;   
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; 

	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable; 
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge; 
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable;
    
	if(deviceNum == 1)
	{  
        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; 
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART1_IRQ_PRIORITY;   
        NVIC_Init(&NVIC_InitStructure);	   

        GPIO_PinAFConfig(USART1_GPIO, USART1_PINSOURCE_TX, GPIO_AF_USART1);
        GPIO_PinAFConfig(USART1_GPIO, USART1_PINSOURCE_RX, GPIO_AF_USART1);
        
        GPIO_InitStructure.GPIO_Pin =  USART1_PIN_TX; 
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(USART1_GPIO, &GPIO_InitStructure); 
        GPIO_InitStructure.GPIO_Pin =  USART1_PIN_RX; 
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(USART1_GPIO, &GPIO_InitStructure); 
        
        USART_InitStructure.USART_BaudRate = USART1_BAUDRATE; 
        USART_Init(USART1, &USART_InitStructure);
        USART_ClockInit(USART1, &USART_ClockInitStruct);
        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
        USART_Cmd(USART1, ENABLE);
    }
	else if(deviceNum == 2)
	{  
        NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; 
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART2_IRQ_PRIORITY;   
        NVIC_Init(&NVIC_InitStructure);	   

        GPIO_PinAFConfig(USART2_GPIO, USART2_PINSOURCE_TX, GPIO_AF_USART2);
        GPIO_PinAFConfig(USART2_GPIO, USART2_PINSOURCE_RX, GPIO_AF_USART2);
        
        GPIO_InitStructure.GPIO_Pin =  USART2_PIN_TX; 
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(USART2_GPIO, &GPIO_InitStructure); 
        GPIO_InitStructure.GPIO_Pin =  USART2_PIN_RX; 
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(USART2_GPIO, &GPIO_InitStructure); 
        
        USART_InitStructure.USART_BaudRate = USART2_BAUDRATE; 
        USART_Init(USART2, &USART_InitStructure);
        USART_ClockInit(USART2, &USART_ClockInitStruct);
        USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
        USART_Cmd(USART2, ENABLE);
    }    
	else if(deviceNum == 3)
	{  
        NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; 
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART3_IRQ_PRIORITY;   
        NVIC_Init(&NVIC_InitStructure);	   

        GPIO_PinAFConfig(USART3_GPIO, USART3_PINSOURCE_TX, GPIO_AF_USART3);
        GPIO_PinAFConfig(USART3_GPIO, USART3_PINSOURCE_RX, GPIO_AF_USART3);
        
        GPIO_InitStructure.GPIO_Pin =  USART3_PIN_TX; 
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(USART3_GPIO, &GPIO_InitStructure); 
        GPIO_InitStructure.GPIO_Pin =  USART3_PIN_RX; 
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(USART3_GPIO, &GPIO_InitStructure); 
        
        USART_InitStructure.USART_BaudRate = USART3_BAUDRATE; 
        USART_Init(USART3, &USART_InitStructure);
        USART_ClockInit(USART3, &USART_ClockInitStruct);
        USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
        USART_Cmd(USART3, ENABLE);
    } 
	else if(deviceNum == 4)
	{  
        NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn; 
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART1_IRQ_PRIORITY;   
        NVIC_Init(&NVIC_InitStructure);	   

        GPIO_PinAFConfig(UART4_GPIO, UART4_PINSOURCE_TX, GPIO_AF_UART4);
        GPIO_PinAFConfig(UART4_GPIO, UART4_PINSOURCE_RX, GPIO_AF_UART4);
        
        GPIO_InitStructure.GPIO_Pin =  UART4_PIN_TX; 
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(UART4_GPIO, &GPIO_InitStructure); 
        GPIO_InitStructure.GPIO_Pin =  UART4_PIN_RX; 
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(UART4_GPIO, &GPIO_InitStructure); 
        
        USART_InitStructure.USART_BaudRate = UART4_BAUDRATE; 
        USART_Init(UART4, &USART_InitStructure);
        USART_ClockInit(UART4, &USART_ClockInitStruct);
        USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
        USART_Cmd(UART4, ENABLE);
    } 
	else if(deviceNum == 5)
	{  
        NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn; 
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = UART5_IRQ_PRIORITY;   
        NVIC_Init(&NVIC_InitStructure);	   

        GPIO_PinAFConfig(UART5_GPIO, UART5_PINSOURCE_TX, GPIO_AF_UART5);
        GPIO_PinAFConfig(UART5_GPIO, UART5_PINSOURCE_RX, GPIO_AF_UART5);
        
        GPIO_InitStructure.GPIO_Pin =  UART5_PIN_TX; 
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(UART5_GPIO, &GPIO_InitStructure); 
        GPIO_InitStructure.GPIO_Pin =  UART5_PIN_RX; 
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(UART5_GPIO, &GPIO_InitStructure); 
        
        USART_InitStructure.USART_BaudRate = UART5_BAUDRATE; 
        USART_Init(UART5, &USART_InitStructure);
        USART_ClockInit(UART5, &USART_ClockInitStruct);
        USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
        USART_Cmd(UART5, ENABLE);
    }   
	else if(deviceNum == 6)
	{  
        NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn; 
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART6_IRQ_PRIORITY;   
        NVIC_Init(&NVIC_InitStructure);	   

        GPIO_PinAFConfig(USART6_GPIO, USART6_PINSOURCE_TX, GPIO_AF_USART6);
        GPIO_PinAFConfig(USART6_GPIO, USART6_PINSOURCE_RX, GPIO_AF_USART6);
        
        GPIO_InitStructure.GPIO_Pin =  USART6_PIN_TX; 
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(USART6_GPIO, &GPIO_InitStructure); 
        GPIO_InitStructure.GPIO_Pin =  USART6_PIN_RX; 
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(USART6_GPIO, &GPIO_InitStructure); 
        
        USART_InitStructure.USART_BaudRate = USART6_BAUDRATE; 
        USART_Init(USART6, &USART_InitStructure);
        USART_ClockInit(USART6, &USART_ClockInitStruct);
        USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
        USART_Cmd(USART6, ENABLE);
    }     
}

/**********************************************************************************************************
*函 数 名: IRQHandler
*功能说明: 串口中断函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
#if (configUSE_USART1 == 1)
static UsartCallback usart1CallbackFunc;

void USART1_IRQHandler(void)
{
	u8 com_data;
	
	if(USART1->SR & USART_SR_ORE)
	{
		com_data = USART1->DR;
	}

	if( USART_GetITStatus(USART1,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		com_data = USART1->DR;
        if(usart1CallbackFunc != 0)
            (*usart1CallbackFunc)(com_data);
	}

	if( USART_GetITStatus(USART1,USART_IT_TXE ) )
	{				
		USART1->DR = usartTxBuf[0][usartTxCounter[0]++];       
		if(usartTxCounter[0] == usartTxDataCnt[0])
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;	
		}
	}
}
#endif

#if (configUSE_USART2 == 1)
static UsartCallback usart2CallbackFunc;

void USART2_IRQHandler(void)
{
	u8 com_data;
	
	if(USART2->SR & USART_SR_ORE)
	{
		com_data = USART2->DR;
	}

	if( USART_GetITStatus(USART2,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		com_data = USART2->DR;
        if(usart2CallbackFunc != 0)
            (*usart2CallbackFunc)(com_data);
	}

	if( USART_GetITStatus(USART2,USART_IT_TXE ) )
	{				
		USART2->DR = usartTxBuf[1][usartTxCounter[1]++];       
		if(usartTxCounter[1] == usartTxDataCnt[1])
		{
			USART2->CR1 &= ~USART_CR1_TXEIE;	
		}
	}
}
#endif

#if (configUSE_USART3 == 1)
static UsartCallback usart3CallbackFunc;

void USART3_IRQHandler(void)
{
	u8 com_data;
	
	if(USART3->SR & USART_SR_ORE)
	{
		com_data = USART3->DR;
	}

	if( USART_GetITStatus(USART3,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);
		com_data = USART3->DR;
        if(usart3CallbackFunc != 0)
            (*usart3CallbackFunc)(com_data);
	}

	if( USART_GetITStatus(USART3,USART_IT_TXE ) )
	{				
		USART3->DR = usartTxBuf[2][usartTxCounter[2]++];       
		if(usartTxCounter[2] == usartTxDataCnt[2])
		{
			USART3->CR1 &= ~USART_CR1_TXEIE;	
		}
	}
}
#endif

#if (configUSE_UART4 == 1)
static UsartCallback uart4CallbackFunc;

void UART4_IRQHandler(void)
{
	u8 com_data;
	
	if(UART4->SR & USART_SR_ORE)
	{
		com_data = UART4->DR;
	}

	if( USART_GetITStatus(UART4,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART4,USART_IT_RXNE);
		com_data = UART4->DR;
        if(uart4CallbackFunc != 0)
            (*uart4CallbackFunc)(com_data);
	}

	if( USART_GetITStatus(UART4,USART_IT_TXE ) )
	{				
		UART4->DR = usartTxBuf[3][usartTxCounter[3]++];       
		if(usartTxCounter[3] == usartTxDataCnt[3])
		{
			UART4->CR1 &= ~USART_CR1_TXEIE;	
		}
	}
}
#endif

#if (configUSE_UART5 == 1)
static UsartCallback uart5CallbackFunc;

void UART5_IRQHandler(void)
{
	u8 com_data;
	
	if(UART5->SR & USART_SR_ORE)
	{
		com_data = UART5->DR;
	}

	if( USART_GetITStatus(UART5,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART5,USART_IT_RXNE);
		com_data = UART5->DR;
        if(uart5CallbackFunc != 0)
            (*uart5CallbackFunc)(com_data);
	}

	if( USART_GetITStatus(UART5,USART_IT_TXE ) )
	{				
		UART5->DR = usartTxBuf[4][usartTxCounter[4]++];       
		if(usartTxCounter[4] == usartTxDataCnt[4])
		{
			UART5->CR1 &= ~USART_CR1_TXEIE;	
		}
	}
}
#endif


#if (configUSE_USART6 == 1)
static UsartCallback usart6CallbackFunc;

void USART6_IRQHandler(void)
{
	u8 com_data;
	
	if(USART6->SR & USART_SR_ORE)
	{
		com_data = USART6->DR;
	}

	if( USART_GetITStatus(USART6,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART6,USART_IT_RXNE);
		com_data = USART6->DR;
        if(usart6CallbackFunc != 0)
            (*usart6CallbackFunc)(com_data);
	}

	if( USART_GetITStatus(USART6,USART_IT_TXE ) )
	{				
		USART6->DR = usartTxBuf[5][usartTxCounter[5]++];       
		if(usartTxCounter[5] == usartTxDataCnt[5])
		{
			USART6->CR1 &= ~USART_CR1_TXEIE;	
		}
	}
}
#endif

/**********************************************************************************************************
*函 数 名: Usart_SetIRQCallback
*功能说明: 设置串口接收中断回调函数
*形    参: 串口号 回调函数
*返 回 值: 无
**********************************************************************************************************/
void Usart_SetIRQCallback(uint8_t deviceNum, UsartCallback usartCallback)
{
    if(deviceNum == 1)
    {
        #if (configUSE_USART1 == 1)
        usart1CallbackFunc = usartCallback;
        #endif
    }
	else if(deviceNum == 2)
    {
        #if (configUSE_USART2 == 1)
        usart2CallbackFunc = usartCallback;
        #endif
    }
	else if(deviceNum == 3)
    {
        #if (configUSE_USART3 == 1)
        usart3CallbackFunc = usartCallback;
        #endif
    }
	else if(deviceNum == 4)
    {
        #if (configUSE_UART4 == 1)
        uart4CallbackFunc = usartCallback;
        #endif
    }
	else if(deviceNum == 5)
    {
        #if (configUSE_UART5 == 1)
        uart5CallbackFunc = usartCallback;
        #endif
    }
	else if(deviceNum == 6)
    {
        #if (configUSE_USART6 == 1)
        usart6CallbackFunc = usartCallback;
        #endif
    }
}

/**********************************************************************************************************
*函 数 名: Usart_SendData
*功能说明: 串口数据发送函数
*形    参: 串口号 发送数据缓冲区指针 数据长度
*返 回 值: 无
**********************************************************************************************************/
void Usart_SendData(uint8_t deviceNum, uint8_t *DataToSend ,uint8_t length)
{
    uint8_t i;
    
    if(deviceNum == 1)
    {
        for(i=0;i<length;i++)
        {
            usartTxBuf[0][usartTxDataCnt[0]++] = *(DataToSend+i);
        }

        if(!(USART1->CR1 & USART_CR1_TXEIE))
        {
            USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
        }
    }
    else if(deviceNum == 2)
    {
        for(i=0;i<length;i++)
        {
            usartTxBuf[1][usartTxDataCnt[1]++] = *(DataToSend+i);
        }

        if(!(USART2->CR1 & USART_CR1_TXEIE))
        {
            USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
        }
    }
    else if(deviceNum == 3)
    {
        for(i=0;i<length;i++)
        {
            usartTxBuf[2][usartTxDataCnt[2]++] = *(DataToSend+i);
        }

        if(!(USART3->CR1 & USART_CR1_TXEIE))
        {
            USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
        }
    }
    else if(deviceNum == 4)
    {
        for(i=0;i<length;i++)
        {
            usartTxBuf[3][usartTxDataCnt[3]++] = *(DataToSend+i);
        }

        if(!(UART4->CR1 & USART_CR1_TXEIE))
        {
            USART_ITConfig(UART4, USART_IT_TXE, ENABLE);
        }
    }
    else if(deviceNum == 5)
    {
        for(i=0;i<length;i++)
        {
            usartTxBuf[4][usartTxDataCnt[4]++] = *(DataToSend+i);
        }

        if(!(UART5->CR1 & USART_CR1_TXEIE))
        {
            USART_ITConfig(UART5, USART_IT_TXE, ENABLE);
        }
    }
    else if(deviceNum == 6)
    {
        for(i=0;i<length;i++)
        {
            usartTxBuf[5][usartTxDataCnt[5]++] = *(DataToSend+i);
        }

        if(!(USART6->CR1 & USART_CR1_TXEIE))
        {
            USART_ITConfig(USART6, USART_IT_TXE, ENABLE);
        }
    }   
}



