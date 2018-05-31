/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     drv_ppm.c
 * @说明     PPM信号输入捕获
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "drv_ppm.h"

/**********************************************************************************************************
*函 数 名: PPM_Init
*功能说明: PPM定时器配置初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void PPM_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef  TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    uint16_t PrescalerValue = 0;

	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
 
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;

    #if(TIM1_PPM_IN == 1)
        PrescalerValue =  (uint16_t) ((SystemCoreClock) / TIM1_CLOCK) - 1;
        TIM_TimeBaseStructure.TIM_Period = TIM1_PERIOD;		
        TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;	    
        TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 
        
        NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM1_IRQ_PRIORITY;
        NVIC_Init(&NVIC_InitStructure); 
        
        #if(configUSE_TIM1_CH1 == 1)
            GPIO_InitStructure.GPIO_Pin = TIM1_CH1_PIN;
            GPIO_Init(TIM1_CH1_GPIO, &GPIO_InitStructure); 
            GPIO_PinAFConfig(TIM1_CH1_GPIO, TIM1_CH1_PINSOURCE, GPIO_AF_TIM1);
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
            TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);   
        #endif
        #if(configUSE_TIM1_CH2 == 1)
            GPIO_InitStructure.GPIO_Pin = TIM1_CH2_PIN;
            GPIO_Init(TIM1_CH2_GPIO, &GPIO_InitStructure); 
            GPIO_PinAFConfig(TIM1_CH2_GPIO, TIM1_CH2_PINSOURCE, GPIO_AF_TIM1);
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
            TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE);  
        #endif
        #if(configUSE_TIM1_CH3 == 1)
            GPIO_InitStructure.GPIO_Pin = TIM1_CH3_PIN;
            GPIO_Init(TIM1_CH3_GPIO, &GPIO_InitStructure); 
            GPIO_PinAFConfig(TIM1_CH3_GPIO, TIM1_CH3_PINSOURCE, GPIO_AF_TIM1);
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
            TIM_ITConfig(TIM1, TIM_IT_CC3, ENABLE);  
        #endif
        #if(configUSE_TIM1_CH4 == 1)
            GPIO_InitStructure.GPIO_Pin = TIM1_CH4_PIN;
            GPIO_Init(TIM1_CH4_GPIO, &GPIO_InitStructure); 
            GPIO_PinAFConfig(TIM1_CH4_GPIO, TIM1_CH4_PINSOURCE, GPIO_AF_TIM1);
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
            TIM_ITConfig(TIM1, TIM_IT_CC4, ENABLE);  
        #endif
        TIM_ICInit(TIM1, &TIM_ICInitStructure);
        TIM_Cmd(TIM1, ENABLE);
    #endif    
    
    #if(TIM2_PPM_IN == 1)
        PrescalerValue =  (uint16_t) ((SystemCoreClock) / 2 / TIM2_CLOCK) - 1;
        TIM_TimeBaseStructure.TIM_Period = TIM2_PERIOD;		
        TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;	    
        TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
        
        NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM2_IRQ_PRIORITY;
        NVIC_Init(&NVIC_InitStructure); 
        
        #if(configUSE_TIM2_CH1 == 1)
            GPIO_InitStructure.GPIO_Pin = TIM2_CH1_PIN;
            GPIO_Init(TIM2_CH1_GPIO, &GPIO_InitStructure); 
            GPIO_PinAFConfig(TIM2_CH1_GPIO, TIM2_CH1_PINSOURCE, GPIO_AF_TIM2);
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
            TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);   
        #endif
        #if(configUSE_TIM2_CH2 == 1)
            GPIO_InitStructure.GPIO_Pin = TIM2_CH2_PIN;
            GPIO_Init(TIM2_CH2_GPIO, &GPIO_InitStructure); 
            GPIO_PinAFConfig(TIM2_CH2_GPIO, TIM2_CH2_PINSOURCE, GPIO_AF_TIM2);
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
            TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);  
        #endif
        #if(configUSE_TIM2_CH3 == 1)
            GPIO_InitStructure.GPIO_Pin = TIM2_CH3_PIN;
            GPIO_Init(TIM2_CH3_GPIO, &GPIO_InitStructure); 
            GPIO_PinAFConfig(TIM2_CH3_GPIO, TIM3_CH3_PINSOURCE, GPIO_AF_TIM2);
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
            TIM_ITConfig(TIM1, TIM_IT_CC3, ENABLE);  
        #endif
        #if(configUSE_TIM2_CH4 == 1)
            GPIO_InitStructure.GPIO_Pin = TIM1_CH4_PIN;
            GPIO_Init(TIM2_CH4_GPIO, &GPIO_InitStructure); 
            GPIO_PinAFConfig(TIM2_CH4_GPIO, TIM1_CH4_PINSOURCE, GPIO_AF_TIM1);
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
            TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);  
        #endif
        TIM_ICInit(TIM2, &TIM_ICInitStructure);
        TIM_Cmd(TIM2, ENABLE);
    #endif    

    #if(TIM3_PPM_IN== 1)
        PrescalerValue =  (uint16_t) ((SystemCoreClock) / 2 / TIM3_CLOCK) - 1;
        TIM_TimeBaseStructure.TIM_Period = TIM3_PERIOD;		
        TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;	    
        TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
        
        NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM3_IRQ_PRIORITY;
        NVIC_Init(&NVIC_InitStructure); 
        
        #if(configUSE_TIM3_CH1 == 1)
            GPIO_InitStructure.GPIO_Pin = TIM3_CH1_PIN;
            GPIO_Init(TIM3_CH1_GPIO, &GPIO_InitStructure); 
            GPIO_PinAFConfig(TIM3_CH1_GPIO, TIM3_CH1_PINSOURCE, GPIO_AF_TIM3);
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
            TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);   
        #endif
        #if(configUSE_TIM3_CH2 == 1)
            GPIO_InitStructure.GPIO_Pin = TIM3_CH2_PIN;
            GPIO_Init(TIM3_CH2_GPIO, &GPIO_InitStructure); 
            GPIO_PinAFConfig(TIM3_CH2_GPIO, TIM3_CH2_PINSOURCE, GPIO_AF_TIM3);
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
            TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);  
        #endif
        #if(configUSE_TIM3_CH3 == 1)
            GPIO_InitStructure.GPIO_Pin = TIM3_CH3_PIN;
            GPIO_Init(TIM3_CH3_GPIO, &GPIO_InitStructure); 
            GPIO_PinAFConfig(TIM3_CH3_GPIO, TIM3_CH3_PINSOURCE, GPIO_AF_TIM3);
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
            TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);  
        #endif
        #if(configUSE_TIM3_CH4 == 1)
            GPIO_InitStructure.GPIO_Pin = TIM3_CH4_PIN;
            GPIO_Init(TIM3_CH4_GPIO, &GPIO_InitStructure); 
            GPIO_PinAFConfig(TIM3_CH4_GPIO, TIM3_CH4_PINSOURCE, GPIO_AF_TIM3);
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
            TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);  
        #endif
        TIM_ICInit(TIM3, &TIM_ICInitStructure);
        TIM_Cmd(TIM3, ENABLE);
    #endif    
    
     #if(TIM4_PPM_IN == 1)
        PrescalerValue =  (uint16_t) ((SystemCoreClock) / 2 / TIM4_CLOCK) - 1;
        TIM_TimeBaseStructure.TIM_Period = TIM4_PERIOD;		
        TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;	    
        TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
        
        NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM4_IRQ_PRIORITY;
        NVIC_Init(&NVIC_InitStructure); 
        
        #if(configUSE_TIM4_CH1 == 1)
            GPIO_InitStructure.GPIO_Pin = TIM4_CH1_PIN;
            GPIO_Init(TIM4_CH1_GPIO, &GPIO_InitStructure); 
            GPIO_PinAFConfig(TIM4_CH1_GPIO, TIM4_CH1_PINSOURCE, GPIO_AF_TIM4);
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
            TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);   
        #endif
        #if(configUSE_TIM4_CH2 == 1)
            GPIO_InitStructure.GPIO_Pin = TIM4_CH2_PIN;
            GPIO_Init(TIM4_CH2_GPIO, &GPIO_InitStructure); 
            GPIO_PinAFConfig(TIM4_CH2_GPIO, TIM4_CH2_PINSOURCE, GPIO_AF_TIM4);
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
            TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);  
        #endif
        #if(configUSE_TIM4_CH3 == 1)
            GPIO_InitStructure.GPIO_Pin = TIM4_CH3_PIN;
            GPIO_Init(TIM4_CH3_GPIO, &GPIO_InitStructure); 
            GPIO_PinAFConfig(TIM4_CH3_GPIO, TIM4_CH3_PINSOURCE, GPIO_AF_TIM4);
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
            TIM_ITConfig(TIM4, TIM_IT_CC3, ENABLE);  
        #endif
        #if(configUSE_TIM4_CH4 == 1)
            GPIO_InitStructure.GPIO_Pin = TIM4_CH4_PIN;
            GPIO_Init(TIM4_CH4_GPIO, &GPIO_InitStructure); 
            GPIO_PinAFConfig(TIM4_CH4_GPIO, TIM4_CH4_PINSOURCE, GPIO_AF_TIM4);
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
            TIM_ITConfig(TIM4, TIM_IT_CC4, ENABLE);  
        #endif
        TIM_ICInit(TIM4, &TIM_ICInitStructure);
        TIM_Cmd(TIM4, ENABLE);
    #endif    
}

#if(TIM1_PPM_IN == 1)
void TIM1_CC_IRQHandler(void)
{
    
}
#endif

#if(TIM2_PPM_IN == 1)
void TIM2_IRQHandler(void)	
{
    
}
#endif

#if(TIM3_PPM_IN == 1)
void TIM3_IRQHandler(void)	
{
	static u16 temp_cnt1,temp_cnt1_2;
	static uint16_t pulseHigh;
	
	if(TIM3->SR & TIM_IT_CC3) 
	{
		TIM3->SR = ~TIM_IT_CC3;
		TIM3->SR = ~TIM_FLAG_CC3OF;
		
		if(GPIOB->IDR & GPIO_Pin_0)
		{
			temp_cnt1 = TIM_GetCapture3(TIM3);	
		}
		else
		{
			temp_cnt1_2 = TIM_GetCapture3(TIM3);
			
			if(temp_cnt1_2>=temp_cnt1)
				pulseHigh = temp_cnt1_2 - temp_cnt1;
			else
				pulseHigh = 0xffff-temp_cnt1+temp_cnt1_2;	
			
			//ppmCal(pulseHigh);	
		}
		
	}
}
#endif

#if(TIM4_PPM_IN == 1)
void TIM4_IRQHandler(void)	
{
    
}
#endif
