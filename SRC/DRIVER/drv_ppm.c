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
#include "drv_sbus.h"

#define PULSE_MIN   300
#define PULSE_MAX   1800

union
{
    uint16_t captures[16];
    RCDATA_t data;
} ppm;

static RcDataCallback rcDataCallbackFunc;

/**********************************************************************************************************
*函 数 名: PPM_SetRcDataCallback
*功能说明: 设置遥控数据处理回调函数
*形    参: 回调函数
*返 回 值: 无
**********************************************************************************************************/
void PPM_SetRcDataCallback(RcDataCallback rcDataCallback)
{
    rcDataCallbackFunc = rcDataCallback;
}

/**********************************************************************************************************
*函 数 名: PPM_Cal
*功能说明: PPM通道数据计算
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void PPM_Cal(uint16_t pulseHigh)
{
    static uint8_t chan = 0;
    static uint8_t  initFlag = 0;
    
    //脉宽高于一定值说明一帧数据已经结束
    if(pulseHigh > 5000)
    {
        uint16_t temp;
        temp = ppm.captures[3];
        ppm.captures[3] = ppm.captures[2];
        ppm.captures[2] = temp;

        //通道反向
        ppm.captures[0] = 3000 - ppm.captures[0];
        ppm.captures[2] = 3000 - ppm.captures[2];

        //一帧数据解析完成
        if(rcDataCallbackFunc != 0)
            (*rcDataCallbackFunc)(ppm.data);

        chan = 0;
        
        if(!initFlag)
        {
            //禁用SBUS输入
            Sbus_Disable();
            initFlag = 1;
        }
    }
    else
    {
        if (pulseHigh > PULSE_MIN && pulseHigh < PULSE_MAX)
        {
            if(chan < 16)
            {
                ppm.captures[chan++] = (pulseHigh - 600) + 1000;
            }
        }
    }
}

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

/**********************************************************************************************************
*函 数 名: PPM_Disable
*功能说明: PPM输入关闭
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void PPM_Disable(void)
{
#if(TIM1_PPM_IN == 1)
#if(configUSE_TIM1_CH1 == 1)
    TIM_ITConfig(TIM1, TIM_IT_CC1, DISABLE);
#endif
#if(configUSE_TIM1_CH2 == 1)
    TIM_ITConfig(TIM1, TIM_IT_CC2, DISABLE);
#endif
#if(configUSE_TIM1_CH3 == 1)
    TIM_ITConfig(TIM1, TIM_IT_CC3, DISABLE);
#endif
#if(configUSE_TIM1_CH4 == 1)
    TIM_ITConfig(TIM1, TIM_IT_CC4, DISABLE);
#endif
    TIM_Cmd(TIM1, DISABLE);
#endif

#if(TIM2_PPM_IN == 1)
#if(configUSE_TIM2_CH1 == 1)
    TIM_ITConfig(TIM2, TIM_IT_CC1, DISABLE);
#endif
#if(configUSE_TIM2_CH2 == 1)
    TIM_ITConfig(TIM2, TIM_IT_CC2, DISABLE);
#endif
#if(configUSE_TIM2_CH3 == 1)
    TIM_ITConfig(TIM1, TIM_IT_CC3, DISABLE);
#endif
#if(configUSE_TIM2_CH4 == 1)
    TIM_ITConfig(TIM2, TIM_IT_CC4, DISABLE);
#endif
    TIM_Cmd(TIM2, DISABLE);
#endif

#if(TIM3_PPM_IN== 1)
#if(configUSE_TIM3_CH1 == 1)
    TIM_ITConfig(TIM3, TIM_IT_CC1, DISABLE);
#endif
#if(configUSE_TIM3_CH2 == 1)
    TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);
#endif
#if(configUSE_TIM3_CH3 == 1)
    TIM_ITConfig(TIM3, TIM_IT_CC3, DISABLE);
#endif
#if(configUSE_TIM3_CH4 == 1)
    TIM_ITConfig(TIM3, TIM_IT_CC4, DISABLE);
#endif
    TIM_Cmd(TIM3, DISABLE);
#endif

#if(TIM4_PPM_IN == 1)
#if(configUSE_TIM4_CH1 == 1)
    TIM_ITConfig(TIM4, TIM_IT_CC1, DISABLE);
#endif
#if(configUSE_TIM4_CH2 == 1)
    TIM_ITConfig(TIM4, TIM_IT_CC2, DISABLE);
#endif
#if(configUSE_TIM4_CH3 == 1)
    TIM_ITConfig(TIM4, TIM_IT_CC3, DISABLE);
#endif
#if(configUSE_TIM4_CH4 == 1)
    TIM_ITConfig(TIM4, TIM_IT_CC4, DISABLE);
#endif
    TIM_Cmd(TIM4, DISABLE);
#endif
}

/**********************************************************************************************************
*函 数 名: PPM_Decode
*功能说明: PPM定时器数据捕获与解码
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void PPM_Decode(void)
{
    TIM_TypeDef* timer[] = {TIM1, TIM2, TIM3, TIM4};
    static uint16_t periodVal1, periodVal2;
    static uint16_t pulseHigh;

#if( PPM_CH == 1 )
    if(timer[PPM_TIM - 1]->SR & TIM_IT_CC1)
    {
        timer[PPM_TIM - 1]->SR = ~TIM_IT_CC1;
        timer[PPM_TIM - 1]->SR = ~TIM_FLAG_CC1OF;
        if(PPM_GPIO->IDR & PPM_PIN)
        {
            periodVal1 = TIM_GetCapture1(timer[PPM_TIM - 1]);
        }
        else
        {
            periodVal2 = TIM_GetCapture1(timer[PPM_TIM - 1]);
#endif
#if( PPM_CH == 2 )
    if(timer[PPM_TIM - 1]->SR & TIM_IT_CC2)
    {
        timer[PPM_TIM - 1]->SR = ~TIM_IT_CC2;
        timer[PPM_TIM - 1]->SR = ~TIM_FLAG_CC2OF;
        if(PPM_GPIO->IDR & PPM_PIN)
        {
            periodVal1 = TIM_GetCapture2(timer[PPM_TIM - 1]);
        }
        else
        {
            periodVal2 = TIM_GetCapture2(timer[PPM_TIM - 1]);
#endif
#if( PPM_CH == 3 )
    if(timer[PPM_TIM - 1]->SR & TIM_IT_CC3)
    {
        timer[PPM_TIM - 1]->SR = ~TIM_IT_CC3;
        timer[PPM_TIM - 1]->SR = ~TIM_FLAG_CC3OF;
        if(PPM_GPIO->IDR & PPM_PIN)
        {
            periodVal1 = TIM_GetCapture3(timer[PPM_TIM - 1]);
        }
        else
        {
            periodVal2 = TIM_GetCapture3(timer[PPM_TIM - 1]);
#endif
#if( PPM_CH == 4 )
    if(timer[PPM_TIM - 1]->SR & TIM_IT_CC4)
    {
        timer[PPM_TIM - 1]->SR = ~TIM_IT_CC4;
        timer[PPM_TIM - 1]->SR = ~TIM_FLAG_CC4OF;
        if(PPM_GPIO->IDR & PPM_PIN)
        {
            periodVal1 = TIM_GetCapture4(timer[PPM_TIM - 1]);
        }
        else
        {
            periodVal2 = TIM_GetCapture4(timer[PPM_TIM - 1]);
#endif

            if(periodVal2 >= periodVal1)
                pulseHigh = periodVal2 - periodVal1;
            else
                pulseHigh = 0xffff - periodVal1 + periodVal2;

            PPM_Cal(pulseHigh);
        }
    }
}

/**********************************************************************************************************
*函 数 名: TIMx_IRQHandler
*功能说明: 定时器中断函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
#if(TIM1_PPM_IN == 1)
void TIM1_CC_IRQHandler(void)
{
    PPM_Decode();
}
#endif

#if(TIM2_PPM_IN == 1)
void TIM2_IRQHandler(void)
{
    PPM_Decode();
}
#endif

#if(TIM3_PPM_IN == 1)
void TIM3_IRQHandler(void)
{
    PPM_Decode();
}
#endif

#if(TIM4_PPM_IN == 1)
void TIM4_IRQHandler(void)
{
    PPM_Decode();
}
#endif
