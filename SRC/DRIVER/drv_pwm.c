/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     drv_pwm.c
 * @说明     定时器PWM输出驱动
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05
**********************************************************************************************************/
#include "drv_pwm.h"

/**********************************************************************************************************
*函 数 名: PWM_Init
*功能说明: PWM输出初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void PWM_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    uint16_t PrescalerValue = 0;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

#if(TIM1_PWM_OUT == 1)
    PrescalerValue =  (uint16_t) ((SystemCoreClock) / TIM1_CLOCK) - 1;
    TIM_TimeBaseStructure.TIM_Period = TIM1_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

#if(configUSE_TIM1_CH1 == 1)
    GPIO_InitStructure.GPIO_Pin = TIM1_CH1_PIN;
    GPIO_Init(TIM1_CH1_GPIO, &GPIO_InitStructure);
    GPIO_PinAFConfig(TIM1_CH1_GPIO, TIM1_CH1_PINSOURCE, GPIO_AF_TIM1);
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
#endif
#if(configUSE_TIM1_CH2 == 1)
    GPIO_InitStructure.GPIO_Pin = TIM1_CH2_PIN;
    GPIO_Init(TIM1_CH2_GPIO, &GPIO_InitStructure);
    GPIO_PinAFConfig(TIM1_CH2_GPIO, TIM1_CH2_PINSOURCE, GPIO_AF_TIM1);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
#endif
#if(configUSE_TIM1_CH3 == 1)
    GPIO_InitStructure.GPIO_Pin = TIM1_CH3_PIN;
    GPIO_Init(TIM1_CH3_GPIO, &GPIO_InitStructure);
    GPIO_PinAFConfig(TIM1_CH3_GPIO, TIM1_CH3_PINSOURCE, GPIO_AF_TIM1);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
#endif
#if(configUSE_TIM1_CH4 == 1)
    GPIO_InitStructure.GPIO_Pin = TIM1_CH4_PIN;
    GPIO_Init(TIM1_CH4_GPIO, &GPIO_InitStructure);
    GPIO_PinAFConfig(TIM1_CH4_GPIO, TIM1_CH4_PINSOURCE, GPIO_AF_TIM1);
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
#endif
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
#endif

#if(TIM2_PWM_OUT == 1)
    PrescalerValue =  (uint16_t) ((SystemCoreClock / 2) / TIM2_CLOCK) - 1;
    TIM_TimeBaseStructure.TIM_Period = TIM2_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

#if(configUSE_TIM2_CH1 == 1)
    GPIO_InitStructure.GPIO_Pin = TIM2_CH1_PIN;
    GPIO_Init(TIM2_CH1_GPIO, &GPIO_InitStructure);
    GPIO_PinAFConfig(TIM2_CH1_GPIO, TIM2_CH1_PINSOURCE, GPIO_AF_TIM2);
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
#endif
#if(configUSE_TIM2_CH2 == 1)
    GPIO_InitStructure.GPIO_Pin = TIM2_CH2_PIN;
    GPIO_Init(TIM2_CH2_GPIO, &GPIO_InitStructure);
    GPIO_PinAFConfig(TIM2_CH2_GPIO, TIM2_CH2_PINSOURCE, GPIO_AF_TIM2);
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
#endif
#if(configUSE_TIM2_CH3 == 1)
    GPIO_InitStructure.GPIO_Pin = TIM2_CH3_PIN;
    GPIO_Init(TIM2_CH3_GPIO, &GPIO_InitStructure);
    GPIO_PinAFConfig(TIM2_CH3_GPIO, TIM2_CH3_PINSOURCE, GPIO_AF_TIM2);
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
#endif
#if(configUSE_TIM2_CH4 == 1)
    GPIO_InitStructure.GPIO_Pin = TIM2_CH4_PIN;
    GPIO_Init(TIM2_CH4_GPIO, &GPIO_InitStructure);
    GPIO_PinAFConfig(TIM2_CH4_GPIO, TIM2_CH4_PINSOURCE, GPIO_AF_TIM2);
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
#endif
    TIM_Cmd(TIM2, ENABLE);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
#endif

#if(TIM3_PWM_OUT == 1)
    PrescalerValue =  (uint16_t) ((SystemCoreClock / 2) / TIM3_CLOCK) - 1;
    TIM_TimeBaseStructure.TIM_Period = TIM3_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

#if(configUSE_TIM3_CH1 == 1)
    GPIO_InitStructure.GPIO_Pin = TIM3_CH1_PIN;
    GPIO_Init(TIM3_CH1_GPIO, &GPIO_InitStructure);
    GPIO_PinAFConfig(TIM3_CH1_GPIO, TIM3_CH1_PINSOURCE, GPIO_AF_TIM3);
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
#endif
#if(configUSE_TIM3_CH2 == 1)
    GPIO_InitStructure.GPIO_Pin = TIM3_CH2_PIN;
    GPIO_Init(TIM3_CH2_GPIO, &GPIO_InitStructure);
    GPIO_PinAFConfig(TIM3_CH2_GPIO, TIM3_CH2_PINSOURCE, GPIO_AF_TIM3);
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
#endif
#if(configUSE_TIM3_CH3 == 1)
    GPIO_InitStructure.GPIO_Pin = TIM3_CH3_PIN;
    GPIO_Init(TIM3_CH3_GPIO, &GPIO_InitStructure);
    GPIO_PinAFConfig(TIM3_CH3_GPIO, TIM3_CH3_PINSOURCE, GPIO_AF_TIM3);
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
#endif
#if(configUSE_TIM3_CH4 == 1)
    GPIO_InitStructure.GPIO_Pin = TIM3_CH4_PIN;
    GPIO_Init(TIM3_CH4_GPIO, &GPIO_InitStructure);
    GPIO_PinAFConfig(TIM3_CH4_GPIO, TIM3_CH4_PINSOURCE, GPIO_AF_TIM3);
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
#endif
    TIM_Cmd(TIM3, ENABLE);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
#endif

#if(TIM4_PWM_OUT == 1)
    PrescalerValue =  (uint16_t) ((SystemCoreClock / 2) / TIM4_CLOCK) - 1;
    TIM_TimeBaseStructure.TIM_Period = TIM4_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

#if(configUSE_TIM4_CH1 == 1)
    GPIO_InitStructure.GPIO_Pin = TIM4_CH1_PIN;
    GPIO_Init(TIM4_CH1_GPIO, &GPIO_InitStructure);
    GPIO_PinAFConfig(TIM4_CH1_GPIO, TIM4_CH1_PINSOURCE, GPIO_AF_TIM4);
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
#endif
#if(configUSE_TIM4_CH2 == 1)
    GPIO_InitStructure.GPIO_Pin = TIM4_CH2_PIN;
    GPIO_Init(TIM4_CH2_GPIO, &GPIO_InitStructure);
    GPIO_PinAFConfig(TIM4_CH2_GPIO, TIM4_CH2_PINSOURCE, GPIO_AF_TIM4);
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
#endif
#if(configUSE_TIM4_CH3 == 1)
    GPIO_InitStructure.GPIO_Pin = TIM4_CH3_PIN;
    GPIO_Init(TIM4_CH3_GPIO, &GPIO_InitStructure);
    GPIO_PinAFConfig(TIM4_CH3_GPIO, TIM4_CH3_PINSOURCE, GPIO_AF_TIM4);
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
#endif
#if(configUSE_TIM4_CH4 == 1)
    GPIO_InitStructure.GPIO_Pin = TIM4_CH4_PIN;
    GPIO_Init(TIM4_CH4_GPIO, &GPIO_InitStructure);
    GPIO_PinAFConfig(TIM4_CH4_GPIO, TIM4_CH4_PINSOURCE, GPIO_AF_TIM4);
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
#endif
    TIM_Cmd(TIM4, ENABLE);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
#endif

#if(TIM12_PWM_OUT == 1)
    PrescalerValue =  (uint16_t) ((SystemCoreClock / 2) / TIM12_CLOCK) - 1;
    TIM_TimeBaseStructure.TIM_Period = TIM12_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);

#if(configUSE_TIM12_CH1 == 1)
    GPIO_InitStructure.GPIO_Pin = TIM12_CH1_PIN;
    GPIO_Init(TIM12_CH1_GPIO, &GPIO_InitStructure);
    GPIO_PinAFConfig(TIM12_CH1_GPIO, TIM12_CH1_PINSOURCE, GPIO_AF_TIM12);
    TIM_OC1Init(TIM12, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);
#endif
#if(configUSE_TIM12_CH2 == 1)
    GPIO_InitStructure.GPIO_Pin = TIM12_CH2_PIN;
    GPIO_Init(TIM12_CH2_GPIO, &GPIO_InitStructure);
    GPIO_PinAFConfig(TIM12_CH2_GPIO, TIM12_CH2_PINSOURCE, GPIO_AF_TIM12);
    TIM_OC2Init(TIM12, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM12, TIM_OCPreload_Enable);
#endif
#if(configUSE_TIM12_CH3 == 1)
    GPIO_InitStructure.GPIO_Pin = TIM12_CH3_PIN;
    GPIO_Init(TIM12_CH3_GPIO, &GPIO_InitStructure);
    GPIO_PinAFConfig(TIM12_CH3_GPIO, TIM12_CH3_PINSOURCE, GPIO_AF_TIM12);
    TIM_OC3Init(TIM12, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM12, TIM_OCPreload_Enable);
#endif
#if(configUSE_TIM12_CH4 == 1)
    GPIO_InitStructure.GPIO_Pin = TIM12_CH4_PIN;
    GPIO_Init(TIM12_CH4_GPIO, &GPIO_InitStructure);
    GPIO_PinAFConfig(TIM12_CH4_GPIO, TIM12_CH4_PINSOURCE, GPIO_AF_TIM12);
    TIM_OC4Init(TIM12, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM12, TIM_OCPreload_Enable);
#endif
    TIM_Cmd(TIM12, ENABLE);
    TIM_ARRPreloadConfig(TIM12, ENABLE);
#endif
}

/**********************************************************************************************************
*函 数 名: TempControlPWMSet
*功能说明: 温控PWM输出值设置
*形    参: PWM值（0-1000）
*返 回 值: 无
**********************************************************************************************************/
void TempControlPWMSet(int32_t pwmValue)
{
    TIM_TypeDef* timer[] = {TIM1, TIM2, TIM3, TIM4, TIM5, TIM6, TIM7, TIM8, TIM9, TIM10, TIM11, TIM12};

    pwmValue = pwmValue * ((int32_t)TEMP_TIM_PERIOD / 1000);

#if(TEMP_CH == 1)
    timer[TEMP_TIM-1]->CCR1 = pwmValue;
#endif
#if(TEMP_CH == 2)
    timer[TEMP_TIM-1]->CCR2 = pwmValue;
#endif
#if(TEMP_CH == 3)
    timer[TEMP_TIM-1]->CCR3 = pwmValue;
#endif
#if(TEMP_CH == 4)
    timer[TEMP_TIM-1]->CCR4 = pwmValue;
#endif
}


/**********************************************************************************************************
*函 数 名: MotorPWMSet
*功能说明: 电机PWM输出值设置
*形    参: 电机号 PWM值（0-2000）
*返 回 值: 无
**********************************************************************************************************/
void MotorPWMSet(uint8_t motor, uint16_t pwmValue)
{
    TIM_TypeDef* timer[] = {TIM1, TIM2, TIM3, TIM4};

    //使用PWM协议输出信号给电调
#if(ESC_PROTOCOL == PWM)
    pwmValue = 21 * (pwmValue * 0.5f) + 21000;
#endif

    if(motor == 1)
    {
#if(PWM1_CH == 1)
        timer[PWM1_TIM-1]->CCR1 = pwmValue;
#endif
#if(PWM1_CH == 2)
        timer[PWM1_TIM-1]->CCR2 = pwmValue;
#endif
#if(PWM1_CH == 3)
        timer[PWM1_TIM-1]->CCR3 = pwmValue;
#endif
#if(PWM1_CH == 4)
        timer[PWM1_TIM-1]->CCR4 = pwmValue;
#endif
    }
    else if(motor == 2)
    {
#if(PWM2_CH == 1)
        timer[PWM2_TIM-1]->CCR1 = pwmValue;
#endif
#if(PWM2_CH == 2)
        timer[PWM2_TIM-1]->CCR2 = pwmValue;
#endif
#if(PWM2_CH == 3)
        timer[PWM2_TIM-1]->CCR3 = pwmValue;
#endif
#if(PWM2_CH == 4)
        timer[PWM2_TIM-1]->CCR4 = pwmValue;
#endif
    }
    else if(motor == 3)
    {
#if(PWM3_CH == 1)
        timer[PWM3_TIM-1]->CCR1 = pwmValue;
#endif
#if(PWM3_CH == 2)
        timer[PWM3_TIM-1]->CCR2 = pwmValue;
#endif
#if(PWM3_CH == 3)
        timer[PWM3_TIM-1]->CCR3 = pwmValue;
#endif
#if(PWM3_CH == 4)
        timer[PWM3_TIM-1]->CCR4 = pwmValue;
#endif
    }
    else if(motor == 4)
    {
#if(PWM4_CH == 1)
        timer[PWM4_TIM-1]->CCR1 = pwmValue;
#endif
#if(PWM4_CH == 2)
        timer[PWM4_TIM-1]->CCR2 = pwmValue;
#endif
#if(PWM4_CH == 3)
        timer[PWM4_TIM-1]->CCR3 = pwmValue;
#endif
#if(PWM4_CH == 4)
        timer[PWM4_TIM-1]->CCR4 = pwmValue;
#endif
    }
    else if(motor == 5)
    {
#if(PWM5_CH == 1)
        timer[PWM5_TIM-1]->CCR1 = pwmValue;
#endif
#if(PWM5_CH == 2)
        timer[PWM5_TIM-1]->CCR2 = pwmValue;
#endif
#if(PWM5_CH == 3)
        timer[PWM5_TIM-1]->CCR3 = pwmValue;
#endif
#if(PWM5_CH == 4)
        timer[PWM5_TIM-1]->CCR4 = pwmValue;
#endif
    }
    else if(motor == 6)
    {
#if(PWM6_CH == 1)
        timer[PWM6_TIM-1]->CCR1 = pwmValue;
#endif
#if(PWM6_CH == 2)
        timer[PWM6_TIM-1]->CCR2 = pwmValue;
#endif
#if(PWM6_CH == 3)
        timer[PWM6_TIM-1]->CCR3 = pwmValue;
#endif
#if(PWM6_CH == 4)
        timer[PWM6_TIM-1]->CCR4 = pwmValue;
#endif
    }
    else if(motor == 7)
    {
#if(PWM7_CH == 1)
        timer[PWM7_TIM-1]->CCR1 = pwmValue;
#endif
#if(PWM7_CH == 2)
        timer[PWM7_TIM-1]->CCR2 = pwmValue;
#endif
#if(PWM7_CH == 3)
        timer[PWM7_TIM-1]->CCR3 = pwmValue;
#endif
#if(PWM7_CH == 4)
        timer[PWM7_TIM-1]->CCR4 = pwmValue;
#endif
    }
    else if(motor == 8)
    {
#if(PWM8_CH == 1)
        timer[PWM8_TIM-1]->CCR1 = pwmValue;
#endif
#if(PWM8_CH == 2)
        timer[PWM8_TIM-1]->CCR2 = pwmValue;
#endif
#if(PWM8_CH == 3)
        timer[PWM8_TIM-1]->CCR3 = pwmValue;
#endif
#if(PWM8_CH == 4)
        timer[PWM8_TIM-1]->CCR4 = pwmValue;
#endif
    }
}

