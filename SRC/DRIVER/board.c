/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     board.c
 * @说明     硬件初始化
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "board.h"

#include "FreeRTOS.h"
#include "task.h"

#include "drv_spi.h"
#include "drv_usart.h"
#include "drv_i2c_soft.h"

static void SysPeriphClockInit(void);

/**********************************************************************************************************
*函 数 名: BoardInit
*功能说明: 飞控硬件初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void BoardInit(void)
{
	//中断优先级组别设置
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	//外设时钟使能
	SysPeriphClockInit();
	
	//SPI初始化
    Spi_GPIO_Init();
	#if (configUSE_SPI1 == 1)
		Spi_Open(1);
	#endif
	#if (configUSE_SPI2 == 1)
		Spi_Open(2);
	#endif
    
    //串口初始化
    #if (configUSE_USART1 == 1)
		Usart_Open(1);
	#endif    
    #if (configUSE_USART2 == 1)
		Usart_Open(2);
	#endif  
    #if (configUSE_USART3 == 1)
		Usart_Open(3);
	#endif  
    #if (configUSE_UART4 == 1)
		Usart_Open(4);
	#endif  
    #if (configUSE_UART5 == 1)
		Usart_Open(5);
	#endif  
    #if (configUSE_USART6 == 1)
		Usart_Open(6);
	#endif 

    //软件I2C初始化
    #if (configUSE_SOFT_I2C1 == 1)
		Soft_I2c_Open(1);
	#endif    
    #if (configUSE_SOFT_I2C2 == 1)
		Soft_I2c_Open(2);
	#endif      
}

/**********************************************************************************************************
*函 数 名: SysPeriphClockInit
*功能说明: 单片机外设时钟使能
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void SysPeriphClockInit(void)
{
	//GPIO
	#if(configUSE_GPIOA == 1)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	#endif
	#if(configUSE_GPIOB == 1)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	#endif
	#if(configUSE_GPIOC == 1)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	#endif
	#if(configUSE_GPIOD == 1)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	#endif
	#if(configUSE_GPIOE == 1)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	#endif
	#if(configUSE_GPIOF == 1)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	#endif
	#if(configUSE_GPIOG == 1)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	#endif
	//串口
	#if(configUSE_USART1 == 1)	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	#endif
	#if(configUSE_USART2 == 1)	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	#endif
	#if(configUSE_USART3 == 1)	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	#endif
	#if(configUSE_UART4 == 1)	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	#endif
	#if(configUSE_UART5 == 1)	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	#endif
	#if(configUSE_USART6 == 1)	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	#endif
	//SPI
	#if(configUSE_SPI1 == 1)	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	#endif
	#if(configUSE_SPI2 == 1)	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	#endif	
	//定时器
	#if(configUSE_TIM1 == 1)	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	#endif	
	#if(configUSE_TIM2 == 1)	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	#endif		
	#if(configUSE_TIM3 == 1)	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	#endif	
	#if(configUSE_TIM4 == 1)	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	#endif	
}


/**********************************************************************************************************
*函 数 名: SoftDelayMs
*功能说明: 毫秒级软件延时，基于STM32F4 168M主频的调试值，不同单片机有所区别
*形    参: 延时时间
*返 回 值: 无
**********************************************************************************************************/
void SoftDelayMs(uint32_t ms)
{
	uint32_t us_cnt; 
	for(; ms!= 0;ms--)
    {
        us_cnt = 42000;
		while(us_cnt)
        {
			us_cnt--;
		}	
    }		
}

/**********************************************************************************************************
*函 数 名: SoftDelayUs
*功能说明: 微秒级软件延时，基于STM32F4 168M主频的调试值，不同单片机有所区别
*形    参: 延时时间
*返 回 值: 无
**********************************************************************************************************/
void SoftDelayUs(uint32_t us)
{
	uint32_t us_cnt; 
	for(; us!= 0;us--)
    {
        us_cnt = 35;
		while(us_cnt)
        {
			us_cnt--;
		}
    }        
}

/**********************************************************************************************************
*函 数 名: GetSysTimeUs
*功能说明: 获取当前系统运行时间，单位为微秒
*形    参: 无
*返 回 值: 系统时间
**********************************************************************************************************/
uint32_t GetSysTimeUs(void) 
{
  register uint32_t ms;
	uint32_t value;
	ms = xTaskGetTickCount();
	value = ms * 1000 + (SysTick->LOAD - SysTick->VAL) * 1000 / SysTick->LOAD;
	return value;
}

/**********************************************************************************************************
*函 数 名: GetSysTimeMs
*功能说明: 获取当前系统运行时间，单位为毫秒
*形    参: 无
*返 回 值: 系统时间
**********************************************************************************************************/
uint32_t GetSysTimeMs(void) 
{
	return xTaskGetTickCount();
}


