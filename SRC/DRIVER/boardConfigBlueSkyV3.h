/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     boardConfigBlueSkyV3.h
 * @说明     BlueSky V3 飞控硬件配置文件
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/

#ifndef __BOARDCONFIGBLUESKYV3_H__
#define __BOARDCONFIGBLUESKYV3_H__

/**********************************************************************************************************
*传感器配置
**********************************************************************************************************/
#define GYRO_TYPE            MPU6000
//#define GYRO_TYPE            MPU6500
#define BARO_TYPE            MS5611    
#define MAG_TYPE             QMC5883

#define GYRO_SPI             1
#define GYRO_CS_GPIO         GPIOC
#define GYRO_CS_PIN          GPIO_Pin_2

#define TEMP_TIM_FREQ        21000000
#define TEMP_TIM_PERIOD      52499
#define TEMP_TIM             1
#define TEMP_CH              1

#define BARO_SPI             1
#define BARO_CS_GPIO         GPIOD
#define BARO_CS_PIN          GPIO_Pin_7

#define MAG_I2C              1
#define GPS_UART             4
#define TOF_UART             0
#define SBUS_UART            0

#define PWM_TIM_FREQ         21000000
#define PWM_TIM_PERIOD       52499
#define PWM1_TIM             0
#define PWM1_CH              0
#define PWM2_TIM             0
#define PWM2_CH              0
#define PWM3_TIM             0
#define PWM3_CH              0
#define PWM4_TIM             0
#define PWM4_CH              0

#define PPM_TIM_FREQ         21000000
#define PPM_TIM_PERIOD       52499
#define PPM_TIM              0
#define PPM_CH               0

/**********************************************************************************************************
*外设使能配置
**********************************************************************************************************/
#define configUSE_GPIOA      1
#define configUSE_GPIOB      1
#define configUSE_GPIOC      1
#define configUSE_GPIOD      1
#define configUSE_GPIOE      0
#define configUSE_GPIOF      0
#define configUSE_GPIOG      0

#define configUSE_USART1     0
#define configUSE_USART2     0
#define configUSE_USART3     0
#define configUSE_UART4      1
#define configUSE_UART5      0
#define configUSE_USART6     0

#define configUSE_SPI1       1
#define configUSE_SPI2       0

#define configUSE_SOFT_I2C1  1
#define configUSE_SOFT_I2C2  0

#define configUSE_TIM1       0
#define configUSE_TIM1_CH1   0
#define configUSE_TIM1_CH2   0
#define configUSE_TIM1_CH3   0
#define configUSE_TIM1_CH4   0
#define configUSE_TIM2       0
#define configUSE_TIM2_CH1   0
#define configUSE_TIM2_CH2   0
#define configUSE_TIM2_CH3   0
#define configUSE_TIM2_CH4   0
#define configUSE_TIM3       0
#define configUSE_TIM3_CH1   0
#define configUSE_TIM3_CH2   0
#define configUSE_TIM3_CH3   0
#define configUSE_TIM3_CH4   0
#define configUSE_TIM4       0
#define configUSE_TIM4_CH1   0
#define configUSE_TIM4_CH2   0
#define configUSE_TIM4_CH3   0
#define configUSE_TIM4_CH4   0

/**********************************************************************************************************
*串口引脚及参数配置
**********************************************************************************************************/
#define USART1_GPIO             GPIOA
#define USART1_PINSOURCE_TX     GPIO_PinSource1
#define USART1_PINSOURCE_RX     GPIO_PinSource1
#define USART1_PIN_TX           GPIO_Pin_6
#define USART1_PIN_RX           GPIO_Pin_7
#define USART1_BAUDRATE         115200
#define USART1_IRQ_PRIORITY     3

#define USART2_GPIO             GPIOA
#define USART2_PINSOURCE_TX     GPIO_PinSource1
#define USART2_PINSOURCE_RX     GPIO_PinSource1
#define USART2_PIN_TX           GPIO_Pin_6
#define USART2_PIN_RX           GPIO_Pin_7
#define USART2_BAUDRATE         115200
#define USART2_IRQ_PRIORITY     3

#define USART3_GPIO             GPIOA
#define USART3_PINSOURCE_TX     GPIO_PinSource1
#define USART3_PINSOURCE_RX     GPIO_PinSource1
#define USART3_PIN_TX           GPIO_Pin_6
#define USART3_PIN_RX           GPIO_Pin_7
#define USART3_BAUDRATE         115200
#define USART3_IRQ_PRIORITY     3

#define UART4_GPIO              GPIOA
#define UART4_PINSOURCE_TX      GPIO_PinSource0
#define UART4_PINSOURCE_RX      GPIO_PinSource1
#define UART4_PIN_TX            GPIO_Pin_0
#define UART4_PIN_RX            GPIO_Pin_1
#define UART4_BAUDRATE          230400
#define UART4_IRQ_PRIORITY      3

#define UART5_GPIO              GPIOA
#define UART5_PINSOURCE_TX      GPIO_PinSource1
#define UART5_PINSOURCE_RX      GPIO_PinSource1
#define UART5_PIN_TX            GPIO_Pin_6
#define UART5_PIN_RX            GPIO_Pin_7
#define UART5_BAUDRATE          115200
#define UART5_IRQ_PRIORITY      3

#define USART6_GPIO             GPIOA
#define USART6_PINSOURCE_TX     GPIO_PinSource1
#define USART6_PINSOURCE_RX     GPIO_PinSource1
#define USART6_PIN_TX           GPIO_Pin_6
#define USART6_PIN_RX           GPIO_Pin_7
#define USART6_BAUDRATE         115200
#define USART6_IRQ_PRIORITY     3
/**********************************************************************************************************
*SPI引脚及参数配置
**********************************************************************************************************/
#define SPI1_GPIO               GPIOA
#define SPI1_PINSOURCE_MOSI     GPIO_PinSource7
#define SPI1_PINSOURCE_MISO     GPIO_PinSource6
#define SPI1_PINSOURCE_SCK      GPIO_PinSource5
#define SPI1_PIN_MOSI           GPIO_Pin_7
#define SPI1_PIN_MISO           GPIO_Pin_6
#define SPI1_PIN_SCK            GPIO_Pin_5
#define SPI1_CLOCKDIV           SPI_BaudRatePrescaler_8

#define SPI2_GPIO               GPIOA
#define SPI2_PINSOURCE_MOSI     GPIO_PinSource7
#define SPI2_PINSOURCE_MISO     GPIO_PinSource6
#define SPI2_PINSOURCE_SCK      GPIO_PinSource5
#define SPI2_PIN_MOSI           GPIO_Pin_6
#define SPI2_PIN_MISO           GPIO_Pin_7
#define SPI2_PIN_SCK            GPIO_Pin_7
#define SPI2_CLOCKDIV           SPI_BaudRatePrescaler_8

/**********************************************************************************************************
*软件I2C引脚及参数配置
**********************************************************************************************************/
#define SOFT_I2C1_GPIO              GPIOB
#define SOFT_I2C1_PIN_SCL           GPIO_Pin_8
#define SOFT_I2C1_PIN_SDA           GPIO_Pin_9
#define SOFT_I2C1_DELAY             0

#define SOFT_I2C2_GPIO              GPIOA
#define SOFT_I2C2_PIN_SCL           GPIO_Pin_7
#define SOFT_I2C2_PIN_SDA           GPIO_Pin_7
#define SOFT_I2C2_DELAY             0

/**********************************************************************************************************
*定时器引脚及参数配置
**********************************************************************************************************/
#define TIM1_CLOCK                    84
#define TIM1_PWM_OUT                  0
#define TIM1_PWM_IN                   0		
#if(configUSE_TIM1_CH1 == 1)
	#define TIM1_CH1_GPIO             GPIOA
	#define TIM1_CH1_PIN              GPIO_Pin_7
	#define TIM1_CH1_PINSOURCE        GPIO_PinSource5
#endif
#if(configUSE_TIM1_CH2 == 1)
	#define TIM1_CH2_GPIO             GPIOA
	#define TIM1_CH2_PIN              GPIO_Pin_7
	#define TIM1_CH3_PINSOURCE        GPIO_PinSource5
#endif
#if(configUSE_TIM1_CH3 == 1)
	#define TIM1_CH3_GPIO             GPIOA
	#define TIM1_CH3_PIN              GPIO_Pin_7
	#define TIM1_CH3_PINSOURCE        GPIO_PinSource5
#endif
#if(configUSE_TIM1_CH4 == 1)
	#define TIM1_CH4_GPIO             GPIOA
	#define TIM1_CH4_PIN              GPIO_Pin_7
	#define TIM1_CH4_PINSOURCE        GPIO_PinSource5
#endif

#define TIM2_CLOCK                    84
#define TIM2_PWM_OUT                  0
#define TIM2_PWM_IN                   0		
#if(configUSE_TIM2_CH1 == 1)
	#define TIM2_CH1_GPIO             GPIOA
	#define TIM2_CH1_PIN              GPIO_Pin_7
	#define TIM2_CH1_PINSOURCE        GPIO_PinSource5
#endif
#if(configUSE_TIM2_CH2 == 1)
	#define TIM2_CH2_GPIO             GPIOA
	#define TIM2_CH2_PIN              GPIO_Pin_7
	#define TIM2_CH2_PINSOURCE        GPIO_PinSource5
#endif
#if(configUSE_TIM2_CH3 == 1)
	#define TIM2_CH3_GPIO             GPIOA
	#define TIM2_CH3_PIN              GPIO_Pin_7
	#define TIM2_CH3_PINSOURCE        GPIO_PinSource5
#endif
#if(configUSE_TIM2_CH4 == 1)
	#define TIM2_CH4_GPIO             GPIOA
	#define TIM2_CH4_PIN              GPIO_Pin_7
	#define TIM2_CH4_PINSOURCE        GPIO_PinSource5
#endif	

#define TIM3_CLOCK                    84
#define TIM3_PWM_OUT                  0
#define TIM3_PWM_IN                   0		
#if(configUSE_TIM3_CH1 == 1)
	#define TIM3_CH1_GPIO             GPIOA
	#define TIM3_CH1_PIN              GPIO_Pin_7
	#define TIM3_CH1_PINSOURCE        GPIO_PinSource5
#endif
#if(configUSE_TIM3_CH2 == 1)
	#define TIM3_CH2_GPIO             GPIOA
	#define TIM3_CH2_PIN              GPIO_Pin_7
	#define TIM3_CH2_PINSOURCE        GPIO_PinSource5
#endif
#if(configUSE_TIM3_CH3 == 1)
	#define TIM3_CH3_GPIO             GPIOA
	#define TIM3_CH3_PIN              GPIO_Pin_7
	#define TIM3_CH3_PINSOURCE        GPIO_PinSource5
#endif
#if(configUSE_TIM3_CH4 == 1)
	#define TIM3_CH4_GPIO             GPIOA
	#define TIM3_CH4_PIN              GPIO_Pin_7
	#define TIM3_CH4_PINSOURCE        GPIO_PinSource5
#endif	

#define TIM4_CLOCK                    84
#define TIM4_PWM_OUT                  0
#define TIM4_PWM_IN                   0		
#if(configUSE_TIM4_CH1 == 1)
	#define TIM4_CH1_GPIO             GPIOA
	#define TIM4_CH1_PIN              GPIO_Pin_7
	#define TIM4_CH1_PINSOURCE        GPIO_PinSource5
#endif
#if(configUSE_TIM4_CH2 == 1)
	#define TIM4_CH2_GPIO             GPIOA
	#define TIM4_CH2_PIN              GPIO_Pin_7
	#define TIM4_CH2_PINSOURCE        GPIO_PinSource5
#endif
#if(configUSE_TIM4_CH3 == 1)
	#define TIM4_CH3_GPIO             GPIOA
	#define TIM4_CH3_PIN              GPIO_Pin_7
	#define TIM4_CH3_PINSOURCE        GPIO_PinSource5
#endif
#if(configUSE_TIM4_CH4 == 1)
	#define TIM4_CH4_GPIO             GPIOA
	#define TIM4_CH4_PIN              GPIO_Pin_7
	#define TIM4_CH4_PINSOURCE        GPIO_PinSource5
#endif	


#endif	


