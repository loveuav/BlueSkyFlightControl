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
#define BARO_CS_GPIO         GPIOC
#define BARO_CS_PIN          GPIO_Pin_3

#define MAG_I2C              1
#define GPS_UART             4
#define TOF_UART             0
#define SBUS_UART            6

#define ESC_PROTOCOL         PWM
//#define ESC_PROTOCOL         DSHOT600

#define RC_PROTOCOL		     SBUS
//#define RC_PROTOCOL		     PPM

#define PWM_TIM_FREQ         21000000
#define PWM_TIM_PERIOD       52499
#define PWM1_TIM             3
#define PWM1_CH              1
#define PWM2_TIM             3
#define PWM2_CH              2
#define PWM3_TIM             3
#define PWM3_CH              3
#define PWM4_TIM             3
#define PWM4_CH              4

#define PPM_TIM_FREQ         21000000
#define PPM_TIM_PERIOD       52499
#define PPM_TIM              0
#define PPM_CH               0

/**********************************************************************************************************
*单片机Flash存储区域分配
**********************************************************************************************************/
//Flash扇区的基地址 
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

//    项目            开始地址	          结束地址	      大小
//	bootloader	 0x8000000 sector0	 0x800BFFF sector2	  48KB
//	boot_para  	 0x800C000 sector3	 0x800FFFF sector3	  16KB
//	application	 0x8010000 sector4	 0x807FFFF sector7	  448KB
//	bin file	 0x8080000 sector8	 0x80DFFFF sector10	  384KB
//	app_para	 0x80E0000 sector11	 0x80FFFFF sector11	  128KB
#define FLASH_BASE_START_ADDR           0x08000000 	    //Flash的起始地址
#define FLASH_BASE_END_ADDR             0x080FFFFF 	    //Flash的结束地址

#define FLASH_BOOT_START_ADDR		    ADDR_FLASH_SECTOR_0  	//Bootloader存储区
#define FLASH_BOOT_PARA_START_ADDR      ADDR_FLASH_SECTOR_3  	//Bootloader参数存储区
#define FLASH_USER_START_ADDR		    ADDR_FLASH_SECTOR_4  	//用户程序存储区
#define FLASH_BIN_START_ADDR            ADDR_FLASH_SECTOR_8  	//固件升级bin文件存储区
#define FLASH_USER_PARA_START_ADDR      ADDR_FLASH_SECTOR_11  	//用户参数存储区

#define FLASH_VECTOR_TAB_OFFSET         0x10000

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
#define configUSE_USART6     1

#define configUSE_SPI1       1
#define configUSE_SPI2       0

#define configUSE_SOFT_I2C1  1
#define configUSE_SOFT_I2C2  0

#define configUSE_TIM1       1
#define configUSE_TIM1_CH1   1
#define configUSE_TIM1_CH2   0
#define configUSE_TIM1_CH3   0
#define configUSE_TIM1_CH4   0
#define configUSE_TIM2       0
#define configUSE_TIM2_CH1   0
#define configUSE_TIM2_CH2   0
#define configUSE_TIM2_CH3   0
#define configUSE_TIM2_CH4   0
#define configUSE_TIM3       1
#define configUSE_TIM3_CH1   1
#define configUSE_TIM3_CH2   1
#define configUSE_TIM3_CH3   1
#define configUSE_TIM3_CH4   1
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
#define USART6_BAUDRATE         100000
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
#define TIM1_CLOCK                    TEMP_TIM_FREQ
#define TIM1_PERIOD                   TEMP_TIM_PERIOD
#define TIM1_PWM_OUT                  1
#define TIM1_PPM_IN                   0		
#if(configUSE_TIM1_CH1 == 1)
	#define TIM1_CH1_GPIO             GPIOA
	#define TIM1_CH1_PIN              GPIO_Pin_8
	#define TIM1_CH1_PINSOURCE        GPIO_PinSource8
#endif
#if(configUSE_TIM1_CH2 == 1)
	#define TIM1_CH2_GPIO             GPIOA
	#define TIM1_CH2_PIN              GPIO_Pin_7
	#define TIM1_CH2_PINSOURCE        GPIO_PinSource5
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

#define TIM2_CLOCK                    PWM_TIM_FREQ
#define TIM2_PERIOD                   PWM_TIM_PERIOD
#define TIM2_PWM_OUT                  1
#define TIM2_PPM_IN                   0		
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

#define TIM3_CLOCK                    PWM_TIM_FREQ
#define TIM3_PERIOD                   PWM_TIM_PERIOD
#define TIM3_PWM_OUT                  1
#define TIM3_PPM_IN                   0		
#if(configUSE_TIM3_CH1 == 1)
	#define TIM3_CH1_GPIO             GPIOB
	#define TIM3_CH1_PIN              GPIO_Pin_0
	#define TIM3_CH1_PINSOURCE        GPIO_PinSource0
#endif
#if(configUSE_TIM3_CH2 == 1)
	#define TIM3_CH2_GPIO             GPIOB
	#define TIM3_CH2_PIN              GPIO_Pin_1
	#define TIM3_CH2_PINSOURCE        GPIO_PinSource1
#endif
#if(configUSE_TIM3_CH3 == 1)
	#define TIM3_CH3_GPIO             GPIOB
	#define TIM3_CH3_PIN              GPIO_Pin_4
	#define TIM3_CH3_PINSOURCE        GPIO_PinSource4
#endif
#if(configUSE_TIM3_CH4 == 1)
	#define TIM3_CH4_GPIO             GPIOB
	#define TIM3_CH4_PIN              GPIO_Pin_5
	#define TIM3_CH4_PINSOURCE        GPIO_PinSource5
#endif	

#define TIM4_CLOCK                    PPM_TIM_FREQ
#define TIM4_PERIOD                   PPM_TIM_PERIOD
#define TIM4_PWM_OUT                  0
#define TIM4_PPM_IN                   1		
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


