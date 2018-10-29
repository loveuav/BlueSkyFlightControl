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
*传感器安装方向
**********************************************************************************************************/
#define GYRO_ROTATION       ROTATION_ROLL_180_YAW_270
#define ACC_ROTATION        ROTATION_ROLL_180_YAW_270
#define MAG_ROTATION        ROTATION_ROLL_180_YAW_270

/**********************************************************************************************************
*传感器配置
**********************************************************************************************************/
#define GYRO_TYPE            ICM20689       //陀螺仪型号
#define BARO_TYPE            MS5611         //气压计型号
#define MAG_TYPE             QMC5883        //罗盘型号

#define configUSE_SENSORHEAT 1              //是否使用传感器恒温

#define GYRO_SPI             2              //陀螺仪SPI配置
#define GYRO_CS_GPIO         GPIOA
#define GYRO_CS_PIN          GPIO_Pin_5

#define GYRO2_SPI            2              //陀螺仪2SPI配置
#define GYRO2_CS_GPIO        GPIOB
#define GYRO2_CS_PIN         GPIO_Pin_12

#define BARO_SPI             2              //气压计SPI配置
#define BARO_CS_GPIO         GPIOA
#define BARO_CS_PIN          GPIO_Pin_4

#define MAG_I2C              1              //磁力计I2C配置

#define GPS_UART             3              //GPS串口配置
#define GPS_BAUDRATE         0              //波特率默认115200，可自动识别并对gps模块进行配置
#define DATA_UART            1              //数据链串口配置
#define DATA_BAUDRATE        115200
#define TOF_UART             0              //TOF模块串口配置
#define TOF_BAUDRATE         115200
#define SBUS_UART            6              //SBUS接收机串口配置
#define SBUS_BAUDRATE        100000

#define SBUS_INV             1              //SBUS接收反向设置
#define SBUS_INV_GPIO        GPIOC
#define SBUS_INV_PIN         GPIO_Pin_15

#define ESC_PROTOCOL         PWM            //电调输出信号协议选择

#define TEMP_TIM_FREQ        5000000        //传感器恒温PWM输出定时器配置
#define TEMP_TIM_PERIOD      5000
#define TEMP_TIM             12
#define TEMP_CH              1

#define PWM_TIM_FREQ         21000000       //电机输出PWM输出定时器配置
#define PWM_TIM_PERIOD       45000
#define PWM1_TIM             2
#define PWM1_CH              3
#define PWM2_TIM             2
#define PWM2_CH              4
#define PWM3_TIM             3
#define PWM3_CH              3
#define PWM4_TIM             3
#define PWM4_CH              4
#define PWM5_TIM             2
#define PWM5_CH              1
#define PWM6_TIM             2
#define PWM6_CH              2
#define PWM7_TIM             3
#define PWM7_CH              1
#define PWM8_TIM             3
#define PWM8_CH              2

#define PPM_TIM_FREQ         1000000        //PPM输入捕获定时器配置
#define PPM_TIM_PERIOD       0xFFFF
#define PPM_TIM              1
#define PPM_CH               1
#define PPM_GPIO             GPIOA
#define PPM_PIN              GPIO_Pin_8
#define PPM_PINSOURCE        GPIO_PinSource8

#define ADC_VOLTAGE          ADC1           //电池电压采集ADC配置
#define ADC_VOLTAGE_CHAN     ADC_Channel_10
#define ADC_VOLTAGE_GPIO     GPIOC
#define ADC_VOLTAGE_PIN      GPIO_Pin_0
#define ADC_VOLTAGE_COEF     10.0f

#define ADC_CURRENT          ADC2           //电池电流采集ADC配置
#define ADC_CURRENT_CHAN     ADC_Channel_11
#define ADC_CURRENT_GPIO     GPIOC
#define ADC_CURRENT_PIN      GPIO_Pin_1
#define ADC_CURRENT_COEF     10.0f

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
#define configUSE_TIM1       1
#define configUSE_TIM1_CH1   1
#define configUSE_TIM1_CH2   0
#define configUSE_TIM1_CH3   0
#define configUSE_TIM1_CH4   0
#define configUSE_TIM2       1
#define configUSE_TIM2_CH1   1
#define configUSE_TIM2_CH2   1
#define configUSE_TIM2_CH3   1
#define configUSE_TIM2_CH4   1
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
#define configUSE_TIM12      1
#define configUSE_TIM12_CH1  1
#define configUSE_TIM12_CH2  0
#define configUSE_TIM12_CH3  0
#define configUSE_TIM12_CH4  0

/**********************************************************************************************************
*串口引脚及参数配置
**********************************************************************************************************/
#define USART1_GPIO             GPIOA
#define USART1_PINSOURCE_TX     GPIO_PinSource9
#define USART1_PINSOURCE_RX     GPIO_PinSource10
#define USART1_PIN_TX           GPIO_Pin_9
#define USART1_PIN_RX           GPIO_Pin_10
#define USART1_IRQ_PRIORITY     3

#define USART2_GPIO             GPIOA
#define USART2_PINSOURCE_TX     GPIO_PinSource2
#define USART2_PINSOURCE_RX     GPIO_PinSource3
#define USART2_PIN_TX           GPIO_Pin_2
#define USART2_PIN_RX           GPIO_Pin_3
#define USART2_IRQ_PRIORITY     3

#define USART3_GPIO             GPIOB
#define USART3_PINSOURCE_TX     GPIO_PinSource10
#define USART3_PINSOURCE_RX     GPIO_PinSource11
#define USART3_PIN_TX           GPIO_Pin_10
#define USART3_PIN_RX           GPIO_Pin_11
#define USART3_IRQ_PRIORITY     3

#define UART4_GPIO              GPIOA
#define UART4_PINSOURCE_TX      GPIO_PinSource0
#define UART4_PINSOURCE_RX      GPIO_PinSource1
#define UART4_PIN_TX            GPIO_Pin_0
#define UART4_PIN_RX            GPIO_Pin_1
#define UART4_IRQ_PRIORITY      3

#define UART5_GPIO              GPIOA
#define UART5_PINSOURCE_TX      GPIO_PinSource1
#define UART5_PINSOURCE_RX      GPIO_PinSource1
#define UART5_PIN_TX            GPIO_Pin_6
#define UART5_PIN_RX            GPIO_Pin_7
#define UART5_IRQ_PRIORITY      3

#define USART6_GPIO             GPIOC
#define USART6_PINSOURCE_TX     GPIO_PinSource6
#define USART6_PINSOURCE_RX     GPIO_PinSource7
#define USART6_PIN_TX           GPIO_Pin_6
#define USART6_PIN_RX           GPIO_Pin_7
#define USART6_IRQ_PRIORITY     3

/**********************************************************************************************************
*SPI引脚及参数配置
**********************************************************************************************************/
#define SPI1_GPIO_MOSI          GPIOA
#define SPI1_GPIO_MISO          GPIOA
#define SPI1_GPIO_SCK           GPIOA
#define SPI1_PINSOURCE_MOSI     GPIO_PinSource7
#define SPI1_PINSOURCE_MISO     GPIO_PinSource6
#define SPI1_PINSOURCE_SCK      GPIO_PinSource5
#define SPI1_PIN_MOSI           GPIO_Pin_7
#define SPI1_PIN_MISO           GPIO_Pin_6
#define SPI1_PIN_SCK            GPIO_Pin_5
#define SPI1_CLOCKDIV           SPI_BaudRatePrescaler_8

#define SPI2_GPIO_MOSI          GPIOC
#define SPI2_GPIO_MISO          GPIOC
#define SPI2_GPIO_SCK           GPIOB
#define SPI2_PINSOURCE_MOSI     GPIO_PinSource3
#define SPI2_PINSOURCE_MISO     GPIO_PinSource2
#define SPI2_PINSOURCE_SCK      GPIO_PinSource13
#define SPI2_PIN_MOSI           GPIO_Pin_3
#define SPI2_PIN_MISO           GPIO_Pin_2
#define SPI2_PIN_SCK            GPIO_Pin_13
#define SPI2_CLOCKDIV           SPI_BaudRatePrescaler_8

/**********************************************************************************************************
*软件I2C引脚及参数配置
**********************************************************************************************************/
#define SOFT_I2C1_GPIO          GPIOB
#define SOFT_I2C1_PIN_SCL       GPIO_Pin_6
#define SOFT_I2C1_PIN_SDA       GPIO_Pin_7
#define SOFT_I2C1_DELAY         0

#define SOFT_I2C2_GPIO          GPIOA
#define SOFT_I2C2_PIN_SCL       GPIO_Pin_7
#define SOFT_I2C2_PIN_SDA       GPIO_Pin_7
#define SOFT_I2C2_DELAY         0

/**********************************************************************************************************
*CAN引脚及参数配置
**********************************************************************************************************/
#define CAN1_GPIO               GPIOB
#define CAN1_PINSOURCE_RX       GPIO_PinSource8
#define CAN1_PINSOURCE_TX       GPIO_PinSource9
#define CAN1_PIN_RX             GPIO_Pin_8
#define CAN1_PIN_TX             GPIO_Pin_9

#define CAN2_GPIO               GPIOB
#define CAN2_PINSOURCE_RX       GPIO_PinSource8
#define CAN2_PINSOURCE_TX       GPIO_PinSource9
#define CAN2_PIN_RX             GPIO_Pin_8
#define CAN2_PIN_TX             GPIO_Pin_9

/**********************************************************************************************************
*定时器引脚及参数配置
**********************************************************************************************************/
#define TIM1_CLOCK                    PPM_TIM_FREQ
#define TIM1_PERIOD                   PPM_TIM_PERIOD
#define TIM1_IRQ_PRIORITY 			  3
#define TIM1_PWM_OUT                  0
#define TIM1_PPM_IN                   1
#if(configUSE_TIM1_CH1 == 1)
#define TIM1_CH1_GPIO             PPM_GPIO
#define TIM1_CH1_PIN              PPM_PIN
#define TIM1_CH1_PINSOURCE        PPM_PINSOURCE
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
#define TIM2_IRQ_PRIORITY 			  3
#define TIM2_PWM_OUT                  1
#define TIM2_PPM_IN                   0
#if(configUSE_TIM2_CH1 == 1)
#define TIM2_CH1_GPIO             GPIOA
#define TIM2_CH1_PIN              GPIO_Pin_15
#define TIM2_CH1_PINSOURCE        GPIO_PinSource15
#endif
#if(configUSE_TIM2_CH2 == 1)
#define TIM2_CH2_GPIO             GPIOB
#define TIM2_CH2_PIN              GPIO_Pin_3
#define TIM2_CH2_PINSOURCE        GPIO_PinSource3
#endif
#if(configUSE_TIM2_CH3 == 1)
#define TIM2_CH3_GPIO             GPIOA
#define TIM2_CH3_PIN              GPIO_Pin_2
#define TIM2_CH3_PINSOURCE        GPIO_PinSource2
#endif
#if(configUSE_TIM2_CH4 == 1)
#define TIM2_CH4_GPIO             GPIOA
#define TIM2_CH4_PIN              GPIO_Pin_3
#define TIM2_CH4_PINSOURCE        GPIO_PinSource3
#endif

#define TIM3_CLOCK                    PWM_TIM_FREQ
#define TIM3_PERIOD                   PWM_TIM_PERIOD
#define TIM3_IRQ_PRIORITY 			  3
#define TIM3_PWM_OUT                  1
#define TIM3_PPM_IN                   0
#if(configUSE_TIM3_CH1 == 1)
#define TIM3_CH1_GPIO             GPIOB
#define TIM3_CH1_PIN              GPIO_Pin_4
#define TIM3_CH1_PINSOURCE        GPIO_PinSource4
#endif
#if(configUSE_TIM3_CH2 == 1)
#define TIM3_CH2_GPIO             GPIOB
#define TIM3_CH2_PIN              GPIO_Pin_5
#define TIM3_CH2_PINSOURCE        GPIO_PinSource5
#endif
#if(configUSE_TIM3_CH3 == 1)
#define TIM3_CH3_GPIO             GPIOB
#define TIM3_CH3_PIN              GPIO_Pin_0
#define TIM3_CH3_PINSOURCE        GPIO_PinSource0
#endif
#if(configUSE_TIM3_CH4 == 1)
#define TIM3_CH4_GPIO             GPIOB
#define TIM3_CH4_PIN              GPIO_Pin_1
#define TIM3_CH4_PINSOURCE        GPIO_PinSource1
#endif

#define TIM4_CLOCK                    PPM_TIM_FREQ
#define TIM4_PERIOD                   PPM_TIM_PERIOD
#define TIM4_IRQ_PRIORITY 			  3
#define TIM4_PWM_OUT                  1
#define TIM4_PPM_IN                   0
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

#define TIM12_CLOCK                   TEMP_TIM_FREQ
#define TIM12_PERIOD                  TEMP_TIM_PERIOD
#define TIM12_IRQ_PRIORITY 			  3
#define TIM12_PWM_OUT                 1
#define TIM12_PPM_IN                  0
#if(configUSE_TIM12_CH1 == 1)
#define TIM12_CH1_GPIO            GPIOB
#define TIM12_CH1_PIN             GPIO_Pin_14
#define TIM12_CH1_PINSOURCE       GPIO_PinSource14
#endif
#if(configUSE_TIM12_CH2 == 1)
#define TIM12_CH2_GPIO            GPIOA
#define TIM12_CH2_PIN             GPIO_Pin_7
#define TIM12_CH2_PINSOURCE       GPIO_PinSource5
#endif
#if(configUSE_TIM12_CH3 == 1)
#define TIM12_CH3_GPIO            GPIOA
#define TIM12_CH3_PIN             GPIO_Pin_7
#define TIM12_CH3_PINSOURCE       GPIO_PinSource5
#endif
#if(configUSE_TIM12_CH4 == 1)
#define TIM12_CH4_GPIO            GPIOA
#define TIM12_CH4_PIN             GPIO_Pin_7
#define TIM12_CH4_PINSOURCE       GPIO_PinSource5
#endif

#endif


