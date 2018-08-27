/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     drv_spi.c
 * @说明     SPI驱动
 * @版本  	 V1.0.1
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "drv_spi.h"

/**********************************************************************************************************
*函 数 名: Spi_GPIO_Init
*功能说明: SPI从机设备CS引脚初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Spi_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	

    GPIO_InitStructure.GPIO_Pin = GYRO_CS_PIN;     
	GPIO_Init(GYRO_CS_GPIO, &GPIO_InitStructure);    

    GPIO_InitStructure.GPIO_Pin = GYRO2_CS_PIN;     
	GPIO_Init(GYRO2_CS_GPIO, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = BARO_CS_PIN;     
	GPIO_Init(BARO_CS_GPIO, &GPIO_InitStructure);  

    GPIO_SetBits(GYRO_CS_GPIO, GYRO_CS_PIN);
    GPIO_SetBits(GYRO2_CS_GPIO, GYRO2_CS_PIN);
    GPIO_SetBits(BARO_CS_GPIO, BARO_CS_PIN);    
}

/**********************************************************************************************************
*函 数 名: Spi_Open
*功能说明: SPI初始化
*形    参: 设备号
*返 回 值: 无
**********************************************************************************************************/
void Spi_Open(uint8_t deviceNum)
{
    SPI_InitTypeDef SPI_InitStructure; 
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; 
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; 
	SPI_InitStructure.SPI_CRCPolynomial = 7; 
	
	if(deviceNum == 1)
	{  
		GPIO_PinAFConfig(SPI1_GPIO_MOSI, SPI1_PINSOURCE_MOSI, GPIO_AF_SPI1);
		GPIO_PinAFConfig(SPI1_GPIO_MISO, SPI1_PINSOURCE_MISO, GPIO_AF_SPI1);
		GPIO_PinAFConfig(SPI1_GPIO_SCK, SPI1_PINSOURCE_SCK, GPIO_AF_SPI1);
		GPIO_InitStructure.GPIO_Pin = SPI1_PIN_MOSI;
		GPIO_Init(SPI1_GPIO_MOSI, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin =  SPI1_PIN_MISO;
		GPIO_Init(SPI1_GPIO_MISO, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin =  SPI1_PIN_SCK;
		GPIO_Init(SPI1_GPIO_SCK, &GPIO_InitStructure);		
        
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI1_CLOCKDIV;
        SPI_Init(SPI1, &SPI_InitStructure); 
        SPI_Cmd(SPI1, ENABLE);
	}
    else if(deviceNum == 2)
    {
		GPIO_PinAFConfig(SPI2_GPIO_MOSI, SPI2_PINSOURCE_MOSI, GPIO_AF_SPI2);
		GPIO_PinAFConfig(SPI2_GPIO_MISO, SPI2_PINSOURCE_MISO, GPIO_AF_SPI2);
		GPIO_PinAFConfig(SPI2_GPIO_SCK, SPI2_PINSOURCE_SCK, GPIO_AF_SPI2);
		GPIO_InitStructure.GPIO_Pin = SPI2_PIN_MOSI;
		GPIO_Init(SPI2_GPIO_MOSI, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin =  SPI2_PIN_MISO;
		GPIO_Init(SPI2_GPIO_MISO, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin =  SPI2_PIN_SCK;
		GPIO_Init(SPI2_GPIO_SCK, &GPIO_InitStructure);	  
        
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI2_CLOCKDIV;
        SPI_Init(SPI2, &SPI_InitStructure); 
        SPI_Cmd(SPI2, ENABLE);        
    }
}


/**********************************************************************************************************
*函 数 名: Spi_SingleWirteAndRead
*功能说明: SPI单字节读取
*形    参: 设备号 写入的数据
*返 回 值: 读取到的数据
**********************************************************************************************************/
uint8_t Spi_SingleWirteAndRead(uint8_t deviceNum, uint8_t dat) 
{ 
    if(deviceNum == 1)
    {
        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); 
        SPI_I2S_SendData(SPI1, dat); 
        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 
        return SPI_I2S_ReceiveData(SPI1);         
    }
    else if(deviceNum == 2)
    {
        while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET); 
        SPI_I2S_SendData(SPI2, dat); 
        while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET); 
        return SPI_I2S_ReceiveData(SPI2);             
    }
    else
    {
        return 0;
    }
}

/**********************************************************************************************************
*函 数 名: SPI_MultiWriteAndRead
*功能说明: SPI多字节读取
*形    参: 设备号 写入数据缓冲区指针 读出数据缓冲区指针 数据长度
            同时只能写入或者读出，写入时读取缓冲区设置为NULL，读出时反之
*返 回 值: 无
**********************************************************************************************************/
void SPI_MultiWriteAndRead(uint8_t deviceNum, uint8_t *out, uint8_t *in, int len)
{
    uint8_t b;
    if(deviceNum == 1)
    {    
        while (len--) 
        {
            b = in ? *(in++) : 0xFF;		
            while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
            SPI_I2S_SendData(SPI1, b); 
            while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 
            b = SPI_I2S_ReceiveData(SPI1); 
            if (out)
                *(out++) = b;
        }
    }
    else if(deviceNum == 2)
    {
        while (len--) 
        {
            b = in ? *(in++) : 0xFF;		
            while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
            SPI_I2S_SendData(SPI2, b); 
            while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET); 
            b = SPI_I2S_ReceiveData(SPI2); 
            if (out)
                *(out++) = b;
        }
    }
}

/**********************************************************************************************************
*函 数 名: Spi_GyroEnable
*功能说明: 陀螺仪CS脚使能
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Spi_GyroEnable(void)
{
    GPIO_ResetBits(GYRO_CS_GPIO, GYRO_CS_PIN);
}

/**********************************************************************************************************
*函 数 名: Spi_GyroDisable
*功能说明: 陀螺仪CS脚失能
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Spi_GyroDisable(void)
{
    GPIO_SetBits(GYRO_CS_GPIO, GYRO_CS_PIN);
}

/**********************************************************************************************************
*函 数 名: Spi_GyroSingleWrite
*功能说明: 陀螺仪单个寄存器写入
*形    参: 寄存器地址 写入值
*返 回 值: 无
**********************************************************************************************************/
void Spi_GyroSingleWrite(uint8_t reg, uint8_t value)
{
	Spi_GyroEnable();
	Spi_SingleWirteAndRead(GYRO_SPI, reg);
	Spi_SingleWirteAndRead(GYRO_SPI, value); 
	Spi_GyroDisable();	
}

/**********************************************************************************************************
*函 数 名: Spi_GyroMultiRead
*功能说明: 陀螺仪多个寄存器读出
*形    参: 寄存器地址 读出缓冲区 读出长度
*返 回 值: 无
**********************************************************************************************************/
void Spi_GyroMultiRead(uint8_t reg,uint8_t *data, uint8_t length)
{
	Spi_GyroEnable();
	Spi_SingleWirteAndRead(GYRO_SPI, reg | 0x80);
	SPI_MultiWriteAndRead(GYRO_SPI, data, NULL, length);	
	Spi_GyroDisable();	
}

/**********************************************************************************************************
*函 数 名: Spi_BaroEnable
*功能说明: 气压计CS脚使能
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Spi_BaroEnable(void)
{
    GPIO_ResetBits(BARO_CS_GPIO, BARO_CS_PIN);
}

/**********************************************************************************************************
*函 数 名: Spi_BaroDisable
*功能说明: 气压计CS脚失能
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Spi_BaroDisable(void)
{
    GPIO_SetBits(BARO_CS_GPIO, BARO_CS_PIN);
}

/**********************************************************************************************************
*函 数 名: Spi_BaroSingleWrite
*功能说明: 气压计单个寄存器写入
*形    参: 寄存器地址 写入值
*返 回 值: 无
**********************************************************************************************************/
void Spi_BaroSingleWrite(uint8_t reg, uint8_t value)
{
	Spi_BaroEnable();
	Spi_SingleWirteAndRead(BARO_SPI, reg);
	Spi_SingleWirteAndRead(BARO_SPI, value); 
	Spi_BaroDisable();	
}

/**********************************************************************************************************
*函 数 名: Spi_BaroMultiRead
*功能说明: 气压计多个寄存器读出
*形    参: 寄存器地址 读出缓冲区 读出长度
*返 回 值: 无
**********************************************************************************************************/
void Spi_BaroMultiRead(uint8_t reg,uint8_t *data, uint8_t length)
{
	Spi_BaroEnable();
	Spi_SingleWirteAndRead(BARO_SPI, reg);
	SPI_MultiWriteAndRead(BARO_SPI, data, NULL, length);	
	Spi_BaroDisable();	
}

