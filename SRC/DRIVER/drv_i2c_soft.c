/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     drv_i2c_soft.c
 * @说明     软件I2C驱动
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "drv_i2c_soft.h"

/**********************************************************************************************************
*函 数 名: I2c_Soft_Open
*功能说明: 软件IIC初始化
*形    参: 设备号
*返 回 值: 无
**********************************************************************************************************/
void Soft_I2c_Open(uint8_t deviceNum)
{
    GPIO_InitTypeDef  GPIO_InitStructure; 
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    
    if(deviceNum == 1)
    {
        GPIO_InitStructure.GPIO_Pin =  SOFT_I2C1_PIN_SCL | SOFT_I2C1_PIN_SDA;
        GPIO_Init(SOFT_I2C1_GPIO, &GPIO_InitStructure);	
    }  
    else if(deviceNum == 2)
    {
        GPIO_InitStructure.GPIO_Pin =  SOFT_I2C2_PIN_SCL | SOFT_I2C2_PIN_SDA;
        GPIO_Init(SOFT_I2C2_GPIO, &GPIO_InitStructure);	
    }      
}

/**********************************************************************************************************
*函 数 名: Soft_I2c_Delay
*功能说明: 软件IIC延时函数
*形    参: 设备号
*返 回 值: 无
**********************************************************************************************************/
void Soft_I2c_Delay(uint8_t deviceNum)
{
    uint8_t i;
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	__nop();__nop();__nop();

    if(deviceNum == 1)
    {
        i = SOFT_I2C1_DELAY;    
    }  
    else if(deviceNum == 2)
    {
        i = SOFT_I2C2_DELAY;
    }  	

    while(i--);
}

/**********************************************************************************************************
*函 数 名: Soft_I2c_SCL_H
*功能说明: SCL引脚拉高
*形    参: 设备号
*返 回 值: 无
**********************************************************************************************************/
void Soft_I2c_SCL_H(uint8_t deviceNum)
{
    if(deviceNum == 1)
    {
        SOFT_I2C1_GPIO->BSRRL = SOFT_I2C1_PIN_SCL;       
    }  
    else if(deviceNum == 2)
    {
        SOFT_I2C2_GPIO->BSRRL = SOFT_I2C2_PIN_SCL;
    }  
}

/**********************************************************************************************************
*函 数 名: Soft_I2c_SCL_L
*功能说明: SCL引脚拉低
*形    参: 设备号
*返 回 值: 无
**********************************************************************************************************/
void Soft_I2c_SCL_L(uint8_t deviceNum)
{
    if(deviceNum == 1)
    {
        SOFT_I2C1_GPIO->BSRRH = SOFT_I2C1_PIN_SCL;       
    }  
    else if(deviceNum == 2)
    {
        SOFT_I2C2_GPIO->BSRRH = SOFT_I2C2_PIN_SCL;
    } 
}

/**********************************************************************************************************
*函 数 名: Soft_I2c_SDA_L
*功能说明: SDA引脚拉高
*形    参: 设备号
*返 回 值: 无
**********************************************************************************************************/
void Soft_I2c_SDA_H(uint8_t deviceNum)
{
    if(deviceNum == 1)
    {
        SOFT_I2C1_GPIO->BSRRL = SOFT_I2C1_PIN_SDA;       
    }  
    else if(deviceNum == 2)
    {
        SOFT_I2C2_GPIO->BSRRL = SOFT_I2C2_PIN_SDA;
    }  
}

/**********************************************************************************************************
*函 数 名: Soft_I2c_SDA_L
*功能说明: SDA引脚拉低
*形    参: 设备号
*返 回 值: 无
**********************************************************************************************************/
void Soft_I2c_SDA_L(uint8_t deviceNum)
{
    if(deviceNum == 1)
    {
        SOFT_I2C1_GPIO->BSRRH = SOFT_I2C1_PIN_SDA;       
    }  
    else if(deviceNum == 2)
    {
        SOFT_I2C2_GPIO->BSRRH = SOFT_I2C2_PIN_SDA;
    } 
}

/**********************************************************************************************************
*函 数 名: Soft_I2c_SCL_Read
*功能说明: 读取SCL引脚状态：0或1
*形    参: 设备号
*返 回 值: 无
**********************************************************************************************************/
uint8_t Soft_I2c_SCL_Read(uint8_t deviceNum)
{
    if(deviceNum == 1)
    {
        if(!(SOFT_I2C1_GPIO->IDR & SOFT_I2C1_PIN_SCL))
            return 0;
        else
            return 1;
    }  
    else if(deviceNum == 2)
    {
        if(!(SOFT_I2C2_GPIO->IDR & SOFT_I2C2_PIN_SCL))
            return 0;
        else
            return 1;
    } 
    else
    {
        return 0;
    }
}

/**********************************************************************************************************
*函 数 名: Soft_I2c_SDA_Read
*功能说明: 读取SDA引脚状态：0或1
*形    参: 设备号
*返 回 值: 无
**********************************************************************************************************/
uint8_t Soft_I2c_SDA_Read(uint8_t deviceNum)
{
    if(deviceNum == 1)
    {
        if(!(SOFT_I2C1_GPIO->IDR & SOFT_I2C1_PIN_SDA))
            return 0;
        else
            return 1;
    }  
    else if(deviceNum == 2)
    {
        if(!(SOFT_I2C2_GPIO->IDR & SOFT_I2C2_PIN_SDA))
            return 0;
        else
            return 1;
    } 
    else
    {
        return 0;
    }
}

/**********************************************************************************************************
*函 数 名: Soft_I2c_Start
*功能说明: 发送I2C总线启动信号
*形    参: 设备号
*返 回 值: 无
**********************************************************************************************************/
bool Soft_I2c_Start(uint8_t deviceNum)
{
	Soft_I2c_SDA_H(deviceNum);
	Soft_I2c_SCL_H(deviceNum);
	Soft_I2c_Delay(deviceNum);
	if(!Soft_I2c_SDA_Read(deviceNum))
        return 0;
	Soft_I2c_SDA_L(deviceNum);
	Soft_I2c_Delay(deviceNum);
	if(Soft_I2c_SDA_Read(deviceNum)) 
        return 0;	
	Soft_I2c_SDA_L(deviceNum);
	Soft_I2c_Delay(deviceNum);
	return 1;	
}

/**********************************************************************************************************
*函 数 名: Soft_I2c_Stop
*功能说明: 发送I2C总线停止信号
*形    参: 设备号
*返 回 值: 无
**********************************************************************************************************/
void Soft_I2c_Stop(uint8_t deviceNum)
{
	Soft_I2c_SCL_L(deviceNum);
	Soft_I2c_Delay(deviceNum);
	Soft_I2c_SDA_L(deviceNum);
	Soft_I2c_Delay(deviceNum);
	Soft_I2c_SCL_H(deviceNum);
	Soft_I2c_Delay(deviceNum);
	Soft_I2c_SDA_H(deviceNum);
	Soft_I2c_Delay(deviceNum);
} 

/**********************************************************************************************************
*函 数 名: Soft_I2c_Ack
*功能说明: 发送I2C应答信号
*形    参: 设备号
*返 回 值: 无
**********************************************************************************************************/
void Soft_I2c_Ack(uint8_t deviceNum)
{	
	Soft_I2c_SCL_L(deviceNum);
	Soft_I2c_Delay(deviceNum);
	Soft_I2c_SDA_L(deviceNum);
	Soft_I2c_Delay(deviceNum);
	Soft_I2c_SCL_H(deviceNum);
	Soft_I2c_Delay(deviceNum);
	Soft_I2c_SCL_L(deviceNum);
	Soft_I2c_Delay(deviceNum);
}   

/**********************************************************************************************************
*函 数 名: Soft_I2c_NoAck
*功能说明: 发送I2C非应答信号
*形    参: 设备号
*返 回 值: 无
**********************************************************************************************************/
void Soft_I2c_NoAck(uint8_t deviceNum)
{	
	Soft_I2c_SCL_L(deviceNum);
	Soft_I2c_Delay(deviceNum);
	Soft_I2c_SDA_H(deviceNum);
	Soft_I2c_Delay(deviceNum);
	Soft_I2c_SCL_H(deviceNum);
	Soft_I2c_Delay(deviceNum);
	Soft_I2c_SCL_L(deviceNum);
	Soft_I2c_Delay(deviceNum);
} 

/**********************************************************************************************************
*函 数 名: Soft_I2c_WaitAck
*功能说明: 等待从机应答信号
*形    参: 设备号
*返 回 值: 应答状态——1表示有应答
**********************************************************************************************************/
bool Soft_I2c_WaitAck(uint8_t deviceNum) 	
{
	Soft_I2c_SCL_L(deviceNum);
	Soft_I2c_Delay(deviceNum);
	Soft_I2c_SDA_H(deviceNum);			
	Soft_I2c_Delay(deviceNum);
	Soft_I2c_SCL_H(deviceNum);
	Soft_I2c_Delay(deviceNum);
	if(Soft_I2c_SDA_Read(deviceNum))
	{
        Soft_I2c_SCL_L(deviceNum);
        Soft_I2c_Delay(deviceNum);
        return 0;
	}
	Soft_I2c_SCL_L(deviceNum);
	Soft_I2c_Delay(deviceNum);
	return 1;
}

/**********************************************************************************************************
*函 数 名: Soft_I2c_SendByte
*功能说明: 发送一个字节
*形    参: 设备号
*返 回 值: 无
**********************************************************************************************************/
void Soft_I2c_SendByte(uint8_t deviceNum, u8 SendByte) 
{
    u8 i=8;
    while(i--)
    {
        Soft_I2c_SCL_L(deviceNum);
        Soft_I2c_Delay(deviceNum);
        if(SendByte&0x80)
            Soft_I2c_SDA_H(deviceNum);  
        else 
            Soft_I2c_SDA_L(deviceNum);   
        SendByte<<=1;
        Soft_I2c_Delay(deviceNum);
        Soft_I2c_SCL_H(deviceNum);
        Soft_I2c_Delay(deviceNum);
    }
    Soft_I2c_SCL_L(deviceNum);
}  

/**********************************************************************************************************
*函 数 名: Soft_I2c_ReadByt
*功能说明: 读取一个字节
*形    参: 设备号
*返 回 值: 读取到的数据
**********************************************************************************************************/
uint8_t Soft_I2c_ReadByte(uint8_t deviceNum) 
{ 
    uint8_t i=8;
    uint8_t ReceiveByte=0;

    Soft_I2c_SDA_H(deviceNum);				
    while(i--)
    {
        ReceiveByte<<=1;      
        Soft_I2c_SCL_L(deviceNum);
        Soft_I2c_Delay(deviceNum);
        Soft_I2c_SCL_H(deviceNum);
        Soft_I2c_Delay(deviceNum);	
        if(Soft_I2c_SDA_Read(deviceNum))
        {
            ReceiveByte|=0x01;
        }
    }
    Soft_I2c_SCL_L(deviceNum);
    return ReceiveByte;
} 

/**********************************************************************************************************
*函 数 名: Soft_I2c_Single_Write
*功能说明: 单个寄存器写入
*形    参: 设备号 从机地址 寄存器地址 写入数据
*返 回 值: 写入状态
**********************************************************************************************************/
bool Soft_I2c_Single_Write(uint8_t deviceNum, u8 SlaveAddress,u8 REG_Address,u8 REG_data)		
{
  	if(!Soft_I2c_Start(deviceNum))
        return false;
    Soft_I2c_SendByte(deviceNum, SlaveAddress);  
    if(!Soft_I2c_WaitAck(deviceNum))
    {
        Soft_I2c_Stop(deviceNum); 
        return false;
    }
    Soft_I2c_SendByte(deviceNum, REG_Address);        
    Soft_I2c_WaitAck(deviceNum);	
    Soft_I2c_SendByte(deviceNum, REG_data);
    Soft_I2c_WaitAck(deviceNum);   
    Soft_I2c_Stop(deviceNum); 
    return true;
}

/**********************************************************************************************************
*函 数 名: Soft_I2c_Single_Read
*功能说明: 单个寄存器读取
*形    参: 设备号 从机地址 寄存器地址
*返 回 值: 读出数据
**********************************************************************************************************/
uint8_t Soft_I2C_Single_Read(uint8_t deviceNum, u8 SlaveAddress,u8 REG_Address)
{   
    uint8_t REG_data;     	
    if(!Soft_I2c_Start(deviceNum))
        return false;
    Soft_I2c_SendByte(deviceNum, SlaveAddress);  
    if(!Soft_I2c_WaitAck(deviceNum))
    {
        Soft_I2c_Stop(deviceNum);
        return false;
    }
    Soft_I2c_SendByte(deviceNum, (u8)REG_Address); 
    Soft_I2c_WaitAck(deviceNum);
    Soft_I2c_Start(deviceNum);
    Soft_I2c_SendByte(deviceNum, SlaveAddress+1);
    Soft_I2c_WaitAck(deviceNum);

	REG_data = Soft_I2c_ReadByte(deviceNum);
    Soft_I2c_NoAck(deviceNum);
    Soft_I2c_Stop(deviceNum);
    return REG_data;
}	

/**********************************************************************************************************
*函 数 名: Soft_I2C_Multi_Read
*功能说明: 多个寄存器连续读取
*形    参: 设备号 从机地址 寄存器地址 读出缓冲区指针 读出长度
*返 回 值: 读取状态
**********************************************************************************************************/
bool Soft_I2C_Multi_Read(uint8_t deviceNum, u8 SlaveAddress,u8 REG_Address,u8 * ptChar,u8 size)
{
    uint8_t i;
    
    if(size < 1)
        return false;
    if(!Soft_I2c_Start(deviceNum))
		return false;
    Soft_I2c_SendByte(deviceNum, SlaveAddress);
    if(!Soft_I2c_WaitAck(deviceNum))
    {
        Soft_I2c_Stop(deviceNum);
        return false;
    }
    Soft_I2c_SendByte(deviceNum, REG_Address);    
    Soft_I2c_WaitAck(deviceNum);
    
    Soft_I2c_Start(deviceNum);
    Soft_I2c_SendByte(deviceNum, SlaveAddress+1);
    Soft_I2c_WaitAck(deviceNum);
    
    for(i=1;i<size; i++)
    {
        *ptChar++ = Soft_I2c_ReadByte(deviceNum);
        Soft_I2c_Ack(deviceNum);
    }
    *ptChar++ = Soft_I2c_ReadByte(deviceNum);
    Soft_I2c_NoAck(deviceNum);
    Soft_I2c_Stop(deviceNum);
    return true;    
}	

