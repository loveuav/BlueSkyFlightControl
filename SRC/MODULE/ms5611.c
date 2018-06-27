/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     ms5611.c
 * @说明     MS5611气压传感器驱动
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "ms5611.h"
#include "drv_spi.h"

#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8
#define MS5611_OSR				CMD_ADC_4096

typedef struct{
	int32_t baroAlt;
	int32_t pressure;
	int32_t temperature;	
	uint32_t ut;  // static result of temperature measurement
	uint32_t up;  // static result of pressure measurement
	uint16_t prom[PROM_NB];  // on-chip ROM
	uint8_t t_rxbuf[3];
	uint8_t p_rxbuf[3];
}MS5611_t;

static void MS5611_Reset(void);
static void MS5611_Read_Prom(void);
static void MS5611_Start_T(void);
static void MS5611_Start_P(void);
static void MS5611_Read_Adc_T(void);
static void MS5611_Read_Adc_P(void);
static void MS5611_BaroAltCalculate(void);

static MS5611_t ms5611;

/**********************************************************************************************************
*函 数 名: MS5611_Detect
*功能说明: 检测MS5611是否存在
*形    参: 无
*返 回 值: 存在状态
**********************************************************************************************************/
bool MS5611_Detect(void)
{
	return true;
}

/**********************************************************************************************************
*函 数 名: MS5611_Init
*功能说明: MS5611寄存器配置初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MS5611_Init(void)
{
	MS5611_Reset();
	SoftDelayMs(3);
	MS5611_Read_Prom();

	MS5611_Start_T();
}

/**********************************************************************************************************
*函 数 名: MS5611_Reset
*功能说明: MS5611复位
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void MS5611_Reset(void)
{
	Spi_BaroSingleWrite(CMD_RESET, 0x01);
}

/**********************************************************************************************************
*函 数 名: MS5611_Read_Prom
*功能说明: MS5611读取出厂校准参数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void MS5611_Read_Prom(void)
{
	uint8_t rxbuf[2] = { 0, 0 };

	for (u8 i = 0; i < PROM_NB; i++)
	{
		Spi_BaroMultiRead((CMD_PROM_RD + i * 2), rxbuf, 2);
		ms5611.prom[i] = rxbuf[0] << 8 | rxbuf[1];
	}
}

/**********************************************************************************************************
*函 数 名: MS5611_Read_Adc_T
*功能说明: MS5611读取温度测量值
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void MS5611_Read_Adc_T(void)
{
	Spi_BaroMultiRead(CMD_ADC_READ, ms5611.t_rxbuf, 3);
}

/**********************************************************************************************************
*函 数 名: MS5611_Read_Adc_P
*功能说明: MS5611读取气压测量值
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void MS5611_Read_Adc_P(void)
{
	Spi_BaroMultiRead(CMD_ADC_READ, ms5611.p_rxbuf, 3);
}

/**********************************************************************************************************
*函 数 名: MS5611_Start_T
*功能说明: MS5611发送温度测量命令
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void MS5611_Start_T(void)
{
	Spi_BaroSingleWrite(CMD_ADC_CONV + CMD_ADC_D2 + MS5611_OSR, 0x01);
}

/**********************************************************************************************************
*函 数 名: MS5611_Start_P
*功能说明: MS5611发送气压测量命令
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void MS5611_Start_P(void)
{
    Spi_BaroSingleWrite(CMD_ADC_CONV + CMD_ADC_D1 + MS5611_OSR, 0x01);
}

/**********************************************************************************************************
*函 数 名: MS5611_Update
*功能说明: MS5611数据更新
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MS5611_Update(void)
{
	static int state = 0;
	
	if (state) 
    {
			MS5611_Read_Adc_P();
			MS5611_Start_T();
			MS5611_BaroAltCalculate();
			state = 0;
	}
    else 
    {
			MS5611_Read_Adc_T();
			MS5611_Start_P();
			state = 1;
	}
}

/**********************************************************************************************************
*函 数 名: MS5611_BaroAltCalculate
*功能说明: MS5611数据校准及温度补偿，并将气压值转换为高度值
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void MS5611_BaroAltCalculate(void)
{
	int32_t  off2 = 0, sens2 = 0, delt;

	ms5611.ut = (ms5611.t_rxbuf[0] << 16) | (ms5611.t_rxbuf[1] << 8) | ms5611.t_rxbuf[2];
	ms5611.up = (ms5611.p_rxbuf[0] << 16) | (ms5611.p_rxbuf[1] << 8) | ms5611.p_rxbuf[2];

	int32_t dT = ms5611.ut - ((uint32_t)ms5611.prom[5] << 8);
	int64_t off = ((uint32_t)ms5611.prom[2] << 16) + (((int64_t)dT * ms5611.prom[4]) >> 7);
	int64_t sens = ((uint32_t)ms5611.prom[1] << 15) + (((int64_t)dT * ms5611.prom[3]) >> 8);
	ms5611.temperature = 2000 + (((int64_t)dT * ms5611.prom[6]) >> 23);

	if (ms5611.temperature < 2000)
    { // temperature lower than 20degC 
			delt = ms5611.temperature - 2000;
			delt = delt * delt;
			off2 = (5 * delt) >> 1;
			sens2 = (5 * delt) >> 2;
			if (ms5611.temperature < -1500)
            { // temperature lower than -15degC
					delt = ms5611.temperature + 1500;
					delt = delt * delt;
					off2  += 7 * delt;
					sens2 += (11 * delt) >> 1;
			}
	}
	off  -= off2; 
	sens -= sens2;
	ms5611.pressure = (((ms5611.up * sens ) >> 21) - off) >> 15;

	ms5611.baroAlt = (int32_t)((1.0f - pow(ms5611.pressure / 101325.0f, 0.190295f)) * 4433000.0f); // centimeter
}

/**********************************************************************************************************
*函 数 名: MS5611_Read
*功能说明: 读取MS5611的气压高度值
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void MS5611_Read(int32_t* baroAlt)
{
    *baroAlt = ms5611.baroAlt;
}

/**********************************************************************************************************
*函 数 名: MS5611_ReadTemp
*功能说明: 读取MS5611的温度值
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void MS5611_ReadTemp(float* temp)
{
    *temp = (float)ms5611.temperature * 0.01f;
}

