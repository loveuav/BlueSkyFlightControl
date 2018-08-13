/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     drv_adc.c
 * @说明     ADC驱动
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.06 
**********************************************************************************************************/
#include "drv_adc.h"

/**********************************************************************************************************
*函 数 名: Adc_Init
*功能说明: ADC初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Adc_Init(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructyre;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    
    GPIO_InitStructure.GPIO_Pin = ADC_VOLTAGE_PIN;
    GPIO_Init(ADC_VOLTAGE_GPIO, &GPIO_InitStructure);    
    
    GPIO_InitStructure.GPIO_Pin = ADC_CURRENT_PIN;
    GPIO_Init(ADC_CURRENT_GPIO, &GPIO_InitStructure); 

    ADC_DeInit();

    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8; 
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);    
    
    ADC_InitStructyre.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructyre.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructyre.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructyre.ADC_NbrOfConversion = 1;
    ADC_InitStructyre.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructyre.ADC_ScanConvMode = ENABLE; 

    ADC_Init(ADC_VOLTAGE, &ADC_InitStructyre);
    ADC_Cmd(ADC_VOLTAGE, ENABLE);
    ADC_RegularChannelConfig(ADC_VOLTAGE, ADC_VOLTAGE_CHAN, 1, ADC_SampleTime_144Cycles);
    ADC_SoftwareStartConv(ADC_VOLTAGE);
    
    ADC_Init(ADC_CURRENT, &ADC_InitStructyre);
    ADC_Cmd(ADC_CURRENT, ENABLE);
    ADC_RegularChannelConfig(ADC_CURRENT, ADC_CURRENT_CHAN, 1, ADC_SampleTime_144Cycles);
    ADC_SoftwareStartConv(ADC_CURRENT);    
}

/**********************************************************************************************************
*函 数 名: GetVoltageAdcValue
*功能说明: 获取电压ADC采样值
*形    参: 无
*返 回 值: ADC采样值
**********************************************************************************************************/
uint16_t GetVoltageAdcValue(void)
{
	static uint16_t adcTemp;

    adcTemp = ADC_GetConversionValue(ADC_VOLTAGE) >> 4;
    
	return (adcTemp * 330 / 0xFFF);
}

/**********************************************************************************************************
*函 数 名: GetCurrentAdcValue
*功能说明: 获取电流ADC采样值
*形    参: 无
*返 回 值: ADC采样值
**********************************************************************************************************/
uint16_t GetCurrentAdcValue(void)
{
	static uint16_t adcTemp;
	
	adcTemp = ADC_GetConversionValue(ADC_CURRENT) >> 4;
    
	return (adcTemp * 330 / 0xFFF);
}







