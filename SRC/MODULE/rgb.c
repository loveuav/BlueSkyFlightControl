/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     rgb.c
 * @说明     RGB驱动
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.11
**********************************************************************************************************/
#include "rgb.h"
#include "board.h"
#include "flightStatus.h"
#include "sensor.h"

/*
Blue  - PA6
Green - PC4
Red   - PC5
*/
#define LED_BLUE_GPIO     GPIOA
#define LED_BLUE_PIN      GPIO_Pin_6
#define LED_GREEN_GPIO    GPIOC
#define LED_GREEN_PIN     GPIO_Pin_4
#define LED_RED_GPIO      GPIOC
#define LED_RED_PIN       GPIO_Pin_5

void RGB_Green_On(void)
{
    GPIO_ResetBits(LED_GREEN_GPIO, LED_GREEN_PIN);
}

void RGB_Green_Off(void)
{
    GPIO_SetBits(LED_GREEN_GPIO, LED_GREEN_PIN);
}

void RGB_Red_On(void)
{
    GPIO_ResetBits(LED_RED_GPIO, LED_RED_PIN);
}

void RGB_Red_Off(void)
{
    GPIO_SetBits(LED_RED_GPIO, LED_RED_PIN);
}

void RGB_Blue_On(void)
{
    GPIO_ResetBits(LED_BLUE_GPIO, LED_BLUE_PIN);
}

void RGB_Blue_Off(void)
{
    GPIO_SetBits(LED_BLUE_GPIO, LED_BLUE_PIN);
}

/**********************************************************************************************************
*函 数 名: RGB_Init
*功能说明: RGB初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void RGB_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_InitStructure.GPIO_Pin = LED_BLUE_PIN;
    GPIO_Init(LED_BLUE_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = LED_GREEN_PIN;
    GPIO_Init(LED_GREEN_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = LED_RED_PIN;
    GPIO_Init(LED_RED_GPIO, &GPIO_InitStructure);

    RGB_Green_Off();
    RGB_Red_Off();
    RGB_Blue_Off();
}

/**********************************************************************************************************
*函 数 名: RGB_Flash
*功能说明: RGB闪烁 运行频率200Hz
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void RGB_Flash(void)
{
    static uint32_t cnt = 0;
    static uint8_t initFlag = 0;
    
    switch(GetInitStatus())
    {
    case HEATING:
        if(initFlag == 0)
        {
            if(cnt > 250)
            {
                initFlag = 1;
            }
            else if(cnt > 150)
            {
                RGB_Green_Off();
                RGB_Red_Off();
                RGB_Blue_Off(); 
            }
            else if(cnt > 100)
            {
                RGB_Green_On();
                RGB_Red_Off();
                RGB_Blue_Off(); 
            }
            else if(cnt > 50)
            {
                RGB_Green_Off();
                RGB_Red_On();
                RGB_Blue_Off(); 
            }
            else
            {
                RGB_Green_Off();
                RGB_Red_Off();
                RGB_Blue_On();
            }
        }
        else
        { 
            //传感器检测正常则红灯慢闪，不正常快闪
            if(SensorCheckStatus())
            {
                if(cnt % 100 == 0)
                {
                    RGB_Green_Off();
                    RGB_Red_On();
                    RGB_Blue_Off(); 
                }
                if(cnt % 200 == 0)
                {
                    RGB_Green_Off();
                    RGB_Red_Off();
                    RGB_Blue_Off(); 
                }
            }
            else
            {
                if(cnt % 10 == 0)
                {
                    RGB_Green_Off();
                    RGB_Red_On();
                    RGB_Blue_Off(); 
                }
                if(cnt % 20 == 0)
                {
                    RGB_Green_Off();
                    RGB_Red_Off();
                    RGB_Blue_Off(); 
                }
            }
        }
        break;

    case HEAT_FINISH:
            if(cnt % 100 == 0)
            {
                RGB_Green_Off();
                RGB_Red_Off();
                RGB_Blue_On(); 
            }
            if(cnt % 200 == 0)
            {
                RGB_Green_Off();
                RGB_Red_Off();
                RGB_Blue_Off(); 
            }
        break;

    case INIT_FINISH:
            if(cnt % 10 == 0)
            {
                RGB_Green_Off();
                RGB_Red_Off();
                RGB_Blue_Off(); 
            }
            if(cnt % 300 == 0)
            {
                RGB_Green_On();
                RGB_Red_Off();
                RGB_Blue_Off(); 
            }
        break;

    default:
        break;
    }
    
    cnt++;
}





