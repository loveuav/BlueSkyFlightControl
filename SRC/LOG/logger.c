/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     logger.c
 * @说明     飞行日志
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.08
**********************************************************************************************************/
#include "logger.h"
#include "ulog.h"

#include "ff.h"		
#include "diskio.h"	
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "flightStatus.h"

FATFS fs;
FIL   file;

uint8_t cardExistFlag = 0;

void LoggerTest(void);

/**********************************************************************************************************
*函 数 名: LoggerInit
*功能说明: 飞行日志初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void LoggerInit(void)
{
    FRESULT fresult;
    TCHAR testName[] = "0:init.test";   

    //挂载文件系统
    f_mount(0, &fs);
    
    //打开文件
    for(uint8_t i=10; i>0; i--)
    {
        fresult = f_open(&file, testName, FA_OPEN_ALWAYS | FA_WRITE);
        
        if(fresult == FR_OK)
            break;
        
        OsDelayMs(50);
    }
    
    if(fresult == FR_OK)
    {
        cardExistFlag = 1;
        //关闭文件
        fresult = f_close(&file);
        //删除测试文件
        f_unlink(testName);
        //创建日志文件夹
        fresult = f_mkdir("ulog");
    }
    else
    {
        cardExistFlag = 0;
    }
}

/**********************************************************************************************************
*函 数 名: LoggerLoop
*功能说明: 日志记录主循环
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void LoggerLoop(void)
{  
    FRESULT fresult;
    static TCHAR fileName[] = "0:ulog/bluesky.ulg";
    static uint8_t state = 0;
    static uint32_t cnt = 0;
    
    //未检测到tf卡或初始化不成功则退出
    if(!cardExistFlag)
        return;

    //解锁后开始记录log
    if(GetArmedStatus() == ARMED && state == 0)
        state = 1;
    
    switch(state)
    {
        /*打开文件*/
        case 1:
            fresult = f_open(&file, fileName, FA_CREATE_ALWAYS | FA_WRITE);
            if(fresult == FR_OK)
                state++;
            else
                cardExistFlag = 0;
            break;   

        /*写入log头部信息*/
        case 2:
            UlogWriteHeader();
            UlogWriteFlag();
            UlogWriteFormat();
            UlogWriteAddLogged();
            state++;
            break; 

        /*写入飞控数据*/
        case 3:
            UlogWriteData();
            //固定间隔刷新文件缓存
            if(cnt++ % 300 == 0)
                f_sync(&file); 
            //上锁后停止记录log并关闭文件
            if(GetArmedStatus() == DISARMED)
            {
                f_close(&file);
                state = 0;
            }
            break;
        
        default:
            break;
    }
}

/**********************************************************************************************************
*函 数 名: LoggerWrite
*功能说明: 往打开的日志文件里写入数据
*形    参: 数据指针 数据长度
*返 回 值: 无
**********************************************************************************************************/
void LoggerWrite(void *data, uint16_t size)
{  
    UINT btw;
    
    f_write(&file, data, size, &btw);
}



