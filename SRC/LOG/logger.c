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
#include "ulog_data.h"

#include "ff.h"		
#include "diskio.h"	
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "flightStatus.h"
#include "ublox.h"

#define LOG_NAME_SIZE 19

FATFS fs;
FIL   file;

uint8_t cardExistFlag = 0;

static void GetLogNameByTime(TCHAR* name);

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
    static TCHAR logPath[LOG_NAME_SIZE+10];
    static TCHAR logName[LOG_NAME_SIZE];
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
        /*创建文件*/
        case 1:
            //根据当前时间获取文件名
            GetLogNameByTime(logName);  

            //连接文件路径与文件名
            memset(logPath, 0, LOG_NAME_SIZE+10);
            strcpy(logPath, "0:ulog/");
            strcat(logPath, logName);
        
            //创建文件
            fresult = f_open(&file, logPath, FA_CREATE_ALWAYS | FA_WRITE);
        
            if(fresult == FR_OK)
                state++;
            else
                cardExistFlag = 0;
            break;   

        /*写入log头部信息*/
        case 2:
            //头部
            UlogWriteHeader();
            //数据格式定义
            UlogWriteFormat("flight", ulog_data_flight, ULOG_DATA_FLIGHT_NUM);
            UlogWriteFormat("gps", ulog_data_gps, ULOG_DATA_GPS_NUM);
            //订阅数据
            UlogWriteAddLogged("flight", ULOG_DATA_FLIGHT_ID);
            UlogWriteAddLogged("gps", ULOG_DATA_GPS_ID);
            state++;
            break; 

        /*记录飞控数据*/
        case 3:	  
            //记录基本飞行数据
            UlogWriteData_Flight();

            //记录GPS数据:10Hz
            if(cnt % (LOG_RATE / 10) == 0)
                UlogWriteData_GPS();
        
            //固定间隔刷新文件缓存:5秒
            if(cnt % (LOG_RATE * 5) == 0)
                fresult = f_sync(&file); 
            
            //上锁后停止记录log并关闭文件
            if(GetArmedStatus() == DISARMED)
            {
                fresult = f_close(&file);
                state = 0;
            }
            
            cnt++;
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

/**********************************************************************************************************
*函 数 名: GetLogNameByTime
*功能说明: 根据当前时间获取日志文件名
*形    参: 数据指针 数据长度
*返 回 值: 无
**********************************************************************************************************/
static void GetLogNameByTime(TCHAR* name)
{  
    UTC_TIME_t time = GetUTCTime();
    
    name[0] = (time.year / 1000) + '0';
    name[1] = ((time.year / 100) % 10) + '0';
    name[2] = ((time.year / 10) % 10) + '0';
    name[3] = (time.year % 10) + '0';
    name[4] = (time.month / 10) + '0';
    name[5] = (time.month % 10) + '0';
    name[6] = (time.day / 10) + '0';
    name[7] = (time.day % 10) + '0';
    name[8] = '_';
    name[9] = (time.hour / 10) + '0';
    name[10] = (time.hour % 10) + '0';
    name[11] = (time.min / 10) + '0';
    name[12] = (time.min % 10) + '0';
    name[13] = (time.sec / 10) + '0';
    name[14] = (time.sec % 10) + '0';
    name[15] = '.';
    name[16] = 'u';
    name[17] = 'l';
    name[18] = 'g';
}


