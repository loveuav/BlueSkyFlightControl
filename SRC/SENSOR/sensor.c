/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     sensor.c
 * @说明     传感器数据预处理
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "sensor.h"
#include "module.h"
#include "board.h"
#include "pid.h"
#include "flightStatus.h"
#include "faultDetect.h"

PID_t tempPID;

/**********************************************************************************************************
*函 数 名: ImuTempControlInit
*功能说明: IMU传感器恒温参数初始化
*形    参: 无 
*返 回 值: 无
**********************************************************************************************************/
void ImuTempControlInit(void)
{
    PID_SetParam(&tempPID, 3, 0.2, 1, 100, 30);	
}

/**********************************************************************************************************
*函 数 名: ImuTempControl
*功能说明: IMU传感器恒温控制
*形    参: 温度测量值
*返 回 值: 无
**********************************************************************************************************/
void ImuTempControl(float tempMeasure)
{
	static uint64_t lastTime = 0;   
	int32_t tempError = 0;	//误差变量，使用整型并保留原始数据小数点后两位，避免引入噪声    
    static int32_t tempPIDTerm = 0;
    float	deltaT = (GetSysTimeUs() - lastTime) * 1e-6;
	lastTime = GetSysTimeUs();
    static uint16_t cnt = 0;
    static uint8_t overPreHeatFLag = 0;
    
    if(configUSE_SENSORHEAT == 0)
    {
        if(GetSysTimeMs() > 8000)
        {
            if(GetInitStatus() < HEAT_FINISH)
                SetInitStatus(HEAT_FINISH);
        }
    }
    else
    {
        //检测不到陀螺仪时停止加热并退出该函数
        if(FaultDetectGetErrorStatus(GYRO_UNDETECTED))
        {
            TempControlSet(0);
            return;
        }

        //计算温度误差
        tempError = SENSOR_TEMP_KEPT * 100 - tempMeasure * 100;	  

        //超前预热
        if(tempMeasure < SENSOR_TEMP_KEPT + 2 && !overPreHeatFLag)
        {
            //全速加热
            TempControlSet(1000);     
            return;
        }
        else
        {
            overPreHeatFLag = 1;
        }
        
        //计算PID输出
        tempPIDTerm = PID_GetPID(&tempPID, tempError, deltaT);
        //PID输出限幅
        tempPIDTerm = ConstrainInt32(tempPIDTerm, 0, 1000);
        //转换控制量为PWM输出
        TempControlSet(tempPIDTerm);
        
        if(GetInitStatus() < HEAT_FINISH)
        {
            //温度接近预定温度
            if(abs(tempError) < 80 && overPreHeatFLag)
            {
                cnt++;
                if(cnt > 5000)
                    SetInitStatus(HEAT_FINISH);
            }          
            else
            {
                PID_ResetI(&tempPID); 
                SetInitStatus(HEATING); 
            }
        }
    }
}




