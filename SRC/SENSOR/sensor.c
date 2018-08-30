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
#include "accelerometer.h"
#include "gyroscope.h"

PID_t tempPID;
enum ORIENTATION_STATUS orientationStatus;
SENSOR_HEALTH_t sensorHealth;

/**********************************************************************************************************
*函 数 名: ImuTempControlInit
*功能说明: IMU传感器恒温参数初始化
*形    参: 无 
*返 回 值: 无
**********************************************************************************************************/
void ImuTempControlInit(void)
{
    PID_SetParam(&tempPID, 8, 0.3, 1, 300, 30);	
}

/**********************************************************************************************************
*函 数 名: SensorCheckStatus
*功能说明: 检查传感器状态
*形    参: 无 
*返 回 值: 状态
**********************************************************************************************************/
bool SensorCheckStatus(void)
{
    bool status = true;
    
    //检测陀螺仪状态
    if(FaultDetectGetErrorStatus(GYRO_UNDETECTED))
        status = false;    

    //检测磁力计状态
    if(FaultDetectGetErrorStatus(MAG_UNDETECTED))
        status = false;     

    //检测气压计状态
    if(FaultDetectGetErrorStatus(BARO_UNDETECTED))
        status = false;      
     
    return status;
}

/**********************************************************************************************************
*函 数 名: SensorHealthCheck
*功能说明: 检查传感器健康状态
*形    参: 无 
*返 回 值: 无
**********************************************************************************************************/
void SensorHealthCheck(void)
{
    static uint16_t period_cnt = 0;
    static float gyro_offset, acc_offset;
    static float gyro_offset_temp, acc_offset_temp;
    static uint8_t staticFlag = 0;
    
    //5秒一个检测周期
    if(period_cnt < 1000)
    {
        //陀螺仪零偏累加
        gyro_offset_temp += abs(GyroGetData().x) + abs(GyroGetData().y) + abs(GyroGetData().z);
        //加速度零偏累加
        acc_offset_temp  += abs(GetAccMag() - 1);
        
        //检测期间飞控出现抖动则放弃此次数据
        if(GetPlaceStatus() != STATIC)
        {
            staticFlag = 1;
        }
        
        period_cnt++;
    }
    else
    {
        if(!staticFlag)
        { 
            //计算陀螺仪零偏平均值
            gyro_offset = gyro_offset_temp / 1000;
            //计算加速度零偏平均值
            acc_offset  = acc_offset_temp / 1000;
            
            //计算陀螺仪零偏百分比
            sensorHealth.gyro_offset = ConstrainFloat(ApplyDeadbandFloat(gyro_offset, 0.1) / 1, 0.01, 1);
            //计算加速度零偏百分比
            sensorHealth.acc_offset  = ConstrainFloat(ApplyDeadbandFloat(acc_offset, 0.01) / 0.1f, 0.01, 1);

            //判断陀螺仪健康状态
            if(sensorHealth.gyro_offset > 0.35f)
                sensorHealth.gyro = SENSOR_UNHEALTH;
            else
                sensorHealth.gyro = SENSOR_NORMAL;
            //判断加速度计健康状态
            if(sensorHealth.acc_offset > 0.35f)
                sensorHealth.acc = SENSOR_UNHEALTH;
            else
                sensorHealth.acc = SENSOR_NORMAL;
        }
        
        gyro_offset_temp = 0;
        acc_offset_temp  = 0;
        staticFlag = 0;
        period_cnt = 0;
    }
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
    
    /*加热功能关闭*/
    if(configUSE_SENSORHEAT == 0)
    {
        if(GetSysTimeMs() > 8000 && SensorCheckStatus())
        {
            if(GetInitStatus() < HEAT_FINISH)
                SetInitStatus(HEAT_FINISH);
        }
    }
    /*加热功能开启*/
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
        if(tempMeasure < SENSOR_TEMP_KEPT && !overPreHeatFLag)
        {
            //全速加热
            TempControlSet(1000); 
            PID_ResetI(&tempPID);             
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
        
        if(GetInitStatus() < HEAT_FINISH  && SensorCheckStatus())
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
                SetInitStatus(HEATING); 
            }
        }
    }
}

/**********************************************************************************************************
*函 数 名: ImuOrientationDetect
*功能说明: 检测传感器放置方向
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ImuOrientationDetect(void)
{
    const float CONSTANTS_ONE_G = 1;
    const float accel_err_thr = 0.5;
    
    Vector3f_t acc;
    
    //读取加速度数据
    acc = AccGetData();
    
    // [ g, 0, 0 ]
    if (fabsf(acc.x - CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(acc.y) < accel_err_thr &&
	    fabsf(acc.z) < accel_err_thr) 
    {
		orientationStatus = ORIENTATION_FRONT;        
	}
    // [ -g, 0, 0 ]
	if (fabsf(acc.x + CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(acc.y) < accel_err_thr &&
	    fabsf(acc.z) < accel_err_thr) 
    {
		orientationStatus = ORIENTATION_BACK;        
	}
    // [ 0, g, 0 ]
	if (fabsf(acc.x) < accel_err_thr &&
	    fabsf(acc.y - CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(acc.z) < accel_err_thr) 
    {
		orientationStatus = ORIENTATION_LEFT;        
	}
    // [ 0, -g, 0 ]
	if (fabsf(acc.x) < accel_err_thr &&
	    fabsf(acc.y + CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(acc.z) < accel_err_thr) 
    {
		orientationStatus = ORIENTATION_RIGHT;        
	}
    // [ 0, 0, g ]
	if (fabsf(acc.x) < accel_err_thr &&
	    fabsf(acc.y) < accel_err_thr &&
	    fabsf(acc.z - CONSTANTS_ONE_G) < accel_err_thr) 
    {
		orientationStatus = ORIENTATION_UP;        
	}
    // [ 0, 0, -g ]
	if (fabsf(acc.x) < accel_err_thr &&
	    fabsf(acc.y) < accel_err_thr &&
	    fabsf(acc.z + CONSTANTS_ONE_G) < accel_err_thr) 
    {
		orientationStatus = ORIENTATION_DOWN;        
	}    
}

/**********************************************************************************************************
*函 数 名: GetImuOrientation
*功能说明: 获取传感器放置状态
*形    参: 无
*返 回 值: 状态
**********************************************************************************************************/
enum ORIENTATION_STATUS GetImuOrientation(void)
{
    return orientationStatus;
}

/**********************************************************************************************************
*函 数 名: GetGyroHealthStatus
*功能说明: 获取陀螺仪健康状态
*形    参: 无
*返 回 值: 状态
**********************************************************************************************************/
enum SENSOR_HEALTH GetGyroHealthStatus(void)
{
    return sensorHealth.gyro;
}

/**********************************************************************************************************
*函 数 名: GetAccHealthStatus
*功能说明: 获取加速度计健康状态
*形    参: 无
*返 回 值: 状态
**********************************************************************************************************/
enum SENSOR_HEALTH GetAccHealthStatus(void)
{
    return sensorHealth.acc;
}


