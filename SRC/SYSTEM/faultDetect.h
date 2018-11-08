#ifndef __FAULTDETECT_H__
#define __FAULTDETECT_H__

#include "mathTool.h"

//错误类型
enum ERROR_TYPE
{
    ERROR_RESERVE1,
    GYRO_UNDETECTED,	    	//未检测到陀螺仪传感器
    ACC_UNDETECTED,	        	//未检测到加速度传感器
    MAG_UNDETECTED,	        	//未检测到地磁传感器
    BARO_UNDETECTED,	    	//未检测到气压传感器
    GPS_UNDETECTED,	        	//未检测到GPS
    OPTICALFLOW_UNDETECTED,	    //未检测到光流
    ULTRASONIC_UNDETECTED,		//未检测到超声波传感器
    TOF_UNDETECTED,	        	//未检测到TOF传感器
    ERROR_RESERVE2,
    ERROR_RESERVE3,
    ERROR_RESERVE4,
    ERROR_RESERVE5,
    GYRO_UNCALIBRATED,	    	//陀螺仪传感器未校准
    ACC_UNCALIBRATED,	        //加速度传感器未校准
    MAG_UNCALIBRATED,	        //地磁传感器未校准
    TOF_UNCALIBRATED,	        //TOF传感器未校准
    ERROR_RESERVE6,
    ERROR_RESERVE7,
    ERROR_RESERVE8,
    ERROR_RESERVE9,
    SENSOR_HEAT_ERROR,          //IMU传感器恒温不工作
    BMS_UNDETECTED,             //未检测到电池管理系统
    CRITICAL_LOW_BATTERY,       //严重低电量/电压
    ERROR_NUM
};

//警告类型
enum WARNNING_TYPE
{
    WARNNING_RESERVE1,
    SYSTEM_INITIALIZING,        //系统正在初始化
    WARNNING_RESERVE2,
    GYRO_NEED_CALIBRATED,	    //建议校准陀螺仪传感器
    ACC_NEED_CALIBRATED,	    //建议校准加速度传感器
    MAG_NEED_CALIBRATED,	    //建议校准地磁传感器
    WARNNING_RESERVE3,
    WARNNING_RESERVE4,
    WARNNING_RESERVE5,
    WARNNING_RESERVE6,
    MAG_DISTURBING,             //地磁传感器受干扰
    WARNNING_RESERVE7,
    LOW_BATTERY,                //低电量/电压
    LOW_BATTERY_FOR_RETURN,     //电量/电压低于返航所需
    WARNNING_NUM
};


void FaultDetectInit(void);
void FaultDetectSetError(uint16_t error_type);
void FaultDetectResetError(uint16_t error_type);
void FaultDetectSetWarnning(uint16_t warnning_type);
void FaultDetectResetWarnning(uint16_t warnning_type);
uint8_t FaultDetectGetErrorStatus(uint16_t error_type);
uint8_t FaultDetectGetWarnningStatus(uint16_t warnning_type);
uint8_t* FaultDetectGetError(void);
uint8_t* FaultDetectGetWarnning(void);

#endif



















