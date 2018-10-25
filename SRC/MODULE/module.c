/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     module.c
 * @说明     提供通用的传感器数据接口,除了对数据实时性要求较高的IMU之外（即读即用），其它传感器的数据更新与使用并不完全同步
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05
**********************************************************************************************************/
#include "module.h"
#include "mpu6000.h"
#include "mpu6500.h"
#include "icm20602.h"
#include "icm20689.h"
#include "ms5611.h"
#include "2smpb.h"
#include "qmc5883.h"
#include "ist8310.h"
#include "mmc3630.h"
#include "ublox.h"
#include "drv_pwm.h"
#include "faultDetect.h"
#include "rotation.h"

/**********************************************************************************************************
*函 数 名: GyroSensorRotate
*功能说明: 陀螺仪传感器方向变换，以适应传感器的实际安装方向
*形    参: 陀螺仪数据指针
*返 回 值: 无
**********************************************************************************************************/
static void GyroSensorRotate(Vector3f_t* gyro)
{
    RotateVector3f(GYRO_ROTATION, gyro);
}

/**********************************************************************************************************
*函 数 名: AccSensorRotate
*功能说明: 加速度数据方向变换，以适应传感器的实际安装方向
*形    参: 加速度数据指针
*返 回 值: 无
**********************************************************************************************************/
static void AccSensorRotate(Vector3f_t *acc)
{
    RotateVector3f(ACC_ROTATION, acc);
}

/**********************************************************************************************************
*函 数 名: MagSensorRotate
*功能说明: 磁力计数据方向变换，以适应传感器的实际安装方向
*形    参: 磁力计数据指针
*返 回 值: 无
**********************************************************************************************************/
static void MagSensorRotate(Vector3f_t *mag)
{
    RotateVector3f(MAG_ROTATION, mag);
}

/**********************************************************************************************************
*函 数 名: GyroSensorInit
*功能说明: 陀螺仪传感器初始化
*形    参: 无
*返 回 值: 陀螺仪存在状态
**********************************************************************************************************/
void GyroSensorInit(void)
{
    uint8_t detectFlag = 0;

    if(GYRO_TYPE == MPU6000)
    {
        if(MPU6000_Detect())
        {
            MPU6000_Init();
            detectFlag = 1;
        }
    }
    else if (GYRO_TYPE == MPU6500)
    {
        if(MPU6500_Detect())
        {
            MPU6500_Init();
            detectFlag = 1;
        }
    }
    else if (GYRO_TYPE == ICM20602)
    {
        if(ICM20602_Detect())
        {
            ICM20602_Init();
            detectFlag = 1;
        }
    }
    else if (GYRO_TYPE == ICM20689)
    {
        if(ICM20689_Detect())
        {
            ICM20689_Init();
            detectFlag = 1;
        }
    }

    //未检测到陀螺仪
    if(!detectFlag)
        FaultDetectSetError(GYRO_UNDETECTED);
}

/**********************************************************************************************************
*函 数 名: MagSensorInit
*功能说明: 磁力传感器初始化
*形    参: 无
*返 回 值: 磁力计存在状态
**********************************************************************************************************/
void MagSensorInit(void)
{
    uint8_t detectFlag = 0;

    if(MAG_TYPE == QMC5883)
    {
        if(QMC5883_Detect())
        {
            QMC5883_Init();
            detectFlag = 1;
        }
    }
    else if(MAG_TYPE == IST8310)
    {
        if(IST8310_Detect())
        {
            IST8310_Init();
            detectFlag = 1;
        }
    }
    else if(MAG_TYPE == MMC3630)
    {
        if(MMC3630_Detect())
        {
            MMC3630_Init();
            detectFlag = 1;
        }
    }

    //未检测到磁力计
    if(!detectFlag)
        FaultDetectSetError(MAG_UNDETECTED);
}

/**********************************************************************************************************
*函 数 名: BaroSensorInit
*功能说明: 气压传感器初始化
*形    参: 无
*返 回 值: 气压计存在状态
**********************************************************************************************************/
void BaroSensorInit(void)
{
    uint8_t detectFlag = 0;

    if(BARO_TYPE == MS5611)
    {
        if(MS5611_Detect())
        {
            MS5611_Init();
            detectFlag = 1;
        }
    }
    else if(BARO_TYPE == _2SMPB)
    {
        if(_2SMPB_Detect())
        {
            _2SMPB_Init();
            detectFlag = 1;
        }
    }

    //未检测到气压计
    if(!detectFlag)
        FaultDetectSetError(BARO_UNDETECTED);
}

/**********************************************************************************************************
*函 数 名: GPSModuleInit
*功能说明: GPS模块初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void GPSModuleInit(void)
{
    Ublox_Init();
}


/**********************************************************************************************************
*函 数 名: GyroSensorRead
*功能说明: 角速度数据读取
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void GyroSensorRead(Vector3f_t* gyro)
{
    if(GYRO_TYPE == MPU6000)
        MPU6000_ReadGyro(gyro);
    else if(GYRO_TYPE == MPU6500)
        MPU6500_ReadGyro(gyro);
    else if(GYRO_TYPE == ICM20602)
        ICM20602_ReadGyro(gyro);
    else if(GYRO_TYPE == ICM20689)
        ICM20689_ReadGyro(gyro);

    //传感器方向转换
    GyroSensorRotate(gyro);
}

/**********************************************************************************************************
*函 数 名: AccSensorRead
*功能说明: 加速度数据读取
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void AccSensorRead(Vector3f_t* acc)
{
    if(GYRO_TYPE == MPU6000)
        MPU6000_ReadAcc(acc);
    else if(GYRO_TYPE == MPU6500)
        MPU6500_ReadAcc(acc);
    else if(GYRO_TYPE == ICM20602)
        ICM20602_ReadAcc(acc);
    else if(GYRO_TYPE == ICM20689)
        ICM20689_ReadAcc(acc);

    //传感器方向转换
    AccSensorRotate(acc);
}

/**********************************************************************************************************
*函 数 名: TempSensorRead
*功能说明: 陀螺仪温度数据读取
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void TempSensorRead(float* temp)
{
    if(GYRO_TYPE == MPU6000)
        MPU6000_ReadTemp(temp);
    else if(GYRO_TYPE == MPU6500)
        MPU6500_ReadTemp(temp);
    else if(GYRO_TYPE == ICM20602)
        ICM20602_ReadTemp(temp);
    else if(GYRO_TYPE == ICM20689)
        ICM20689_ReadTemp(temp);
}

/**********************************************************************************************************
*函 数 名: MagSensorUpdate
*功能说明: 罗盘数据更新
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MagSensorUpdate(void)
{
    if(MAG_TYPE == QMC5883)
        QMC5883_Update();
    else if(MAG_TYPE == IST8310)
        IST8310_Update();
    else if(MAG_TYPE == MMC3630)
        MMC3630_Update();
}

/**********************************************************************************************************
*函 数 名: MagSensorRead
*功能说明: 罗盘数据读取
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void MagSensorRead(Vector3f_t* mag)
{
    if(MAG_TYPE == QMC5883)
        QMC5883_Read(mag);
    else if(MAG_TYPE == IST8310)
        IST8310_Read(mag);
    else if(MAG_TYPE == MMC3630)
        MMC3630_Read(mag);

    //传感器方向转换
    MagSensorRotate(mag);
}

/**********************************************************************************************************
*函 数 名: BaroSensorUpdate
*功能说明: 气压高度数据更新
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void BaroSensorUpdate(void)
{
    if(BARO_TYPE == MS5611)
        MS5611_Update();
    else if(BARO_TYPE == _2SMPB)
        _2SMPB_Update();
}

/**********************************************************************************************************
*函 数 名: BaroSensorRead
*功能说明: 气压高度数据读取
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void BaroSensorRead(int32_t* baroAlt)
{
    if(BARO_TYPE == MS5611)
        MS5611_Read(baroAlt);
    else if(BARO_TYPE == _2SMPB)
        _2SMPB_Read(baroAlt);
}

/**********************************************************************************************************
*函 数 名: BaroTemperatureRead
*功能说明: 气压计温度读取
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void BaroTemperatureRead(float* temp)
{
    if(BARO_TYPE == MS5611)
        MS5611_ReadTemp(temp);
    else if(BARO_TYPE == _2SMPB)
        _2SMPB_ReadTemp(temp);
}

/**********************************************************************************************************
*函 数 名: TempControlSet
*功能说明: 陀螺仪传感器加热数值设置（0-1000）
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void TempControlSet(int16_t value)
{
#if(configUSE_SENSORHEAT == 1)
    TempControlPWMSet(value);
#else
    TempControlPWMSet(0);
#endif
}


