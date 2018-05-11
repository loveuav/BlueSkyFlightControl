/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     gyroscope.c
 * @说明     陀螺仪校准及数据预处理
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "gyroscope.h"
#include "parameter.h"

GYROSCOPE_t gyro;

/**********************************************************************************************************
*函 数 名: GyroRotate
*功能说明: 陀螺仪数据坐标变换
*形    参: 陀螺仪数据指针
*返 回 值: 无
**********************************************************************************************************/
static void GyroRotate(Vector3f_t* gyro)
{
	Vector3f_t temp;
	
	temp = *gyro;
	(*gyro).x = temp.y;
	(*gyro).y = temp.x;
	(*gyro).z = -temp.z;
}

/**********************************************************************************************************
*函 数 名: GyroCaliDataInit
*功能说明: 陀螺仪校准参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void GyroCaliDataInit(void)
{
	ParamGetData(PARAM_GYRO_OFFSET_X, &gyro.cali.offset.x, 4);
	ParamGetData(PARAM_GYRO_OFFSET_Y, &gyro.cali.offset.y, 4);
	ParamGetData(PARAM_GYRO_OFFSET_Z, &gyro.cali.offset.z, 4);
    
    if(isnan(gyro.cali.offset.x) || isnan(gyro.cali.offset.y) || isnan(gyro.cali.offset.z))
    {
        gyro.cali.offset.x = 0;
        gyro.cali.offset.y = 0;
        gyro.cali.offset.z = 0;
    }
}

/**********************************************************************************************************
*函 数 名: GyroDataPreTreat
*功能说明: 陀螺仪数据预处理
*形    参: 陀螺仪原始数据 陀螺仪预处理数据指针
*返 回 值: 无
**********************************************************************************************************/
void GyroDataPreTreat(Vector3f_t gyroRaw, Vector3f_t* gyroData)
{	
	gyro.data = gyroRaw;
    
    //根据传感器的实际安装方向对陀螺仪数据进行坐标变换
	GyroRotate(&gyro.data);
	
	//零偏误差校准
	gyro.data.x -= gyro.cali.offset.x;
	gyro.data.y -= gyro.cali.offset.y;	
	gyro.data.z -= gyro.cali.offset.z;	
	
	//安装误差校准
    //gyro.data = VectorRotate(gyro.data, Vector3f_t deltaAngle);
    
    gyroData->x = gyro.data.x;
    gyroData->y = gyro.data.y;
    gyroData->z = gyro.data.z;
}

/**********************************************************************************************************
*函 数 名: GyroCalibration
*功能说明: 陀螺仪校准
*形    参: 陀螺仪原始数据 
*返 回 值: 无
**********************************************************************************************************/
void GyroCalibration(Vector3f_t gyroRaw)
{
	const int16_t CALIBRATING_GYRO_CYCLES = 1000;	
	static float gyro_sum[3] = {0, 0, 0};
	Vector3f_t gyro_cali_temp, gyro_raw_temp;
	static int16_t count = 0;
	
	if(!gyro.cali.should_cali)
		return;
	
	gyro_raw_temp = gyroRaw;
	GyroRotate(&gyro_raw_temp);
	gyro_sum[0] += gyro_raw_temp.x;
	gyro_sum[1] += gyro_raw_temp.y;
	gyro_sum[2] += gyro_raw_temp.z;
	count++;
	
	if(count == CALIBRATING_GYRO_CYCLES)
	{
		count = 0;
		gyro.cali.should_cali = 0;
		
		gyro_cali_temp.x = gyro_sum[0] / CALIBRATING_GYRO_CYCLES;
		gyro_cali_temp.y = gyro_sum[1] / CALIBRATING_GYRO_CYCLES;
		gyro_cali_temp.z = gyro_sum[2] / CALIBRATING_GYRO_CYCLES;
		gyro_sum[0] = 0;
		gyro_sum[1] = 0;
		gyro_sum[2] = 0;
		
		//检测校准数据是否有效		
		if((abs(gyro_raw_temp.x - gyro_cali_temp.x) + abs(gyro_raw_temp.x - gyro_cali_temp.x)
			+ abs(gyro_raw_temp.x - gyro_cali_temp.x)) < 10)
		{
			gyro.cali.offset.x = gyro_cali_temp.x;
			gyro.cali.offset.y = gyro_cali_temp.y;
			gyro.cali.offset.z = gyro_cali_temp.z;		
				
			//保存陀螺仪校准参数
			ParamUpdateData(PARAM_GYRO_OFFSET_X, &gyro.cali.offset.x);
			ParamUpdateData(PARAM_GYRO_OFFSET_Y, &gyro.cali.offset.y);
			ParamUpdateData(PARAM_GYRO_OFFSET_Z, &gyro.cali.offset.z);
		}		
	}
}

/**********************************************************************************************************
*函 数 名: GyroCalibrateEnable
*功能说明: 陀螺仪校准使能
*形    参: 无 
*返 回 值: 无
**********************************************************************************************************/
void GyroCalibrateEnable(void)
{
	gyro.cali.should_cali = 1;
}





