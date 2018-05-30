/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     accelerometer.c
 * @说明     加速度校准及数据预处理
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "accelerometer.h"
#include "parameter.h"
#include "gaussNewton.h"

ACCELEROMETER_t acc;

/**********************************************************************************************************
*函 数 名: AccPreTreatInit
*功能说明: 加速度预处理初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void AccPreTreatInit(void)
{
	ParamGetData(PARAM_ACC_OFFSET_X, &acc.cali.offset.x, 4);
	ParamGetData(PARAM_ACC_OFFSET_Y, &acc.cali.offset.y, 4);
	ParamGetData(PARAM_ACC_OFFSET_Z, &acc.cali.offset.z, 4);
	ParamGetData(PARAM_ACC_SCALE_X, &acc.cali.scale.x, 4);
	ParamGetData(PARAM_ACC_SCALE_Y, &acc.cali.scale.y, 4);
	ParamGetData(PARAM_ACC_SCALE_Z, &acc.cali.scale.z, 4);

	ParamGetData(PARAM_IMU_LEVEL_X, &acc.levelCali.scale.x, 4);
	ParamGetData(PARAM_IMU_LEVEL_Y, &acc.levelCali.scale.y, 4);
	ParamGetData(PARAM_IMU_LEVEL_Z, &acc.levelCali.scale.z, 4);
    
    if(isnan(acc.cali.offset.x) || isnan(acc.cali.offset.y) || isnan(acc.cali.offset.z) || \
       isnan(acc.cali.scale.x) || isnan(acc.cali.scale.y) || isnan(acc.cali.scale.z) ||    \
       acc.cali.scale.x == 0 || acc.cali.scale.y == 0 || acc.cali.scale.z == 0)
    {
        acc.cali.offset.x = 0;
        acc.cali.offset.y = 0;
        acc.cali.offset.z = 0;
        acc.cali.scale.x = 1;
        acc.cali.scale.y = 1;
        acc.cali.scale.z = 1;
    } 
    
    if(isnan(acc.levelCali.scale.x) || isnan(acc.levelCali.scale.y) || isnan(acc.levelCali.scale.z))
    {
        acc.levelCali.scale.x = 0;
        acc.levelCali.scale.y = 0;
        acc.levelCali.scale.z = 0;
    } 
}

/**********************************************************************************************************
*函 数 名: AccDataPreTreat
*功能说明: 加速度数据预处理
*形    参: 加速度原始数据 加速度预处理数据指针
*返 回 值: 无
**********************************************************************************************************/
void AccDataPreTreat(Vector3f_t accRaw, Vector3f_t* accData)
{
	static float lastAccMag, accMagderi;
	const float deltaT = 0.001f;

	acc.data = accRaw;
	
	//加速度数据校准
	acc.data.x = (acc.data.x - acc.cali.offset.x) * acc.cali.scale.x;
	acc.data.y = (acc.data.y - acc.cali.offset.y) * acc.cali.scale.y;	
	acc.data.z = (acc.data.z - acc.cali.offset.z) * acc.cali.scale.z;	
	
	//水平误差校准
	acc.data = VectorRotate(acc.data, acc.levelCali.scale);
	
	//计算加速度模值
	acc.mag = acc.mag * 0.95f + Pythagorous3(acc.data.x, acc.data.y, acc.data.z) / GRAVITY_ACCEL * 0.05f;
	
	//震动系数计算
	accMagderi = (acc.mag - lastAccMag) / deltaT;
	lastAccMag = acc.mag;	
	acc.vibraCoef = acc.vibraCoef * 0.9995f + abs(accMagderi) * 0.0005f;
    
    *accData = acc.data;
}

/**********************************************************************************************************
*函 数 名: AccCalibration
*功能说明: 加速度校准
*形    参: 加速度原始数据 
*返 回 值: 无
**********************************************************************************************************/
void AccCalibration(Vector3f_t accRaw)
{
    static uint16_t samples_count = 0;
	static Vector3f_t new_offset;
	static Vector3f_t new_scale;
    static Vector3f_t samples[6];
    bool success = true;
    
	if(acc.cali.should_cali)
	{
		acc.cali.step++;
		acc.cali.should_cali = 0;
		samples_count = 0;		
	}
    
    //分别采集加速度计六个方向的数据，顺序随意，每个方向取100个样本求平均值
 	if(acc.cali.step)
	{
		if(samples_count < 100)
		{
			samples[acc.cali.step - 1].x += accRaw.x;
			samples[acc.cali.step - 1].y += accRaw.y;
			samples[acc.cali.step - 1].z += accRaw.z;
			samples_count++;
		}
		else if(samples_count == 100)
		{   
			samples[acc.cali.step - 1].x /= 100;
			samples[acc.cali.step - 1].y /= 100;
			samples[acc.cali.step - 1].z /= 100;
			samples_count++;		
		}		
	}	

	if(acc.cali.step == 6)
	{
		if(samples_count == 101)
		{
            //高斯牛顿法求解误差方程
            GaussNewtonCalibrate(samples, &new_offset, &new_scale, GRAVITY_ACCEL, 20);
			acc.cali.step = 0;

            //判断校准参数是否正常
            if(fabsf(new_scale.x-1.0f) > 0.1f || fabsf(new_scale.y-1.0f) > 0.1f || fabsf(new_scale.z-1.0f) > 0.1f) 
            {
                success = false;
            }
            if(fabsf(new_offset.x) > (GRAVITY_ACCEL * 0.35f) || fabsf(new_offset.y) > (GRAVITY_ACCEL * 0.35f) || fabsf(new_offset.z) > (GRAVITY_ACCEL * 0.6f)) 
            {
                success = false;
            }
        }
        
        if(success)
        {
			acc.cali.offset = new_offset;
			acc.cali.scale = new_scale;

			//保存加速度校准参数
			ParamUpdateData(PARAM_ACC_OFFSET_X, &acc.cali.offset.x);
			ParamUpdateData(PARAM_ACC_OFFSET_Y, &acc.cali.offset.y);
			ParamUpdateData(PARAM_ACC_OFFSET_Z, &acc.cali.offset.z);
			ParamUpdateData(PARAM_ACC_SCALE_X, &acc.cali.scale.x);
			ParamUpdateData(PARAM_ACC_SCALE_Y, &acc.cali.scale.y);
			ParamUpdateData(PARAM_ACC_SCALE_Z, &acc.cali.scale.z);
		}
        else
        {
            
        }
	}       
}

/**********************************************************************************************************
*函 数 名: ImuLevelCalibration
*功能说明: IMU传感器的水平校准（安装误差），主要读取静止时的加速度数据并求平均值，得到校准角度值
*形    参: 无 
*返 回 值: 无
**********************************************************************************************************/
void ImuLevelCalibration(void)
{
	const int16_t CALIBRATING_ACC_LEVEL_CYCLES = 500;	
	static float acc_sum[3] = {0, 0, 0};
	Vector3f_t accAverage;
	Vector3f_t caliTemp;
	static int16_t count = 0;
	
	if(!acc.levelCali.should_cali)
		return;
		
	if(count == 0)
	{
		acc.levelCali.scale.x = 0;
		acc.levelCali.scale.y = 0;
		acc.levelCali.scale.z = 0;
	}
	else
	{
		acc_sum[0] += acc.data.x;
		acc_sum[1] += acc.data.y;
		acc_sum[2] += acc.data.z;
	}
	count++;
	
	if(count == CALIBRATING_ACC_LEVEL_CYCLES)
	{
		accAverage.x = acc_sum[0] / (CALIBRATING_ACC_LEVEL_CYCLES-1);
		accAverage.y = acc_sum[1] / (CALIBRATING_ACC_LEVEL_CYCLES-1);
		accAverage.z = acc_sum[2] / (CALIBRATING_ACC_LEVEL_CYCLES-1);
		acc_sum[0] = 0;
		acc_sum[1] = 0;
		acc_sum[2] = 0;
		count = 0;
		acc.levelCali.should_cali = 0;
		
		caliTemp.x = atan2f(accAverage.y, Pythagorous2(accAverage.x, accAverage.z));
		caliTemp.y = atan2f(-accAverage.x, Pythagorous2(accAverage.y, accAverage.z));
		
		if(abs(Degrees(caliTemp.x)) < 10 && abs(Degrees(caliTemp.y)) < 10)
		{
			acc.levelCali.scale.x = -caliTemp.x;
			acc.levelCali.scale.y = -caliTemp.y;
			acc.levelCali.scale.z = 0;
			
            //保存IMU安装误差校准参数
			ParamUpdateData(PARAM_IMU_LEVEL_X, &acc.levelCali.scale.x);
			ParamUpdateData(PARAM_IMU_LEVEL_Y, &acc.levelCali.scale.y);
			ParamUpdateData(PARAM_IMU_LEVEL_Z, &acc.levelCali.scale.z);
		}
	}    
}

/**********************************************************************************************************
*函 数 名: GetLevelCalibraData
*功能说明: 获取IMU安装误差校准参数
*形    参: 无 
*返 回 值: 校准参数
**********************************************************************************************************/
Vector3f_t GetLevelCalibraData(void)
{
    return acc.levelCali.scale;
}

/**********************************************************************************************************
*函 数 名: GetAccMag
*功能说明: 获取加速度数据模值
*形    参: 无 
*返 回 值: 模值
**********************************************************************************************************/
float GetAccMag(void)
{
    return acc.mag;
}

/**********************************************************************************************************
*函 数 名: AccGetData
*功能说明: 获取经过处理后的加速度数据
*形    参: 无 
*返 回 值: 加速度
**********************************************************************************************************/
Vector3f_t AccGetData(void)
{
    return acc.data;
}

