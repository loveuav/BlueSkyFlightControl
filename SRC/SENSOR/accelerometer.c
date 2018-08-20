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
#include "flightStatus.h"
#include "sensor.h"
#include "parameter.h"
#include "LevenbergMarquardt.h"
#include "message.h"
#include "mavlinkSend.h"
#include "mavlinkParam.h"

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

    if(abs(acc.cali.offset.x) > 1 || abs(acc.cali.offset.y) > 1 || abs(acc.cali.offset.z) > 1 ||
       abs(acc.cali.scale.x) > 2 || abs(acc.cali.scale.y) > 2 || abs(acc.cali.scale.z) > 2 ||
       abs(acc.cali.scale.x) < 0.3f || abs(acc.cali.scale.y) < 0.3f || abs(acc.cali.scale.z) < 0.3f)
    {
        acc.cali.offset.x = 0;
        acc.cali.offset.y = 0;
        acc.cali.offset.z = 0;
        acc.cali.scale.x = 1;
        acc.cali.scale.y = 1;
        acc.cali.scale.z = 1;
    } 
    
    if(isnan(acc.levelCali.scale.x) || isnan(acc.levelCali.scale.y) || isnan(acc.levelCali.scale.z) ||
       abs(acc.levelCali.scale.x) > 0.2f || abs(acc.levelCali.scale.y) > 0.2f || abs(acc.levelCali.scale.z) > 0.2f)
    {
        acc.levelCali.scale.x = 0;
        acc.levelCali.scale.y = 0;
        acc.levelCali.scale.z = 0;
    } 
    
	//加速度低通滤波系数计算
	LowPassFilter2ndFactorCal(0.001, ACC_LPF_CUT, &acc.lpf_2nd);	
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
	acc.data = VectorRotateToBodyFrame(acc.data, acc.levelCali.scale);
	
	//低通滤波
	acc.dataLpf = LowPassFilter2nd(&acc.lpf_2nd, acc.data);
    
	//计算加速度模值
	acc.mag = Pythagorous3(acc.dataLpf.x, acc.dataLpf.y, acc.dataLpf.z);
    
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
    static uint8_t orientationCaliFlag[6];
    static uint8_t currentOrientation;
	static Vector3f_t new_offset;
	static Vector3f_t new_scale;
    static Vector3f_t samples[6];
    static uint8_t caliFlag = 0;
    static uint32_t caliCnt = 0;
    
	if(!acc.cali.should_cali)
        return;
    
    /*********************************检测IMU放置方向************************************/
    if(GetImuOrientation() == ORIENTATION_UP && !orientationCaliFlag[ORIENTATION_UP])
    {
        //判断IMU是否处于静止状态
        if(GetPlaceStatus() == STATIC)
            caliCnt++;
        else
            caliCnt = 0;
        
        if(caliCnt > 1000)
        {
            caliFlag = 1;
            orientationCaliFlag[ORIENTATION_UP] = 1;
            samples_count = 0;
            acc.cali.step++;
            currentOrientation = ORIENTATION_UP;
            //mavlink发送检测提示
            MavlinkSendNoticeEnable(CAL_DOWN_DETECTED);
        }
    }

    if(GetImuOrientation() == ORIENTATION_DOWN && !orientationCaliFlag[ORIENTATION_DOWN])
    {
        //判断IMU是否处于静止状态
        if(GetPlaceStatus() == STATIC)
            caliCnt++;
        else
            caliCnt = 0;
        
        if(caliCnt > 1000)
        {
            caliFlag = 1;
            orientationCaliFlag[ORIENTATION_DOWN] = 1;
            samples_count = 0;
            acc.cali.step++;
            currentOrientation = ORIENTATION_DOWN;
            //mavlink发送检测提示
            MavlinkSendNoticeEnable(CAL_UP_DETECTED);
        }
    }

    if(GetImuOrientation() == ORIENTATION_FRONT && !orientationCaliFlag[ORIENTATION_FRONT])
    {
        //判断IMU是否处于静止状态
        if(GetPlaceStatus() == STATIC)
            caliCnt++;
        else
            caliCnt = 0;
        
        if(caliCnt > 1000)
        {
            caliFlag = 1;
            orientationCaliFlag[ORIENTATION_FRONT] = 1;           
            samples_count = 0;
            acc.cali.step++;
            currentOrientation = ORIENTATION_FRONT;
            //mavlink发送检测提示
            MavlinkSendNoticeEnable(CAL_FRONT_DETECTED);
        }
    }

    if(GetImuOrientation() == ORIENTATION_BACK && !orientationCaliFlag[ORIENTATION_BACK])
    {
        //判断IMU是否处于静止状态
        if(GetPlaceStatus() == STATIC)
            caliCnt++;
        else
            caliCnt = 0;
        
        if(caliCnt > 1000)
        {
            caliFlag = 1;
            orientationCaliFlag[ORIENTATION_BACK] = 1;              
            samples_count = 0;
            acc.cali.step++;
            currentOrientation = ORIENTATION_BACK;
            //mavlink发送检测提示
            MavlinkSendNoticeEnable(CAL_BACK_DETECTED);
        }
    }

    if(GetImuOrientation() == ORIENTATION_LEFT && !orientationCaliFlag[ORIENTATION_LEFT])
    {
        //判断IMU是否处于静止状态
        if(GetPlaceStatus() == STATIC)
            caliCnt++;
        else
            caliCnt = 0;
        
        if(caliCnt > 1000)
        {
            caliFlag = 1;
            orientationCaliFlag[ORIENTATION_LEFT] = 1;              
            samples_count = 0;
            acc.cali.step++;
            currentOrientation = ORIENTATION_LEFT;
            //mavlink发送检测提示
            MavlinkSendNoticeEnable(CAL_LEFT_DETECTED);                    
        }
    }

    if(GetImuOrientation() == ORIENTATION_RIGHT && !orientationCaliFlag[ORIENTATION_RIGHT])
    {
        //判断IMU是否处于静止状态
        if(GetPlaceStatus() == STATIC)
            caliCnt++;
        else
            caliCnt = 0;
        
        if(caliCnt > 1000)
        {
            caliFlag = 1;
            orientationCaliFlag[ORIENTATION_RIGHT] = 1;              
            samples_count = 0;
            acc.cali.step++;
            currentOrientation = ORIENTATION_RIGHT;
            //mavlink发送检测提示
            MavlinkSendNoticeEnable(CAL_RIGHT_DETECTED);                      
        }
    }
    /****************************************************************************************/
    
    //分别采集加速度计六个方向的数据，顺序随意，每个方向取500个样本求平均值
 	if(caliFlag)
	{
		if(samples_count < 500)
		{
			samples[acc.cali.step - 1].x += accRaw.x;
			samples[acc.cali.step - 1].y += accRaw.y;
			samples[acc.cali.step - 1].z += accRaw.z;
			samples_count++;
		}
		else if(samples_count == 500)
		{   
			samples[acc.cali.step - 1].x /= 500;
			samples[acc.cali.step - 1].y /= 500;
			samples[acc.cali.step - 1].z /= 500;
			samples_count++;
            
            caliFlag = 0;
            caliCnt  = 0;
            
            //bsklink发送当前校准步骤
            MessageSensorCaliFeedbackEnable(ACC, acc.cali.step, acc.cali.success);  

            //mavlink发送当前校准步骤
            switch(currentOrientation)
            {
                case ORIENTATION_UP:
                    MavlinkSendNoticeEnable(CAL_DOWN_DONE); 
                    MavlinkSendNoticeProgress(acc.cali.step * 1.6f);
                    break;
                case ORIENTATION_DOWN:
                    MavlinkSendNoticeEnable(CAL_UP_DONE); 
                    MavlinkSendNoticeProgress(acc.cali.step * 1.6f);
                    break;
                case ORIENTATION_FRONT:
                    MavlinkSendNoticeEnable(CAL_FRONT_DONE); 
                    MavlinkSendNoticeProgress(acc.cali.step * 1.6f);
                    break;
                case ORIENTATION_BACK:
                    MavlinkSendNoticeEnable(CAL_BACK_DONE); 
                    MavlinkSendNoticeProgress(acc.cali.step * 1.6f);
                    break;
                case ORIENTATION_LEFT:
                    MavlinkSendNoticeEnable(CAL_LEFT_DONE); 
                    MavlinkSendNoticeProgress(acc.cali.step * 1.6f);
                    break;
                case ORIENTATION_RIGHT:
                    MavlinkSendNoticeEnable(CAL_RIGHT_DONE); 
                    MavlinkSendNoticeProgress(acc.cali.step * 1.6f);
                    break;
                default:
                    break;
            }
                         
		}		
	}

	if(acc.cali.step == 6 && samples_count == 501)
	{		
		//LM法求解传感器误差方程最优解
		LevenbergMarquardt(samples, &new_offset, &new_scale, 1);

		//判断校准参数是否正常
		if(fabsf(new_scale.x-1.0f) > 0.1f || fabsf(new_scale.y-1.0f) > 0.1f || fabsf(new_scale.z-1.0f) > 0.1f) 
		{
			acc.cali.success = false;
		}
		else if(fabsf(new_offset.x) > (1 * 0.35f) || fabsf(new_offset.y) > (1 * 0.35f) || fabsf(new_offset.z) > (1 * 0.6f)) 
		{
			acc.cali.success = false;
		}
		else
		{
			acc.cali.success = true;
		}
		
		for(u8 i=0; i<6; i++)
		{
			samples[i].x = 0;
			samples[i].y = 0;
			samples[i].z = 0;            
		}
       
        if(acc.cali.success)
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
            //更新mavlink参数
            MavParamSetValue(CAL_ACC0_XOFF, acc.cali.offset.x);
            MavParamSetValue(CAL_ACC0_YOFF, acc.cali.offset.y);
            MavParamSetValue(CAL_ACC0_ZOFF, acc.cali.offset.z);
            MavParamSetValue(CAL_ACC0_XSCALE, acc.cali.scale.x);
            MavParamSetValue(CAL_ACC0_YSCALE, acc.cali.scale.y);
            MavParamSetValue(CAL_ACC0_XSCALE, acc.cali.scale.z);
            
            //mavlink发送校准结果
            MavlinkSendNoticeEnable(CAL_DONE);
		}
        else
        {
            //mavlink发送校准结果
            MavlinkSendNoticeEnable(CAL_FAILED);
        }
		
		//发送校准结果
		MessageSensorCaliFeedbackEnable(ACC, acc.cali.step, acc.cali.success);
        acc.cali.should_cali = 0;
        acc.cali.step = 0;
        for(uint8_t i=0; i<6; i++)
            orientationCaliFlag[i] = 0;
	}       
}

/**********************************************************************************************************
*函 数 名: AccScaleCalibrate
*功能说明: 加速度各轴正反比例误差校正
*形    参: 无 
*返 回 值: 无
**********************************************************************************************************/
void AccScaleCalibrate(Vector3f_t* acc)
{
    if(GYRO_TYPE == MPU6500)
    {
        if(acc->z > 0)
            acc->z *= 1.003f;
        else
            acc->z *= 0.997f;
    }
    else if(GYRO_TYPE == ICM20689)
    {
        if(acc->z > 0)
            acc->z *= 1.002f;
        else
            acc->z *= 0.998f;            
    }
    else
    {
      
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
	const int16_t CALIBRATING_ACC_LEVEL_CYCLES = 3000;	
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

    //mavlink发送校准进度
    MavlinkSendNoticeProgress(((float)count / CALIBRATING_ACC_LEVEL_CYCLES) * 10);
	
	acc.levelCali.step = 1;
	
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
		acc.levelCali.step = 2;
		
        //加速度向量转化为姿态角
        AccVectorToRollPitchAngle(&caliTemp, accAverage);
		
		if(abs(Degrees(caliTemp.x)) < 10 && abs(Degrees(caliTemp.y)) < 10)
		{
			acc.levelCali.success = 1;
			
			acc.levelCali.scale.x = -caliTemp.x;
			acc.levelCali.scale.y = -caliTemp.y;
			acc.levelCali.scale.z = 0;
			
            //保存IMU安装误差校准参数
			ParamUpdateData(PARAM_IMU_LEVEL_X, &acc.levelCali.scale.x);
			ParamUpdateData(PARAM_IMU_LEVEL_Y, &acc.levelCali.scale.y);
			ParamUpdateData(PARAM_IMU_LEVEL_Z, &acc.levelCali.scale.z);
            //更新mavlink参数
            MavParamSetValue(SENS_BOARD_X_OFF, Degrees(acc.levelCali.scale.x));
            MavParamSetValue(SENS_BOARD_Y_OFF, Degrees(acc.levelCali.scale.y));
            MavParamSetValue(SENS_BOARD_Z_OFF, Degrees(acc.levelCali.scale.z));
            
            //mavlink发送校准结果
            MavlinkSendNoticeEnable(CAL_DONE);
		}
		else
		{
			acc.levelCali.success = 0;
            
            //bsklink发送校准结果
            MavlinkSendNoticeEnable(CAL_FAILED);
		}
		
		//发送校准结果
		MessageSensorCaliFeedbackEnable(ANGLE, acc.levelCali.step, acc.levelCali.success);
		
		acc.levelCali.step = 0;
	}    
}

/**********************************************************************************************************
*函 数 名: AccCalibrateEnable
*功能说明: 加速度校准使能
*形    参: 无 
*返 回 值: 无
**********************************************************************************************************/
void AccCalibrateEnable(void)
{
	acc.cali.should_cali = 1;
}

/**********************************************************************************************************
*函 数 名: LevelCalibrateEnable
*功能说明: 水平校准使能
*形    参: 无 
*返 回 值: 无
**********************************************************************************************************/
void LevelCalibrateEnable(void)
{
	acc.levelCali.should_cali = 1;
}

/**********************************************************************************************************
*函 数 名: GetAccOffsetCaliData
*功能说明: 获取加速度零偏校准数据
*形    参: 无 
*返 回 值: 校准参数
**********************************************************************************************************/
Vector3f_t GetAccOffsetCaliData(void)
{
    return acc.cali.offset;
}

/**********************************************************************************************************
*函 数 名: GetAccScaleCaliData
*功能说明: 获取加速度比例校准数据
*形    参: 无 
*返 回 值: 校准参数
**********************************************************************************************************/
Vector3f_t GetAccScaleCaliData(void)
{
    return acc.cali.scale;
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

/**********************************************************************************************************
*函 数 名: AccLpfGetData
*功能说明: 获取经过滤波后的加速度数据
*形    参: 无 
*返 回 值: 加速度
**********************************************************************************************************/
Vector3f_t AccLpfGetData(void)
{
    return acc.dataLpf;
}

