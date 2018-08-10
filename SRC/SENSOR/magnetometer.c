/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     magnetometer.c
 * @说明     磁力计校准及数据预处理
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "magnetometer.h"
#include "module.h"
#include "gyroscope.h"
#include "parameter.h"
#include "LevenbergMarquardt.h"
#include "faultDetect.h"
#include "message.h"
#include "mavlinkSend.h"
#include "mavlinkParam.h"

enum{
    MaxX,
    MinX,
    MaxY,
    MinY,
    MaxZ,
    MinZ
};

//地球表面赤道上的磁场强度在0.29～0.40高斯之间,两极处的强度略大,地磁北极约0.61高斯,南极约0.68高斯
//不同地方磁场强度有所区别，所以每次校准磁力计时要把当地磁场强度的大概值保存下来

MAGNETOMETER_t mag;

static void MagDetectCheck(Vector3f_t magRaw);

/**********************************************************************************************************
*函 数 名: MagCaliDataInit
*功能说明: 磁力计校准参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MagCaliDataInit(void)
{
	ParamGetData(PARAM_MAG_OFFSET_X, &mag.cali.offset.x, 4);
	ParamGetData(PARAM_MAG_OFFSET_Y, &mag.cali.offset.y, 4);
	ParamGetData(PARAM_MAG_OFFSET_Z, &mag.cali.offset.z, 4);
	ParamGetData(PARAM_MAG_SCALE_X, &mag.cali.scale.x, 4);
	ParamGetData(PARAM_MAG_SCALE_Y, &mag.cali.scale.y, 4);
	ParamGetData(PARAM_MAG_SCALE_Z, &mag.cali.scale.z, 4);   
	ParamGetData(PARAM_MAG_EARTH_MAG , &mag.earthMag, 4);  

    if(isnan(mag.cali.offset.x) || isnan(mag.cali.offset.y) || isnan(mag.cali.offset.z) || \
       isnan(mag.cali.scale.x) || isnan(mag.cali.scale.y) || isnan(mag.cali.scale.z) ||    \
       abs(mag.cali.scale.x) < 0.3f || abs(mag.cali.scale.x) > 2 || \
       abs(mag.cali.scale.y) < 0.3f || abs(mag.cali.scale.y) > 2 || \
       abs(mag.cali.scale.z) < 0.3f || abs(mag.cali.scale.z) > 2 || \
       mag.cali.scale.x == 0 || mag.cali.scale.y == 0 || mag.cali.scale.z == 0)
    {
        mag.cali.offset.x = 0;
        mag.cali.offset.y = 0;
        mag.cali.offset.z = 0;
        mag.cali.scale.x = 1;
        mag.cali.scale.y = 1;
        mag.cali.scale.z = 1;
        mag.earthMag = 0.4;
    }    
}

/**********************************************************************************************************
*函 数 名: MagDataPreTreat
*功能说明: 磁力计数据预处理
*形    参: 磁力计原始数据 磁力计预处理数据指针
*返 回 值: 无
**********************************************************************************************************/
void MagDataPreTreat(void)
{
	Vector3f_t magRaw;
    
	//获取磁力计传感器采样值	
    MagSensorRead(&magRaw);

	//检测磁力计是否工作正常
	MagDetectCheck(magRaw);
	
	//磁力计数据校准
	mag.data.x = (magRaw.x - mag.cali.offset.x) * mag.cali.scale.x;
	mag.data.y = (magRaw.y - mag.cali.offset.y) * mag.cali.scale.y;	
	mag.data.z = (magRaw.z - mag.cali.offset.z) * mag.cali.scale.z;	

	//计算磁场强度模值，用于判断周边是否存在磁场干扰（正常值为1）
	mag.mag = mag.mag * 0.99f + Pythagorous3(mag.data.x, mag.data.y, mag.data.z) / mag.earthMag * 0.01f;		
}


/**********************************************************************************************************
*函 数 名: MagCalibration
*功能说明: 磁力计校准
*形    参: 磁力计原始数据 
*返 回 值: 无
**********************************************************************************************************/
void MagCalibration(void)
{
    static Vector3f_t samples[6];
	static uint32_t cnt_m=0;
	static float cali_rotate_angle = 0;
    static Vector3f_t new_offset;
    static Vector3f_t new_scale;
	Vector3f_t magRaw;
    static float earthMag = 0;
    
    //计算时间间隔，用于积分
	static uint64_t previousT;
	float deltaT = (GetSysTimeUs() - previousT) * 1e-6;	
	previousT = GetSysTimeUs();	
	
	if(mag.cali.should_cali)
	{
        //读取罗盘数据
        MagSensorRead(&magRaw);
        
        //校准分两个阶段：1.水平旋转 2.机头朝上或朝下然后水平旋转
        //两个阶段分别对飞机的z轴和x轴陀螺仪数据进行积分，记录旋转过的角度
        if(mag.cali.step == 1)
        {
            cali_rotate_angle += GyroGetData().z * deltaT;
        }
        else if(mag.cali.step == 2)
        {
            cali_rotate_angle += GyroGetData().x * deltaT;
        }
    
		if(cnt_m == 0)
        {
			mag.cali.step = 1;
			cali_rotate_angle  = 0;		
			cnt_m++;
			
			//发送当前校准步骤
			MessageSensorCaliFeedbackEnable(MAG, mag.cali.step, mag.cali.success);
		}
		else if(cnt_m == 1)
        {
            //初始化磁场强度模值
            earthMag = Pythagorous3(magRaw.x, magRaw.y, magRaw.z);
            //初始化采样点
			samples[MaxX] = samples[MinX] = magRaw;
			samples[MaxY] = samples[MinY] = magRaw;
			samples[MaxZ] = samples[MinZ] = magRaw;
			cnt_m++;			
		}		
		else
        {
            //实时计算磁场强度模值
            earthMag = earthMag * 0.998f + Pythagorous3(magRaw.x, magRaw.y, magRaw.z) * 0.002f;
            
            //找到每个轴的最大最小值，并对采样值进行一阶低通滤波
            if(Pythagorous3(magRaw.x, magRaw.y, magRaw.z) < earthMag * 1.45f)
            {
                if(magRaw.x > samples[MaxX].x)
                { 
                    LowPassFilter1st(&samples[MaxX], magRaw, 0.3);
                }			
                if(magRaw.x < samples[MinX].x)
                {
                    LowPassFilter1st(&samples[MinX], magRaw, 0.3);
                }		
                if(magRaw.y > samples[MaxY].y)
                {
                    LowPassFilter1st(&samples[MaxY], magRaw, 0.3);
                }			
                if(magRaw.y < samples[MinY].y)
                {
                    LowPassFilter1st(&samples[MinY], magRaw, 0.3);
                }
                if(magRaw.z > samples[MaxZ].z)
                {
                    LowPassFilter1st(&samples[MaxZ], magRaw, 0.3);
                }
                if(magRaw.z < samples[MinZ].z)
                {
                    LowPassFilter1st(&samples[MinZ], magRaw, 0.3);
                }
            }
            else
            {
                earthMag = earthMag;
            }
            
            //mavlink发送当前校准进度
            if(mag.cali.step == 1)
                MavlinkSendNoticeProgress(abs(cali_rotate_angle) / 72);
            else if(mag.cali.step == 2)
                MavlinkSendNoticeProgress((abs(cali_rotate_angle) + 360) / 72);
            
			//水平旋转一圈
			if(mag.cali.step == 1 && abs(cali_rotate_angle) > 360)
			{
				mag.cali.step = 2;
				cali_rotate_angle  = 0;
				//bsklink发送当前校准步骤
				MessageSensorCaliFeedbackEnable(MAG, mag.cali.step, mag.cali.success);  
                //msklink发送当前校准步骤
                MavlinkSendNoticeEnable(CAL_DOWN_DONE); 
			}
            //竖直旋转一圈
			if(mag.cali.step == 2 && abs(cali_rotate_angle ) > 360)
			{
				cnt_m = 0;
				mag.cali.should_cali = 0;
				mag.cali.step = 3;
				cali_rotate_angle  = 0;
                earthMag = 0;
                
                //计算当地地磁场强度模值均值
                for(u8 i=0;i<6;i++)
                {
                    earthMag += Pythagorous3(samples[i].x, samples[i].y, samples[i].z);
                }
                earthMag /= 6;
                
                //LM法求解传感器误差方程最优解
                LevenbergMarquardt(samples, &new_offset, &new_scale, earthMag);
                
                //判断校准参数是否正常
                if(isnan(new_scale.x) || isnan(new_scale.y) || isnan(new_scale.z))
                {
                    mag.cali.success = false;
                }
                else if(fabsf(new_scale.x-1.0f) > 0.8f || fabsf(new_scale.y-1.0f) > 0.8f || fabsf(new_scale.z-1.0f) > 0.8f) 
                {
                    mag.cali.success = false;
                }
                else if(fabsf(new_offset.x) > (earthMag * 0.8f) || fabsf(new_offset.y) > (earthMag * 0.8f) || fabsf(new_offset.z) > (earthMag * 0.8f)) 
                {
                    mag.cali.success = false;
                }
				else
				{
					mag.cali.success = true;
				}
                
                if(mag.cali.success)
                {
                    mag.cali.offset = new_offset;
                    mag.cali.scale = new_scale;
                    mag.earthMag = earthMag;
                    
                    //保存校准参数
                    ParamUpdateData(PARAM_MAG_OFFSET_X, &mag.cali.offset.x);
                    ParamUpdateData(PARAM_MAG_OFFSET_Y, &mag.cali.offset.y);
                    ParamUpdateData(PARAM_MAG_OFFSET_Z, &mag.cali.offset.z);
                    ParamUpdateData(PARAM_MAG_SCALE_X, &mag.cali.scale.x);
                    ParamUpdateData(PARAM_MAG_SCALE_Y, &mag.cali.scale.y);
                    ParamUpdateData(PARAM_MAG_SCALE_Z, &mag.cali.scale.z);
                    ParamUpdateData(PARAM_MAG_EARTH_MAG, &mag.earthMag);
                    //更新mavlink参数
                    MavParamSetValue(CAL_MAG0_XOFF, mag.cali.offset.x);
                    MavParamSetValue(CAL_MAG0_YOFF, mag.cali.offset.y);
                    MavParamSetValue(CAL_MAG0_ZOFF, mag.cali.offset.z);
                    MavParamSetValue(CAL_MAG0_XSCALE, mag.cali.scale.x);
                    MavParamSetValue(CAL_MAG0_YSCALE, mag.cali.scale.y);
                    MavParamSetValue(CAL_MAG0_XSCALE, mag.cali.scale.z);
                    
                    //mavlink发送校准结果
                    MavlinkSendNoticeEnable(CAL_DONE);
                }
                else
                {
                    //mavlink发送校准结果
                    MavlinkSendNoticeEnable(CAL_FAILED);
                }
				
				//bsklink发送校准结果
				MessageSensorCaliFeedbackEnable(MAG, mag.cali.step, mag.cali.success);
				
				mag.cali.step = 0;
                earthMag = 0;
			}
		}
	}
}

/**********************************************************************************************************
*函 数 名: MagGetData
*功能说明: 获取经过处理后的磁力计数据
*形    参: 无 
*返 回 值: 磁力计数据
**********************************************************************************************************/
Vector3f_t MagGetData(void)
{
    return mag.data;
}

/**********************************************************************************************************
*函 数 名: GetMagOffsetCaliData
*功能说明: 获取磁力计零偏校准数据
*形    参: 无 
*返 回 值: 校准参数
**********************************************************************************************************/
Vector3f_t GetMagOffsetCaliData(void)
{
    return mag.cali.offset;
}

/**********************************************************************************************************
*函 数 名: GetAccScaleCaliData
*功能说明: 获取磁力计比例校准数据
*形    参: 无 
*返 回 值: 校准参数
**********************************************************************************************************/
Vector3f_t GetMagScaleCaliData(void)
{
    return mag.cali.scale;
}

/**********************************************************************************************************
*函 数 名: MagDetectCheck
*功能说明: 检测磁力计工作是否正常，通过检测传感器原始数据变化来判断
*形    参: 磁力计原始数据 
*返 回 值: 无
**********************************************************************************************************/
static void MagDetectCheck(Vector3f_t magRaw)
{
	static uint32_t cnt;
	static Vector3f_t lastMagRaw;
	
	if((magRaw.x == lastMagRaw.x) && (magRaw.y == lastMagRaw.y) && (magRaw.z == lastMagRaw.z))
	{
		cnt++;
		
		if(cnt > 50)
		{
			//未检测到磁力计
			FaultDetectSetError(MAG_UNDETECTED);
		}
	}
	else
	{
		cnt = 0;
		FaultDetectResetError(MAG_UNDETECTED);
	}
	
	lastMagRaw = magRaw;	
}

/**********************************************************************************************************
*函 数 名: MagCalibrateEnable
*功能说明: 磁力计校准使能
*形    参: 无 
*返 回 值: 无
**********************************************************************************************************/
void MagCalibrateEnable(void)
{
	mag.cali.should_cali = 1;
}




