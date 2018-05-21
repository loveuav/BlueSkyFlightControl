/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     flightControl.c
 * @说明     飞行控制，由于飞控硬件没完成，飞机也还没组装，暂时无法测试
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "flightControl.h"
#include "flightStatus.h"
#include "board.h"
#include "ahrs.h"
#include "navigation.h"
#include "motor.h"

FLIGHTCONTROL_t fc;

/**********************************************************************************************************
*函 数 名: AttitudeInnerControl
*功能说明: 姿态内环控制
*形    参: 角速度测量值 运行时间间隔
*返 回 值: 姿态内环控制量
**********************************************************************************************************/
static Vector3f_t AttitudeInnerControl(Vector3f_t gyro, float deltaT)
{
	Vector3f_t rateError;
	Vector3f_t rateControlOutput;
    
    //计算角速度环控制误差：目标角速度 - 实际角速度（低通滤波后的陀螺仪测量值）
	rateError.x = fc.attInnerTarget.x - gyro.x;	
	rateError.y = fc.attInnerTarget.y - gyro.y;	
	rateError.z = fc.attInnerTarget.z - gyro.z;		
		
    //PID算法，计算出角速度环的控制量
	rateControlOutput.x = PID_GetPID(&fc.pid[ROLL_INNER], rateError.x, deltaT);
	rateControlOutput.y = PID_GetPID(&fc.pid[PITCH_INNER], rateError.y, deltaT);
	rateControlOutput.z = PID_GetPID(&fc.pid[YAW_INNER], rateError.z, deltaT);
	
    //限制偏航轴控制输出量
	rateControlOutput.z = -ConstrainInt32(rateControlOutput.z, -500, +500);	

    return rateControlOutput;
}

/**********************************************************************************************************
*函 数 名: SetAttInnerCtlTarget
*功能说明: 设置姿态内环控制目标量
*形    参: 控制目标值
*返 回 值: 无
**********************************************************************************************************/
void SetAttInnerCtlTarget(Vector3f_t target)
{
    fc.attInnerTarget = target;
}

/**********************************************************************************************************
*函 数 名: AltitudeInnerControl
*功能说明: 高度内环控制
*形    参: Z轴速度测量值 运行时间间隔
*返 回 值: 高度内环控制量
**********************************************************************************************************/
static float AltitudeInnerControl(float velZ, float deltaT)
{
	float velError, velLpf;
    float altInnerControlOutput;
    //悬停油门中点
	int16_t throttleMid = 1000;
	
    //对速度测量值进行低通滤波，减少数据噪声对控制器的影响
    velLpf = velLpf * 0.992f + velZ * 0.008f;
    
    //计算控制误差
	velError = fc.velInnerTarget.z - velLpf;

    //PID算法，计算出高度内环（Z轴速度）的控制量
	altInnerControlOutput = PID_GetP(&fc.pid[VEL_Z], velError);
	altInnerControlOutput += PID_GetI(&fc.pid[VEL_Z], velError, deltaT);
	altInnerControlOutput += ConstrainInt32(PID_GetD(&fc.pid[VEL_Z], velError, deltaT), -300, 300);
	
	altInnerControlOutput += throttleMid;

    return altInnerControlOutput;
}	

/**********************************************************************************************************
*函 数 名: SetAltInnerCtlTarget
*功能说明: 设置高度内环控制目标量
*形    参: 控制目标值
*返 回 值: 无
**********************************************************************************************************/
void SetAltInnerCtlTarget(float target)
{
    fc.velInnerTarget.z = target;
}

/**********************************************************************************************************
*函 数 名: FlightControlInnerLoop
*功能说明: 飞行内环控制，包括姿态内环和高度内环控制
*          该环节是所有上层控制的基础，因此把该模块独立出来，以1KHz的固定频率运行
*形    参: 角速度测量值
*返 回 值: 无
**********************************************************************************************************/
void FlightControlInnerLoop(Vector3f_t gyro)
{
    //计算函数运行时间间隔
	static uint32_t previousT;
	float deltaT = (GetSysTimeUs() - previousT) * 1e-6;	
	previousT = GetSysTimeUs();	    
    
    //姿态内环控制量
    Vector3f_t attInnerCtlValue;
    //高度内环控制量
    float      altInnerCtlValue;
    
    //姿态内环控制
    attInnerCtlValue = AttitudeInnerControl(gyro, deltaT);
    
    //高度内环控制
    //在手动模式下（MANUAL），油门直接由摇杆数据控制
    if(0)
    {
        //altInnerCtlValue = rcData;
    }
    else
    {
        altInnerCtlValue = AltitudeInnerControl(GetCopterVelocity().z, deltaT);
    }
    
    //将内环控制量转换为动力电机输出
    MotorControl(attInnerCtlValue.x, attInnerCtlValue.y, attInnerCtlValue.z, altInnerCtlValue);
}




