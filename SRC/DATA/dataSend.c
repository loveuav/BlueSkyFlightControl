/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     dataSend.c
 * @说明     飞控数据发送
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.06
**********************************************************************************************************/
#include "dataSend.h"
#include "dataCom.h"

#include "ahrs.h"
#include "flightControl.h"
#include "gyroscope.h"
#include "navigation.h"
#include "accelerometer.h"
#include "barometer.h"
#include "motor.h"
#include "rc.h"
#include "gps.h"
#include "ublox.h"

static DATA_TYPE_t dataTemp;  
static uint8_t dataToSend[100];

/**********************************************************************************************************
*函 数 名: SendFlightData
*功能说明: 发送基本飞行数据
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void SendFlightData(void)
{
	uint8_t _cnt=0;

	dataToSend[_cnt++] = FRAME_HEAD_1;
	dataToSend[_cnt++] = FRAME_HEAD_2;
    dataToSend[_cnt++] = DEVICE_TYPE;
    
	dataToSend[_cnt++] = 0x02;
	dataToSend[_cnt++] = 0;
	
	dataTemp.i16 = GetCopterAngle().x * 10;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataTemp.i16 = GetCopterAngle().y * 10;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataTemp.i16 = GetCopterAngle().z * 10;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataTemp.i16 = GetCopterAccel().x * GRAVITY_ACCEL * 100;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataTemp.i16 = GetCopterAccel().y * GRAVITY_ACCEL * 100;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataTemp.i16 = GetCopterAccel().z * GRAVITY_ACCEL * 100;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataTemp.i16 = GetCopterVelocity().x;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataTemp.i16 = GetCopterVelocity().y;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataTemp.i16 = GetCopterVelocity().z;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataTemp.i32 = GetCopterPosition().x;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3];
	dataTemp.i32 = GetCopterPosition().y;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3];
	dataTemp.i32 = GetCopterPosition().z;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3];
    
	dataToSend[4] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += dataToSend[i];
	
	dataToSend[_cnt++]=sum;
	
	DataSend(dataToSend, _cnt);    
}

/**********************************************************************************************************
*函 数 名: SendGpsData
*功能说明: 发送GPS数据
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void SendGpsData(void)
{
	uint8_t _cnt=0;

	dataToSend[_cnt++] = FRAME_HEAD_1;
	dataToSend[_cnt++] = FRAME_HEAD_2;
    dataToSend[_cnt++] = DEVICE_TYPE;
    
	dataToSend[_cnt++] = 0x08;
	dataToSend[_cnt++] = 0;
	
	dataTemp.f32 = Ublox_GetData().time;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3];
	dataTemp.i8  = Ublox_GetData().numSV;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataTemp.i16 = Ublox_GetData().hAcc * 100;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataTemp.i16 = Ublox_GetData().vAcc * 100;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataTemp.i32 = Ublox_GetData().latitude;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
    dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3];
	dataTemp.i32 = Ublox_GetData().longitude;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
    dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3];
	dataTemp.i32 = Ublox_GetData().altitude;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
    dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3];
	dataTemp.i16 = Ublox_GetData().velN;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataTemp.i16 = Ublox_GetData().velE;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataTemp.i16 = Ublox_GetData().velD;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
    
	dataToSend[4] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += dataToSend[i];
	
	dataToSend[_cnt++]=sum;
	
	DataSend(dataToSend, _cnt);    
}

/**********************************************************************************************************
*函 数 名: SendRcData
*功能说明: 发送遥控通道数据
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void SendRcData(void)
{
	uint8_t _cnt=0;

	dataToSend[_cnt++] = FRAME_HEAD_1;
	dataToSend[_cnt++] = FRAME_HEAD_2;
    dataToSend[_cnt++] = DEVICE_TYPE;
    
	dataToSend[_cnt++] = 0x03;
	dataToSend[_cnt++] = 0;
	
	dataTemp.i16 = GetRcData().roll;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataTemp.i16 = GetRcData().pitch;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataTemp.i16 = GetRcData().yaw;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataTemp.i16 = GetRcData().throttle;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataTemp.i16 = GetRcData().aux1;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataTemp.i16 = GetRcData().aux2;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataTemp.i16 = GetRcData().aux3;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataTemp.i16 = GetRcData().aux4;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataTemp.i16 = GetRcData().aux5;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
 	dataTemp.i16 = GetRcData().aux6;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataTemp.i16 = GetRcData().aux7;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataTemp.i16 = GetRcData().aux8;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
    
	dataToSend[4] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += dataToSend[i];
	
	dataToSend[_cnt++]=sum;
	
	DataSend(dataToSend, _cnt);    
}

/**********************************************************************************************************
*函 数 名: SendPidAttInner
*功能说明: 发送姿态内环PID
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void SendPidAttInner(void)
{
	uint8_t _cnt=0;

	dataToSend[_cnt++] = FRAME_HEAD_1;
	dataToSend[_cnt++] = FRAME_HEAD_2;
    dataToSend[_cnt++] = DEVICE_TYPE;
    
	dataToSend[_cnt++] = 0x10;
	dataToSend[_cnt++] = 0;  

    dataTemp.f32 = FcGetPID(ROLL_INNER).kP;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3];    

    dataTemp.f32 = FcGetPID(ROLL_INNER).kI;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3]; 

    dataTemp.f32 = FcGetPID(ROLL_INNER).kD;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3]; 

    dataTemp.f32 = FcGetPID(PITCH_INNER).kP;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3];    

    dataTemp.f32 = FcGetPID(PITCH_INNER).kI;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3]; 

    dataTemp.f32 = FcGetPID(PITCH_INNER).kD;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3]; 
    
    dataTemp.f32 = FcGetPID(YAW_INNER).kP;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3];    

    dataTemp.f32 = FcGetPID(YAW_INNER).kI;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3]; 

    dataTemp.f32 = FcGetPID(YAW_INNER).kD;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3]; 
    
	dataToSend[4] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += dataToSend[i];
	
	dataToSend[_cnt++]=sum;
	
	DataSend(dataToSend, _cnt);      
}

/**********************************************************************************************************
*函 数 名: SendPidAttOuter
*功能说明: 发送姿态外环PID
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void SendPidAttOuter(void)
{
	uint8_t _cnt=0;

	dataToSend[_cnt++] = FRAME_HEAD_1;
	dataToSend[_cnt++] = FRAME_HEAD_2;
    dataToSend[_cnt++] = DEVICE_TYPE;
    
	dataToSend[_cnt++] = 0x11;
	dataToSend[_cnt++] = 0;  

    dataTemp.f32 = FcGetPID(ROLL_OUTER).kP;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3];    

    dataTemp.f32 = FcGetPID(PITCH_OUTER).kP;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3];    
    
    dataTemp.f32 = FcGetPID(YAW_OUTER).kP;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3];    
    
	dataToSend[4] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += dataToSend[i];
	
	dataToSend[_cnt++]=sum;
	
	DataSend(dataToSend, _cnt);      
}

/**********************************************************************************************************
*函 数 名: SendPidPosInner
*功能说明: 发送位置内环PID
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void SendPidPosInner(void)
{
	uint8_t _cnt=0;

	dataToSend[_cnt++] = FRAME_HEAD_1;
	dataToSend[_cnt++] = FRAME_HEAD_2;
    dataToSend[_cnt++] = DEVICE_TYPE;
    
	dataToSend[_cnt++] = 0x12;
	dataToSend[_cnt++] = 0;  

    dataTemp.f32 = FcGetPID(VEL_X).kP;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3];    

    dataTemp.f32 = FcGetPID(VEL_X).kI;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3]; 

    dataTemp.f32 = FcGetPID(VEL_X).kD;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3]; 

    dataTemp.f32 = FcGetPID(VEL_Y).kP;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3];    

    dataTemp.f32 = FcGetPID(VEL_Y).kI;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3]; 

    dataTemp.f32 = FcGetPID(VEL_Y).kD;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3]; 
    
    dataTemp.f32 = FcGetPID(VEL_Z).kP;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3];    

    dataTemp.f32 = FcGetPID(VEL_Z).kI;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3]; 

    dataTemp.f32 = FcGetPID(VEL_Z).kD;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3]; 
    
	dataToSend[4] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += dataToSend[i];
	
	dataToSend[_cnt++]=sum;
	
	DataSend(dataToSend, _cnt);      
}

/**********************************************************************************************************
*函 数 名: SendPidPosOuter
*功能说明: 发送位置外环PID
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void SendPidPosOuter(void)
{
	uint8_t _cnt=0;

	dataToSend[_cnt++] = FRAME_HEAD_1;
	dataToSend[_cnt++] = FRAME_HEAD_2;
    dataToSend[_cnt++] = DEVICE_TYPE;
    
	dataToSend[_cnt++] = 0x13;
	dataToSend[_cnt++] = 0;  

    dataTemp.f32 = FcGetPID(POS_X).kP;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3];    

    dataTemp.f32 = FcGetPID(POS_Y).kP;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3];    
    
    dataTemp.f32 = FcGetPID(POS_Z).kP;
	dataToSend[_cnt++] = dataTemp.byte[0];
	dataToSend[_cnt++] = dataTemp.byte[1];
	dataToSend[_cnt++] = dataTemp.byte[2];
	dataToSend[_cnt++] = dataTemp.byte[3];    
    
	dataToSend[4] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += dataToSend[i];
	
	dataToSend[_cnt++]=sum;
	
	DataSend(dataToSend, _cnt);      
}




