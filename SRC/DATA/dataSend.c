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
#include "bsklink.h"

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

/**********************************************************************************************************
*函 数 名: BsklinkSendFlightData
*功能说明: 发送基本飞行数据
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendFlightData(void)
{
	BSKLINK_MSG_t msg;
	BSKLINK_PAYLOAD_FLIGHT_DATA_t payload;
	uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];
	
	//数据负载填充
	payload.angleRoll  = GetCopterAngle().x * 10;
	payload.anglePitch = GetCopterAngle().y * 10;
	payload.angleYaw   = GetCopterAngle().z * 10;
	payload.accelX     = GetCopterAccel().x * GRAVITY_ACCEL * 100;
	payload.accelY     = GetCopterAccel().y * GRAVITY_ACCEL * 100;
	payload.accelZ     = GetCopterAccel().z * GRAVITY_ACCEL * 100;
	payload.velocityX  = GetCopterVelocity().x;
	payload.velocityY  = GetCopterVelocity().y;
	payload.velocityZ  = GetCopterVelocity().z;
	payload.positionX  = GetCopterPosition().x;
	payload.positionY  = GetCopterPosition().y;
	payload.positionZ  = GetCopterPosition().z;

	/*********************************************消息帧赋值******************************************/
	msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头	
	msg.head2 	 = BSKLINK_MSG_HEAD_2;
	msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
    
	msg.msgid 	 = BSKLINK_MSG_ID_FLIGHT_DATA;                   //消息ID
	msg.length   = sizeof(BSKLINK_PAYLOAD_FLIGHT_DATA_t);        //数据负载长度
	memcpy(msg.payload, &payload, msg.length);                   //拷贝数据负载
	
	BsklinkMsgCalculateSum(&msg);                                //计算校验和
	/*************************************************************************************************/
    
	//消息帧格式化
	BsklinkMsgFormat(msg, msgToSend);
	//发送消息帧
	DataSend(msgToSend+1, msgToSend[0]);
}

/**********************************************************************************************************
*函 数 名: BsklinkSendImuSensor
*功能说明: 发送IMU传感器数据
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendImuSensor(void)
{
	BSKLINK_MSG_t msg;
	BSKLINK_PAYLOAD_IMU_SENSOR_t payload;
	uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];
	
	//数据负载填充
	payload.gyroX 	 = GyroGetData().x * 10;
	payload.gyroY 	 = GyroGetData().y * 10;
	payload.gyroZ 	 = GyroGetData().z * 10;
	payload.gyroLpfX = GyroLpfGetData().x * 10;
	payload.gyroLpfY = GyroLpfGetData().y * 10;
	payload.gyroLpfZ = GyroLpfGetData().z * 10;
	payload.accX     = AccGetData().x * 1000;
	payload.accY     = AccGetData().y * 1000;
	payload.accZ     = AccGetData().z * 1000;
	payload.accLpfX  = AccLpfGetData().x * 1000;
	payload.accLpfY  = AccLpfGetData().y * 1000;
	payload.accLpfZ  = AccLpfGetData().z * 1000;

	/*********************************************消息帧赋值******************************************/
	msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头	
	msg.head2 	 = BSKLINK_MSG_HEAD_2;
	msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
    
	msg.msgid 	 = BSKLINK_MSG_ID_IMU_SENSOR;                    //消息ID
	msg.length   = sizeof(BSKLINK_PAYLOAD_IMU_SENSOR_t);         //数据负载长度
	memcpy(msg.payload, &payload, msg.length);                   //拷贝数据负载
	
	BsklinkMsgCalculateSum(&msg);                                //计算校验和
	/*************************************************************************************************/
	
	//消息帧格式化
	BsklinkMsgFormat(msg, msgToSend);
	//发送消息帧
	DataSend(msgToSend+1, msgToSend[0]);
}

/**********************************************************************************************************
*函 数 名: BsklinkSendGps
*功能说明: 发送GPS数据
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendGps(void)
{
	BSKLINK_MSG_t msg;
	BSKLINK_PAYLOAD_GPS_t payload;
	uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];
	
	//数据负载填充
	payload.time  	  = Ublox_GetData().time;
	payload.numSV 	  = Ublox_GetData().numSV;
	payload.hAcc 	  = Ublox_GetData().hAcc * 100;
	payload.vAcc 	  = Ublox_GetData().vAcc * 100;
	payload.latitude  = Ublox_GetData().latitude;
	payload.longitude = Ublox_GetData().longitude;
	payload.altitude  = Ublox_GetData().altitude;
	payload.velN	  = Ublox_GetData().velN;
	payload.velE	  = Ublox_GetData().velE;
	payload.velD	  = Ublox_GetData().velD;

	/*********************************************消息帧赋值******************************************/
	msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头	
	msg.head2 	 = BSKLINK_MSG_HEAD_2;
	msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
    
	msg.msgid 	 = BSKLINK_MSG_ID_GPS;                           //消息ID
	msg.length   = sizeof(BSKLINK_PAYLOAD_GPS_t);                //数据负载长度
	memcpy(msg.payload, &payload, msg.length);                   //拷贝数据负载
	
	BsklinkMsgCalculateSum(&msg);                                //计算校验和
	/*************************************************************************************************/
	
	//消息帧格式化
	BsklinkMsgFormat(msg, msgToSend);
	//发送消息帧
	DataSend(msgToSend+1, msgToSend[0]);
}

/**********************************************************************************************************
*函 数 名: BsklinkSendRcData
*功能说明: 发送遥控通道数据
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendRcData(void)
{
	BSKLINK_MSG_t msg;
	BSKLINK_PAYLOAD_RC_DATA_t payload;
	uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];
	
	//数据负载填充
	payload.roll 	 = GetRcData().roll;
	payload.pitch 	 = GetRcData().pitch;
	payload.yaw 	 = GetRcData().yaw;
	payload.throttle = GetRcData().throttle;
	payload.aux1	 = GetRcData().aux1;
	payload.aux2	 = GetRcData().aux2;
	payload.aux3	 = GetRcData().aux3;
	payload.aux4	 = GetRcData().aux4;
	payload.aux5	 = GetRcData().aux5;
	payload.aux6	 = GetRcData().aux6;
	payload.aux7	 = GetRcData().aux7;
	payload.aux8	 = GetRcData().aux8;

	/*********************************************消息帧赋值******************************************/
	msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头	
	msg.head2 	 = BSKLINK_MSG_HEAD_2;
	msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
    
	msg.msgid 	 = BSKLINK_MSG_ID_RC_DATA;                       //消息ID
	msg.length   = sizeof(BSKLINK_PAYLOAD_RC_DATA_t);            //数据负载长度
	memcpy(msg.payload, &payload, msg.length);                   //拷贝数据负载
	
	BsklinkMsgCalculateSum(&msg);                                //计算校验和
	/*************************************************************************************************/
	
	//消息帧格式化
	BsklinkMsgFormat(msg, msgToSend);
	//发送消息帧
	DataSend(msgToSend+1, msgToSend[0]);
}

/**********************************************************************************************************
*函 数 名: BsklinkSendPidAttInner
*功能说明: 发送姿态内环PID
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendPidAttInner(void)
{
	BSKLINK_MSG_t msg;
	BSKLINK_PAYLOAD_PID_ATT_INNER_t payload;
	uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];
	
	//数据负载填充
	payload.roll_kp  = FcGetPID(ROLL_INNER).kP;
	payload.roll_ki  = FcGetPID(ROLL_INNER).kI;
	payload.roll_kd  = FcGetPID(ROLL_INNER).kD;
	payload.pitch_kp = FcGetPID(PITCH_INNER).kP;
	payload.pitch_ki = FcGetPID(PITCH_INNER).kI;
	payload.pitch_kd = FcGetPID(PITCH_INNER).kD;
	payload.yaw_kp   = FcGetPID(YAW_INNER).kP;
	payload.yaw_ki   = FcGetPID(YAW_INNER).kI;
	payload.yaw_kd   = FcGetPID(YAW_INNER).kD;

	/*********************************************消息帧赋值******************************************/
	msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头	
	msg.head2 	 = BSKLINK_MSG_HEAD_2;
	msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
    
	msg.msgid 	 = BSKLINK_MSG_ID_PID_ATT_INNER;                 //消息ID
	msg.length   = sizeof(BSKLINK_PAYLOAD_PID_ATT_INNER_t);      //数据负载长度    
	memcpy(msg.payload, &payload, msg.length);                   //拷贝数据负载
	
	BsklinkMsgCalculateSum(&msg);                                //计算校验和
	/*************************************************************************************************/
	
	//消息帧格式化
	BsklinkMsgFormat(msg, msgToSend);
	//发送消息帧
	DataSend(msgToSend+1, msgToSend[0]);
}

/**********************************************************************************************************
*函 数 名: BsklinkPidAttOuter
*功能说明: 发送姿态外环PID
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void BsklinkPidAttOuter(void)
{
	BSKLINK_MSG_t msg;
	BSKLINK_PAYLOAD_PID_ATT_OUTER_t payload;
	uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];
	
	//数据负载填充
	payload.roll_kp  = FcGetPID(ROLL_OUTER).kP;
	payload.pitch_kp = FcGetPID(PITCH_OUTER).kP;
	payload.yaw_kp   = FcGetPID(YAW_OUTER).kP;

	/*********************************************消息帧赋值******************************************/
	msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头	
	msg.head2 	 = BSKLINK_MSG_HEAD_2;
	msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
    
	msg.msgid 	 = BSKLINK_MSG_ID_PID_ATT_OUTER;                 //消息ID
	msg.length   = sizeof(BSKLINK_PAYLOAD_PID_ATT_OUTER_t);      //数据负载长度   
	memcpy(msg.payload, &payload, msg.length);                   //拷贝数据负载
	
	BsklinkMsgCalculateSum(&msg);                                //计算校验和
	/*************************************************************************************************/
	
	//消息帧格式化
	BsklinkMsgFormat(msg, msgToSend);
	//发送消息帧
	DataSend(msgToSend+1, msgToSend[0]);
}

/**********************************************************************************************************
*函 数 名: BsklinkSendPidPosInner
*功能说明: 发送位置内环PID
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendPidPosInner(void)
{
	BSKLINK_MSG_t msg;
	BSKLINK_PAYLOAD_PID_POS_INNER_t payload;
	uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];
	
	//数据负载填充
	payload.velX_kp = FcGetPID(VEL_X).kP;
	payload.velX_ki = FcGetPID(VEL_X).kI;
	payload.velX_kd = FcGetPID(VEL_X).kD;
	payload.velY_kp = FcGetPID(VEL_Y).kP;
	payload.velY_ki = FcGetPID(VEL_Y).kI;
	payload.velY_kd = FcGetPID(VEL_Y).kD;
	payload.velZ_kp = FcGetPID(VEL_Z).kP;
	payload.velZ_ki = FcGetPID(VEL_Z).kI;
	payload.velZ_kd = FcGetPID(VEL_Z).kD;
	
	/*********************************************消息帧赋值******************************************/
	msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头	
	msg.head2 	 = BSKLINK_MSG_HEAD_2;
	msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
    
	msg.msgid 	 = BSKLINK_MSG_ID_PID_POS_INNER;                 //消息ID
	msg.length   = sizeof(BSKLINK_PAYLOAD_PID_POS_INNER_t);      //数据负载长度  
	memcpy(msg.payload, &payload, msg.length);                   //拷贝数据负载
	
	BsklinkMsgCalculateSum(&msg);                                //计算校验和
	/*************************************************************************************************/
    
	//消息帧格式化
	BsklinkMsgFormat(msg, msgToSend);
	//发送消息帧
	DataSend(msgToSend+1, msgToSend[0]);
}

/**********************************************************************************************************
*函 数 名: BsklinkSendPidPosOuter
*功能说明: 发送位置外环PID
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendPidPosOuter(void)
{
	BSKLINK_MSG_t msg;
	BSKLINK_PAYLOAD_PID_POS_OUTER_t payload;
	uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];
	
	//数据负载填充
	payload.posX_kp = FcGetPID(POS_X).kP;
	payload.posY_kp = FcGetPID(POS_Y).kP;
	payload.posZ_kp = FcGetPID(POS_Z).kP;
	
	/*********************************************消息帧赋值******************************************/
	msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头	
	msg.head2 	 = BSKLINK_MSG_HEAD_2;
	msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
    
	msg.msgid 	 = BSKLINK_MSG_ID_PID_POS_OUTER;                 //消息ID
	msg.length   = sizeof(BSKLINK_PAYLOAD_PID_POS_OUTER_t);      //数据负载长度   
	memcpy(msg.payload, &payload, msg.length);                   //拷贝数据负载
	
	BsklinkMsgCalculateSum(&msg);                                //计算校验和
	/*************************************************************************************************/
    
	//消息帧格式化
	BsklinkMsgFormat(msg, msgToSend);
	//发送消息帧
	DataSend(msgToSend+1, msgToSend[0]);
}



