/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     bsklinkSend.c
 * @说明     bsklink数据帧发送
 * @版本  	 V1.1
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.07
**********************************************************************************************************/
#include "bsklinkSend.h"
#include "message.h"
#include "bsklink.h"

#include "board.h"
#include "ahrs.h"
#include "navigation.h"
#include "flightControl.h"
#include "gyroscope.h"
#include "magnetometer.h"
#include "accelerometer.h"
#include "barometer.h"
#include "motor.h"
#include "rc.h"
#include "gps.h"
#include "ublox.h"
#include "flightStatus.h"

/**********************************************************************************************************
*函 数 名: BsklinkSendFlightData
*功能说明: 发送基本飞行数据
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendFlightData(uint8_t* sendFlag)
{
	BSKLINK_MSG_t msg;
	BSKLINK_PAYLOAD_FLIGHT_DATA_t payload;
	uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];
	
    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;

	//数据负载填充
    payload.angle.x      = GetCopterAngle().x * 10;
	payload.angle.y      = GetCopterAngle().y * 10;
	payload.angle.z      = GetCopterAngle().z * 10;
	payload.accel.x      = GetCopterAccel().x * GRAVITY_ACCEL * 100;
	payload.accel.y      = GetCopterAccel().y * GRAVITY_ACCEL * 100;
	payload.accel.z	     = GetCopterAccel().z * GRAVITY_ACCEL * 100;
	payload.accelLpf.x	 = GetCopterAccEfLpf().x * GRAVITY_ACCEL * 100;
	payload.accelLpf.y	 = GetCopterAccEfLpf().y * GRAVITY_ACCEL * 100;
	payload.accelLpf.z	 = GetCopterAccEfLpf().z * GRAVITY_ACCEL * 100;
	payload.velocity.x	 = GetCopterVelocity().x;
	payload.velocity.y	 = GetCopterVelocity().y;
	payload.velocity.z   = GetCopterVelocity().z;
	payload.velMeasure.x = GetCopterVelMeasure().x;
	payload.velMeasure.y = GetCopterVelMeasure().y;
	payload.velMeasure.z = GetCopterVelMeasure().z;
	payload.position.x   = GetCopterPosition().x;
	payload.position.y   = GetCopterPosition().y;
	payload.position.z   = GetCopterPosition().z;
	payload.posMeasure.x = GetCopterPosMeasure().x;
	payload.posMeasure.y = GetCopterPosMeasure().y;
	payload.posMeasure.z = GetCopterPosMeasure().z;
	
	/*********************************************消息帧赋值******************************************/
	msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头	
	msg.head2 	 = BSKLINK_MSG_HEAD_2;
	msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
	msg.sysid 	 = BSKLINK_SYS_ID;							     //系统ID
    
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
*函 数 名: BsklinkSendFlightStatus
*功能说明: 发送飞行状态
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendFlightStatus(uint8_t* sendFlag)
{
	BSKLINK_MSG_t msg;
	BSKLINK_PAYLOAD_FLIGHT_STATUS_t payload;
	uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
	
	//数据负载填充
    payload.flightMode    = GetFlightMode();
	payload.initStatus	  = GetInitStatus();		  
	payload.armedStatus   = GetArmedStatus();	
	payload.flightStatus  = GetFlightStatus();
	payload.placeStatus	  = GetPlaceStatus();	
	payload.altCtlStatus  = GetAltControlStatus();
	payload.posCtlStatus  = GetPosControlStatus();    
	
	/*********************************************消息帧赋值******************************************/
	msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头	
	msg.head2 	 = BSKLINK_MSG_HEAD_2;
	msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
	msg.sysid 	 = BSKLINK_SYS_ID;							     //系统ID
    
	msg.msgid 	 = BSKLINK_MSG_ID_FLIGHT_STATUS;                 //消息ID
	msg.length   = sizeof(BSKLINK_PAYLOAD_FLIGHT_STATUS_t);      //数据负载长度
	memcpy(msg.payload, &payload, msg.length);                   //拷贝数据负载
	
	BsklinkMsgCalculateSum(&msg);                                //计算校验和
	/*************************************************************************************************/
    
	//消息帧格式化
	BsklinkMsgFormat(msg, msgToSend);
	//发送消息帧
	DataSend(msgToSend+1, msgToSend[0]);
}

/**********************************************************************************************************
*函 数 名: BsklinkSendSensor
*功能说明: 发送传感器数据
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendSensor(uint8_t* sendFlag)
{
	BSKLINK_MSG_t msg;
	BSKLINK_PAYLOAD_SENSOR_t payload;
	uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
	
	//数据负载填充
	payload.gyro.x 	  	= GyroGetData().x * 10;
	payload.gyro.y 	  	= GyroGetData().y * 10;
	payload.gyro.z 	  	= GyroGetData().z * 10;
	payload.gyroLpf.x 	= GyroLpfGetData().x * 10;
	payload.gyroLpf.y 	= GyroLpfGetData().y * 10;
	payload.gyroLpf.z 	= GyroLpfGetData().z * 10;
	payload.gyroTemp    = GyroGetTemp() * 100;
	payload.gyro_offset = 0;
	payload.acc.x     	= AccGetData().x * 1000;
	payload.acc.y     	= AccGetData().y * 1000;
	payload.acc.z     	= AccGetData().z * 1000;
	payload.accLpf.x  	= AccLpfGetData().x * 1000;
	payload.accLpf.y  	= AccLpfGetData().y * 1000;
	payload.accLpf.z  	= AccLpfGetData().z * 1000;
	payload.acc_offset  = 0;
	payload.mag.x	  	= MagGetData().x * 1000;
	payload.mag.y	  	= MagGetData().y * 1000;
	payload.mag.z	  	= MagGetData().z * 1000;
	payload.mag_offset  = 0;
	payload.baroAlt   	= BaroGetAlt();
	payload.baroTemp  	= BaroGetTemp() * 100;
	
	/*********************************************消息帧赋值******************************************/
	msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头	
	msg.head2 	 = BSKLINK_MSG_HEAD_2;
	msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
	msg.sysid 	 = BSKLINK_SYS_ID;							     //系统ID
    
	msg.msgid 	 = BSKLINK_MSG_ID_SENSOR;                        //消息ID
	msg.length   = sizeof(BSKLINK_PAYLOAD_SENSOR_t);             //数据负载长度
	memcpy(msg.payload, &payload, msg.length);                   //拷贝数据负载
	
	BsklinkMsgCalculateSum(&msg);                                //计算校验和
	/*************************************************************************************************/
	
	//消息帧格式化
	BsklinkMsgFormat(msg, msgToSend);
	//发送消息帧
	DataSend(msgToSend+1, msgToSend[0]);
}

/**********************************************************************************************************
*函 数 名: BsklinkSendSensorCaliData
*功能说明: 发送传感器校准数据
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendSensorCaliData(uint8_t* sendFlag)
{
	BSKLINK_MSG_t msg;
	BSKLINK_PAYLOAD_SENSOR_CALI_DATA_t payload;
	uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
	
	//数据负载填充
    payload.gyro_offset.x = GetGyroOffsetCaliData().x;
    payload.gyro_offset.y = GetGyroOffsetCaliData().y;
    payload.gyro_offset.z = GetGyroOffsetCaliData().z;
    payload.acc_offset.x  = GetAccOffsetCaliData().x;
    payload.acc_offset.y  = GetAccOffsetCaliData().y;
    payload.acc_offset.z  = GetAccOffsetCaliData().z;
    payload.acc_scale.x   = GetAccScaleCaliData().x;
    payload.acc_scale.y   = GetAccScaleCaliData().y;
    payload.acc_scale.z   = GetAccScaleCaliData().z;
    payload.mag_offset.x  = GetMagOffsetCaliData().x;
    payload.mag_offset.y  = GetMagOffsetCaliData().y;
    payload.mag_offset.z  = GetMagOffsetCaliData().z;
    payload.mag_scale.x   = GetMagScaleCaliData().x;
    payload.mag_scale.y   = GetMagScaleCaliData().y;
    payload.mag_scale.z   = GetMagScaleCaliData().z;	
    payload.angle.x       = Degrees(GetLevelCalibraData().x);
    payload.angle.y       = Degrees(GetLevelCalibraData().y);
    
	/*********************************************消息帧赋值******************************************/
	msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头	
	msg.head2 	 = BSKLINK_MSG_HEAD_2;
	msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
	msg.sysid 	 = BSKLINK_SYS_ID;							     //系统ID
    
	msg.msgid 	 = BSKLINK_MSG_ID_SENSOR_CALI_DATA;              //消息ID
	msg.length   = sizeof(BSKLINK_PAYLOAD_SENSOR_CALI_DATA_t);   //数据负载长度
	memcpy(msg.payload, &payload, msg.length);                   //拷贝数据负载
	
	BsklinkMsgCalculateSum(&msg);                                //计算校验和
	/*************************************************************************************************/
	
	//消息帧格式化
	BsklinkMsgFormat(msg, msgToSend);
	//发送消息帧
	DataSend(msgToSend+1, msgToSend[0]);
}

/**********************************************************************************************************
*函 数 名: BsklinkSendSensorCaliCmd
*功能说明: 发送传感器校准反馈
*形    参: 发送标志指针 传感器类型 当前步骤 成功标志位
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendSensorCaliCmd(uint8_t* sendFlag, uint8_t type, uint8_t step, uint8_t success)
{
	BSKLINK_MSG_t msg;
	BSKLINK_PAYLOAD_SENSOR_CALI_CMD_t payload;
	uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
	
	//数据负载填充
	payload.type 		= type;
	payload.step		= step;
	payload.successFlag = success;
    
	/*********************************************消息帧赋值******************************************/
	msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头	
	msg.head2 	 = BSKLINK_MSG_HEAD_2;
	msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
	msg.sysid 	 = BSKLINK_SYS_ID;							     //系统ID
    
	msg.msgid 	 = BSKLINK_MSG_ID_SENSOR_CALI_CMD;               //消息ID
	msg.length   = sizeof(BSKLINK_PAYLOAD_SENSOR_CALI_CMD_t);    //数据负载长度
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
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendGps(uint8_t* sendFlag)
{
	BSKLINK_MSG_t msg;
	BSKLINK_PAYLOAD_GPS_t payload;
	uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
	
	//数据负载填充
	payload.time  	  = Ublox_GetData().time;
	payload.numSV 	  = Ublox_GetData().numSV;
    payload.fixStatus = Ublox_GetData().fixStatus;
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
	msg.sysid 	 = BSKLINK_SYS_ID;							     //系统ID
    
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
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendRcData(uint8_t* sendFlag)
{
	BSKLINK_MSG_t msg;
	BSKLINK_PAYLOAD_RC_DATA_t payload;
	uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
    
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
	msg.sysid 	 = BSKLINK_SYS_ID;							     //系统ID
    
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
*函 数 名: BsklinkSendMotor
*功能说明: 发送电机输出数据
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendMotor(uint8_t* sendFlag)
{
	BSKLINK_MSG_t msg;
	BSKLINK_PAYLOAD_MOTOR_t payload;
	uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
    
	//数据负载填充
    payload.num = GetMotorNum();
    payload.motorValue1 = GetMotorValue()[0];
    payload.motorValue2 = GetMotorValue()[1];
    payload.motorValue3 = GetMotorValue()[2];
    payload.motorValue4 = GetMotorValue()[3];
    payload.motorValue5 = GetMotorValue()[4];
    payload.motorValue6 = GetMotorValue()[5];
    payload.motorValue7 = GetMotorValue()[6];
    payload.motorValue8 = GetMotorValue()[7];
    
	/*********************************************消息帧赋值******************************************/
	msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头	
	msg.head2 	 = BSKLINK_MSG_HEAD_2;
	msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
	msg.sysid 	 = BSKLINK_SYS_ID;							     //系统ID
    
	msg.msgid 	 = BSKLINK_MSG_ID_MOTOR;                         //消息ID
	msg.length   = sizeof(BSKLINK_PAYLOAD_MOTOR_t);              //数据负载长度
	memcpy(msg.payload, &payload, msg.length);                   //拷贝数据负载
	
	BsklinkMsgCalculateSum(&msg);                                //计算校验和
	/*************************************************************************************************/
	
	//消息帧格式化
	BsklinkMsgFormat(msg, msgToSend);
	//发送消息帧
	DataSend(msgToSend+1, msgToSend[0]);
}

/**********************************************************************************************************
*函 数 名: BsklinkSendPidAtt
*功能说明: 发送姿态PID
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendPidAtt(uint8_t* sendFlag)
{
	BSKLINK_MSG_t msg;
	BSKLINK_PAYLOAD_PID_ATT_t payload;
	uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
	
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
    payload.rollAngle_kp  = FcGetPID(ROLL_OUTER).kP;
	payload.pitchAngle_kp = FcGetPID(PITCH_OUTER).kP;
	payload.yawAngle_kp   = FcGetPID(YAW_OUTER).kP;
    
	/*********************************************消息帧赋值******************************************/
	msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头	
	msg.head2 	 = BSKLINK_MSG_HEAD_2;
	msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
	msg.sysid 	 = BSKLINK_SYS_ID;							     //系统ID
    
	msg.msgid 	 = BSKLINK_MSG_ID_PID_ATT;                       //消息ID
	msg.length   = sizeof(BSKLINK_PAYLOAD_PID_ATT_t);            //数据负载长度    
	memcpy(msg.payload, &payload, msg.length);                   //拷贝数据负载
	
	BsklinkMsgCalculateSum(&msg);                                //计算校验和
	/*************************************************************************************************/
	
	//消息帧格式化
	BsklinkMsgFormat(msg, msgToSend);
	//发送消息帧
	DataSend(msgToSend+1, msgToSend[0]);
}

/**********************************************************************************************************
*函 数 名: BsklinkSendPidPos
*功能说明: 发送位置PID
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendPidPos(uint8_t* sendFlag)
{
	BSKLINK_MSG_t msg;
	BSKLINK_PAYLOAD_PID_POS_t payload;
	uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
	
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
	payload.posX_kp = FcGetPID(POS_X).kP;
	payload.posY_kp = FcGetPID(POS_Z).kP;
	payload.posZ_kp = FcGetPID(POS_Z).kP;	
	
	/*********************************************消息帧赋值******************************************/
	msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头	
	msg.head2 	 = BSKLINK_MSG_HEAD_2;
	msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
 	msg.sysid 	 = BSKLINK_SYS_ID;							     //系统ID
	
	msg.msgid 	 = BSKLINK_MSG_ID_PID_POS;                       //消息ID
	msg.length   = sizeof(BSKLINK_PAYLOAD_PID_POS_t);            //数据负载长度  
	memcpy(msg.payload, &payload, msg.length);                   //拷贝数据负载
	
	BsklinkMsgCalculateSum(&msg);                                //计算校验和
	/*************************************************************************************************/
    
	//消息帧格式化
	BsklinkMsgFormat(msg, msgToSend);
	//发送消息帧
	DataSend(msgToSend+1, msgToSend[0]);
}

/**********************************************************************************************************
*函 数 名: BsklinkSendHeartBeat
*功能说明: 发送心跳包
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendHeartBeat(uint8_t* sendFlag)
{
	BSKLINK_MSG_t msg;
	BSKLINK_PAYLOAD_HEARTBEAT_t payload;
	uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;
	
	//数据负载填充
	payload.type = BOARD_BLUESKY_V3;
	payload.version_high = SOFTWARE_VERSION_HIGH;
	payload.version_mid  = SOFTWARE_VERSION_MID;
	payload.version_low  = SOFTWARE_VERSION_LOW;
	payload.time		 = GetSysTimeMs();
	payload.freq 		 = MAX_SEND_FREQ;
	
	/*********************************************消息帧赋值******************************************/
	msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头	
	msg.head2 	 = BSKLINK_MSG_HEAD_2;
	msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
 	msg.sysid 	 = BSKLINK_SYS_ID;							     //系统ID
	
	msg.msgid 	 = BSKLINK_MSG_ID_HEARTBEAT;                     //消息ID
	msg.length   = sizeof(BSKLINK_PAYLOAD_HEARTBEAT_t);          //数据负载长度  
	memcpy(msg.payload, &payload, msg.length);                   //拷贝数据负载
	
	BsklinkMsgCalculateSum(&msg);                                //计算校验和
	/*************************************************************************************************/
    
	//消息帧格式化
	BsklinkMsgFormat(msg, msgToSend);
	//发送消息帧
	DataSend(msgToSend+1, msgToSend[0]);
}


