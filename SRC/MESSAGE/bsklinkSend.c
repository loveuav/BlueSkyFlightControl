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
#include "battery.h"
#include "faultDetect.h"

uint8_t pid_ack;

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
    payload.velMeasure.x = GetCopterVelMeasure()[GPS_VEL_X];
    payload.velMeasure.y = GetCopterVelMeasure()[GPS_VEL_Y];
    payload.velMeasure.z = GetCopterVelMeasure()[GPS_VEL_Z];
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
*函 数 名: BsklinkSendBattery
*功能说明: 发送电池数据
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendBattery(uint8_t* sendFlag)
{
    BSKLINK_MSG_t msg;
    BSKLINK_PAYLOAD_BATTERY_t payload;
    uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;

    //数据负载填充
    payload.voltage = GetBatteryVoltage();      //电池电压 单位：0.01V
    payload.current = GetBatteryCurrent();      //电池电流 单位：0.01A
    payload.capacityPer = 0xFF;                 //电池电量百分比（0xFF表示无电量数据）
    payload.capacity = 0;                       //电池设计容量 单位：mah
    payload.capRemain = 0;                      //电池当前剩余容量 单位：mah
    payload.temperature = 0;                    //电池温度 单位：0.01°
    payload.cellNum = 4;                        //电芯节数
    payload.cellVolt[0] = 0;                    //电芯电压（最多6节） 单位：0.01V
    payload.cellVolt[1] = 0;
    payload.cellVolt[2] = 0;
    payload.cellVolt[3] = 0;
    payload.cellVolt[4] = 0;
    payload.cellVolt[5] = 0;

    /*********************************************消息帧赋值******************************************/
    msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头
    msg.head2 	 = BSKLINK_MSG_HEAD_2;
    msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
    msg.sysid 	 = BSKLINK_SYS_ID;							     //系统ID

    msg.msgid 	 = BSKLINK_MSG_ID_BATTERY;                       //消息ID
    msg.length   = sizeof(BSKLINK_PAYLOAD_BATTERY_t);            //数据负载长度
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
    payload.posY_kp = FcGetPID(POS_Y).kP;
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
*函 数 名: BsklinkSendPidAck
*功能说明: 发送PID读写响应
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendPidAck(uint8_t* sendFlag)
{
    BSKLINK_MSG_t msg;
    BSKLINK_PAYLOAD_PID_ACK_t payload;
    uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;

    //数据负载填充
    payload.flag = pid_ack;

    /*********************************************消息帧赋值******************************************/
    msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头
    msg.head2 	 = BSKLINK_MSG_HEAD_2;
    msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
    msg.sysid 	 = BSKLINK_SYS_ID;							     //系统ID

    msg.msgid 	 = BSKLINK_MSG_ID_PID_ACK;                       //消息ID
    msg.length   = sizeof(BSKLINK_PAYLOAD_PID_ACK_t);            //数据负载长度
    memcpy(msg.payload, &payload, msg.length);                   //拷贝数据负载

    BsklinkMsgCalculateSum(&msg);                                //计算校验和
    /*************************************************************************************************/

    //消息帧格式化
    BsklinkMsgFormat(msg, msgToSend);
    //发送消息帧
    DataSend(msgToSend+1, msgToSend[0]);
}

/**********************************************************************************************************
*函 数 名: BsklinkSetPidAck
*功能说明: 设置PID读写响应值
*形    参: 响应值
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSetPidAck(uint8_t ack)
{
    pid_ack = ack;
    BsklinkSendEnable(BSKLINK_MSG_ID_PID_ACK);
}

/**********************************************************************************************************
*函 数 名: BsklinkSendSysError
*功能说明: 发送系统错误信息
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendSysError(uint8_t* sendFlag)
{
    BSKLINK_MSG_t msg;
    BSKLINK_PAYLOAD_SYS_ERROR_t payload;
    uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];
    uint8_t cnt = 0;

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;

    //数据负载填充
    for(uint8_t i=0; i<ERROR_NUM; i++)
    {
        if(cnt < 20)
        {
            if(FaultDetectGetErrorStatus(i))
                payload.error[cnt++] = i;
        }
    }

    /*********************************************消息帧赋值******************************************/
    msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头
    msg.head2 	 = BSKLINK_MSG_HEAD_2;
    msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
    msg.sysid 	 = BSKLINK_SYS_ID;							     //系统ID

    msg.msgid 	 = BSKLINK_MSG_ID_SYS_ERROR;                     //消息ID
    msg.length   = sizeof(BSKLINK_PAYLOAD_SYS_ERROR_t);          //数据负载长度
    memcpy(msg.payload, &payload, msg.length);                   //拷贝数据负载

    BsklinkMsgCalculateSum(&msg);                                //计算校验和
    /*************************************************************************************************/

    //消息帧格式化
    BsklinkMsgFormat(msg, msgToSend);
    //发送消息帧
    DataSend(msgToSend+1, msgToSend[0]);
}

/**********************************************************************************************************
*函 数 名: BsklinkSendSysWarning
*功能说明: 发送系统警告信息
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendSysWarning(uint8_t* sendFlag)
{
    BSKLINK_MSG_t msg;
    BSKLINK_PAYLOAD_SYS_WARNING_t payload;
    uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];
    uint8_t cnt = 0;

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;

    //数据负载填充
    for(uint8_t i=0; i<WARNNING_NUM; i++)
    {
        if(cnt < 20)
        {
            if(FaultDetectGetWarnningStatus(i))
                payload.warning[cnt++] = i;
        }
    }

    /*********************************************消息帧赋值******************************************/
    msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头
    msg.head2 	 = BSKLINK_MSG_HEAD_2;
    msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
    msg.sysid 	 = BSKLINK_SYS_ID;							     //系统ID

    msg.msgid 	 = BSKLINK_MSG_ID_SYS_WARNING;                   //消息ID
    msg.length   = sizeof(BSKLINK_PAYLOAD_SYS_WARNING_t);        //数据负载长度
    memcpy(msg.payload, &payload, msg.length);                   //拷贝数据负载

    BsklinkMsgCalculateSum(&msg);                                //计算校验和
    /*************************************************************************************************/

    //消息帧格式化
    BsklinkMsgFormat(msg, msgToSend);
    //发送消息帧
    DataSend(msgToSend+1, msgToSend[0]);
}

/**********************************************************************************************************
*函 数 名: BsklinkSendAttAnalyse
*功能说明: 发送姿态估计与控制分析数据
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendAttAnalyse(uint8_t* sendFlag)
{
    BSKLINK_MSG_t msg;
    BSKLINK_MSG_ID_ATT_ANALYSE_t payload;
    uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;

    //数据负载填充
    payload.gyro.x          = GyroGetData().x * 10;             //角速度 单位：0.1°/s
    payload.gyro.y          = GyroGetData().y * 10;
    payload.gyro.z          = GyroGetData().z * 10;
    payload.gyroLpf.x       = GyroLpfGetData().x * 10;          //角速度(滤波) 单位：0.1°/s
    payload.gyroLpf.y       = GyroLpfGetData().y * 10;
    payload.gyroLpf.z       = GyroLpfGetData().z * 10;
    payload.gyroTarget.x    = GetAttInnerCtlTarget().x * 10;    //目标角速度 单位：0.1°/s
    payload.gyroTarget.y    = GetAttInnerCtlTarget().y * 10;
    payload.gyroTarget.z    = GetAttInnerCtlTarget().z * 10;
    payload.angle.x         = GetCopterAngle().x * 10;          //姿态角 单位：0.1°
    payload.angle.y         = GetCopterAngle().y * 10;
    payload.angle.z         = GetCopterAngle().z * 10;
    payload.angleTarget.x   = GetAttOuterCtlTarget().x * 10;    //目标姿态角 单位：0.1°
    payload.angleTarget.y   = GetAttOuterCtlTarget().y * 10;
    payload.angleTarget.z   = GetAttOuterCtlTarget().z * 10;
    payload.angleMeasure.x  = GetAngleMeasure().x * 10;         //姿态角测量值 单位：0.1°
    payload.angleMeasure.y  = GetAngleMeasure().y * 10;
    payload.angleMeasure.z  = GetAngleMeasure().z * 10;
    payload.angleEstError.x = GetAngleEstError().x * 10;        //姿态角估计误差 单位：0.1°
    payload.angleEstError.y = GetAngleEstError().y * 10;
    payload.angleEstError.z = GetAngleEstError().z * 10;
    payload.angleCtlError.x = GetAttOuterCtlError().x * 10;     //姿态角控制误差 单位：0.1°
    payload.angleCtlError.y = GetAttOuterCtlError().y * 10;
    payload.angleCtlError.z = GetAttOuterCtlError().z * 10;

    /*********************************************消息帧赋值******************************************/
    msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头
    msg.head2 	 = BSKLINK_MSG_HEAD_2;
    msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
    msg.sysid 	 = BSKLINK_SYS_ID;							     //系统ID

    msg.msgid 	 = BSKLINK_MSG_ID_ATT_ANALYSE;                   //消息ID
    msg.length   = sizeof(BSKLINK_MSG_ID_ATT_ANALYSE_t);         //数据负载长度
    memcpy(msg.payload, &payload, msg.length);                   //拷贝数据负载

    BsklinkMsgCalculateSum(&msg);                                //计算校验和
    /*************************************************************************************************/

    //消息帧格式化
    BsklinkMsgFormat(msg, msgToSend);
    //发送消息帧
    DataSend(msgToSend+1, msgToSend[0]);
}

/**********************************************************************************************************
*函 数 名: BsklinkSendVelAnalyse
*功能说明: 发送速度估计与控制分析数据
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendVelAnalyse(uint8_t* sendFlag)
{
    BSKLINK_MSG_t msg;
    BSKLINK_MSG_ID_VEL_ANALYSE_t payload;
    uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;

    //数据负载填充
    payload.accel.x       = GetCopterAccel().x * GRAVITY_ACCEL * 100;     //加速度 单位：cm/s²
    payload.accel.y       = GetCopterAccel().y * GRAVITY_ACCEL * 100;
    payload.accel.z       = GetCopterAccel().z * GRAVITY_ACCEL * 100;
    payload.accelLpf.x    = GetCopterAccEfLpf().x * GRAVITY_ACCEL * 100;  //加速度(滤波) 单位：cm/s²
    payload.accelLpf.y    = GetCopterAccEfLpf().y * GRAVITY_ACCEL * 100;
    payload.accelLpf.z    = GetCopterAccEfLpf().z * GRAVITY_ACCEL * 100;
    payload.velocity.x    = GetCopterVelocity().x;                        //速度 单位：cm/s
    payload.velocity.y    = GetCopterVelocity().y;
    payload.velocity.z    = GetCopterVelocity().z;
    payload.velTarget.x   = GetPosInnerCtlTarget().x;                     //目标速度 单位：cm/s
    payload.velTarget.y   = GetPosInnerCtlTarget().y;
    payload.velTarget.z   = GetPosInnerCtlTarget().z;
    payload.gpsVel.x      = GetCopterVelMeasure()[GPS_VEL_X];             //GPS速度 单位：cm/s
    payload.gpsVel.y      = GetCopterVelMeasure()[GPS_VEL_Y];
    payload.gpsVel.z      = GetCopterVelMeasure()[GPS_VEL_Z];
    payload.opticalVelX   = GetAccelBias().x * 10;                        //光流速度 单位：cm/s
    payload.opticalVelY   = GetAccelBias().y * 10;
    payload.baroVel       = GetCopterVelMeasure()[BARO_VEL];
    payload.tofVel        = GetAccelBias().z * 10;
    payload.velEstError.x = 0;                                            //速度估计误差 单位：cm/s
    payload.velEstError.y = 0;
    payload.velEstError.z = 0;
    payload.velCtlError.x = GetPosInnerCtlError().x;                      //速度控制误差 单位：cm/s
    payload.velCtlError.y = GetPosInnerCtlError().y;
    payload.velCtlError.z = GetPosInnerCtlError().z;

    /*********************************************消息帧赋值******************************************/
    msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头
    msg.head2 	 = BSKLINK_MSG_HEAD_2;
    msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
    msg.sysid 	 = BSKLINK_SYS_ID;							     //系统ID

    msg.msgid 	 = BSKLINK_MSG_ID_VEL_ANALYSE;                   //消息ID
    msg.length   = sizeof(BSKLINK_MSG_ID_VEL_ANALYSE_t);         //数据负载长度
    memcpy(msg.payload, &payload, msg.length);                   //拷贝数据负载

    BsklinkMsgCalculateSum(&msg);                                //计算校验和
    /*************************************************************************************************/

    //消息帧格式化
    BsklinkMsgFormat(msg, msgToSend);
    //发送消息帧
    DataSend(msgToSend+1, msgToSend[0]);
}

/**********************************************************************************************************
*函 数 名: BsklinkSendPosAnalyse
*功能说明: 发送位置估计与控制分析数据
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendPosAnalyse(uint8_t* sendFlag)
{
    BSKLINK_MSG_t msg;
    BSKLINK_MSG_ID_POS_ANALYSE_t payload;
    uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;

    //数据负载填充
    payload.position.x    = GetCopterPosition().x;               //位置估计值 单位：cm
    payload.position.y    = GetCopterPosition().y;
    payload.position.z    = GetCopterPosition().z;
    payload.posTarget.x   =  GetPosOuterCtlTarget().x;           //位置目标 单位：cm
    payload.posTarget.y   = GetPosOuterCtlTarget().y;
    payload.posTarget.z   = GetPosOuterCtlTarget().z;
    payload.gpsPos.x      = GetCopterPosMeasure().x;             //GPS位置 单位：cm
    payload.gpsPos.y      = GetCopterPosMeasure().y;
    payload.gpsPos.z      = 0;
    payload.opticalPosX   = 0;                                   //光流位置 单位：cm
    payload.opticalPosY   = 0;
    payload.baroAlt       = GetCopterPosMeasure().z;             //气压高度 单位：cm
    payload.tofAlt        = 0;                                   //TOF高度 单位：cm
    payload.posEstError.x = 0;                                   //高度估计误差 单位：cm
    payload.posEstError.y = 0;
    payload.posEstError.z = 0;
    payload.posCtlError.x = GetPosOuterCtlError().x;             //高度控制误差 单位：cm
    payload.posCtlError.y = GetPosOuterCtlError().y;
    payload.posCtlError.z = GetPosOuterCtlError().z;

    /*********************************************消息帧赋值******************************************/
    msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头
    msg.head2 	 = BSKLINK_MSG_HEAD_2;
    msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
    msg.sysid 	 = BSKLINK_SYS_ID;							     //系统ID

    msg.msgid 	 = BSKLINK_MSG_ID_POS_ANALYSE;                   //消息ID
    msg.length   = sizeof(BSKLINK_MSG_ID_POS_ANALYSE_t);         //数据负载长度
    memcpy(msg.payload, &payload, msg.length);                   //拷贝数据负载

    BsklinkMsgCalculateSum(&msg);                                //计算校验和
    /*************************************************************************************************/

    //消息帧格式化
    BsklinkMsgFormat(msg, msgToSend);
    //发送消息帧
    DataSend(msgToSend+1, msgToSend[0]);
}

/**********************************************************************************************************
*函 数 名: BsklinkSendUserDefine
*功能说明: 发送自定义数据
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendUserDefine(uint8_t* sendFlag)
{
    BSKLINK_MSG_t msg;
    BSKLINK_MSG_ID_USER_DEFINE_t payload;
    uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;

    //数据负载填充
    payload.data1   = 1;
    payload.data2   = 2;
    payload.data3   = 3;
    payload.data4   = 4;
    payload.data5   = 5;
    payload.data6   = 6;
    payload.data7   = 7;
    payload.data8   = 8;
    payload.data9   = 9;
    payload.data10  = 10;
    payload.data11  = 11;
    payload.data12  = 12;

    /*********************************************消息帧赋值******************************************/
    msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头
    msg.head2 	 = BSKLINK_MSG_HEAD_2;
    msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
    msg.sysid 	 = BSKLINK_SYS_ID;							     //系统ID

    msg.msgid 	 = BSKLINK_MSG_ID_USER_DEFINE;                   //消息ID
    msg.length   = sizeof(BSKLINK_MSG_ID_USER_DEFINE_t);         //数据负载长度
    memcpy(msg.payload, &payload, msg.length);                   //拷贝数据负载

    BsklinkMsgCalculateSum(&msg);                                //计算校验和
    /*************************************************************************************************/

    //消息帧格式化
    BsklinkMsgFormat(msg, msgToSend);
    //发送消息帧
    DataSend(msgToSend+1, msgToSend[0]);
}

/**********************************************************************************************************
*函 数 名: BsklinkSendFreqSetup
*功能说明: 发送消息频率设置
*形    参: 发送标志指针
*返 回 值: 无
**********************************************************************************************************/
void BsklinkSendFreqSetup(uint8_t* sendFlag)
{
    BSKLINK_MSG_t msg;
    BSKLINK_MSG_ID_FREQ_SETUP_t payload;
    uint8_t msgToSend[BSKLINK_MAX_PAYLOAD_LENGTH+10];

    if(*sendFlag == DISABLE)
        return;
    else
        *sendFlag = DISABLE;

    //数据负载填充
    payload.flag = 1;

    /*********************************************消息帧赋值******************************************/
    msg.head1 	 = BSKLINK_MSG_HEAD_1;                           //帧头
    msg.head2 	 = BSKLINK_MSG_HEAD_2;
    msg.deviceid = BSKLINK_DEVICE_ID;                            //设备ID
    msg.sysid 	 = BSKLINK_SYS_ID;							     //系统ID

    msg.msgid 	 = BSKLINK_MSG_ID_FREQ_SETUP;                    //消息ID
    msg.length   = sizeof(BSKLINK_MSG_ID_FREQ_SETUP_t);          //数据负载长度
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


