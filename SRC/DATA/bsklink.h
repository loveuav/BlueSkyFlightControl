#ifndef _BSKLINK_H_
#define _BSKLINK_H_

#include "mathTool.h"

#define BSKLINK_MSG_HEAD_1    0xDE
#define BSKLINK_MSG_HEAD_2    0xED
#define BSKLINK_DEVICE_ID     0x02

#define BSKLINK_MAX_PAYLOAD_LENGTH 100

/********************************************消息帧结构定义********************************************/
typedef struct
{
    uint8_t head1;                                 //帧头 
    uint8_t head2;                                 //帧头
	uint8_t deviceid;                              //设备ID
    uint8_t msgid;                                 //消息ID
    uint8_t length;                                //数据负载长度
	uint8_t payload[BSKLINK_MAX_PAYLOAD_LENGTH];   //数据负载
    uint8_t checksum;                              //校验和
}BSKLINK_MSG_t;

/********************************************消息类型定义**********************************************/
enum
{
    BSKLINK_MSG_ID_FLIGHT_DATA      = 0x01,     //基本飞行数据
    BSKLINK_MSG_ID_IMU_SENSOR       = 0x03,     //IMU传感器数据
    BSKLINK_MSG_ID_RC_DATA          = 0x08,     //遥控通道数据
    BSKLINK_MSG_ID_PID_ATT_INNER    = 0x10,     //姿态内环PID参数
    BSKLINK_MSG_ID_PID_ATT_OUTER    = 0x11,     //姿态外环PID参数
    BSKLINK_MSG_ID_PID_POS_INNER    = 0x12,     //位置内环PID参数
    BSKLINK_MSG_ID_PID_POS_OUTER    = 0x13,     //位置外环PID参数
    BSKLINK_MSG_ID_GPS              = 0x20,     //GPS数据
};

/******************************************数据负载结构体定义******************************************/
//结构体按1字节方式对齐
#pragma pack (1) 

//基本飞行数据
typedef struct
{
	int16_t angleRoll;      //横滚角 单位：0.1°
	int16_t anglePitch;     //俯仰角 单位：0.1°
	int16_t angleYaw;       //偏航角 单位：0.1°
    int16_t accelX;         //地理系加速度X轴 单位：cm/s²
    int16_t accelY;         //地理系加速度Y轴 单位：cm/s²
    int16_t accelZ;         //地理系加速度Z轴 单位：cm/s²
    int16_t velocityX;      //速度X轴  单位：cm/s
    int16_t velocityY;      //速度Y轴  单位：cm/s
    int16_t velocityZ;      //速度Z轴  单位：cm/s
    int32_t positionX;      //位置X轴  单位：cm
    int32_t positionY;      //位置Y轴  单位：cm
    int32_t positionZ;      //位置Z轴  单位：cm
}BSKLINK_PAYLOAD_FLIGHT_DATA_t;

//IMU传感器数据
typedef struct
{
	int16_t gyroX;          //陀螺仪X轴 单位：0.1°/s
	int16_t gyroY;          //陀螺仪Y轴 单位：0.1°/s
	int16_t gyroZ;          //陀螺仪Z轴 单位：0.1°/s
    int16_t gyroLpfX;       //低通滤波后的陀螺仪X轴 单位：0.1°/s
    int16_t gyroLpfY;       //低通滤波后的陀螺仪Y轴 单位：0.1°/s
    int16_t gyroLpfZ;       //低通滤波后的陀螺仪Z轴 单位：0.1°/s
	int16_t accX;           //加速度X轴 单位：0.001g
	int16_t accY;           //加速度Y轴 单位：0.001g
	int16_t accZ;           //加速度Z轴 单位：0.001g
    int16_t accLpfX;        //通滤波后的加速度X轴 单位：0.001g
    int16_t accLpfY;        //通滤波后的加速度Y轴 单位：0.001g
    int16_t accLpfZ;        //通滤波后的加速度Z轴 单位：0.001g
}BSKLINK_PAYLOAD_IMU_SENSOR_t;

//遥控通道数据
typedef struct
{
	int16_t roll;           //横滚
	int16_t pitch;          //俯仰
	int16_t yaw;            //偏航
    int16_t throttle;       //油门
    int16_t aux1;           //辅助通道1
    int16_t aux2;           //辅助通道2
	int16_t aux3;           //辅助通道3
	int16_t aux4;           //辅助通道4
	int16_t aux5;           //辅助通道5
    int16_t aux6;           //辅助通道6
    int16_t aux7;           //辅助通道7
    int16_t aux8;           //辅助通道8
}BSKLINK_PAYLOAD_RC_DATA_t;

//角速率内环PID参数
typedef struct
{
	float roll_kp;          //横滚角速度环P
	float roll_ki;          //横滚角速度环I
	float roll_kd;          //横滚角速度环D
    float pitch_kp;         //俯仰角速度环P
    float pitch_ki;         //俯仰角速度环I
    float pitch_kd;         //俯仰角速度环D
    float yaw_kp;           //偏航角速度环P
    float yaw_ki;           //偏航角速度环I
    float yaw_kd;           //偏航角速度环D
}BSKLINK_PAYLOAD_PID_ATT_INNER_t;

typedef struct
{
	float roll_kp;          //横滚角度环P
    float pitch_kp;         //俯仰角度环P
    float yaw_kp;           //偏航角度环P
}BSKLINK_PAYLOAD_PID_ATT_OUTER_t;

typedef struct{
	float velX_kp;          //X轴速度环P
	float velX_ki;          //X轴速度环I
	float velX_kd;          //X轴速度环D
    float velY_kp;          //Y轴速度环P
    float velY_ki;          //Y轴速度环I
    float velY_kd;          //Y轴速度环D
    float velZ_kp;          //Z轴速度环P
    float velZ_ki;          //Z轴速度环I
    float velZ_kd;          //Z轴速度环D
}BSKLINK_PAYLOAD_PID_POS_INNER_t;

typedef struct
{
	float posX_kp;          //X轴位置环P
    float posY_kp;          //Y轴位置环P
    float posZ_kp;          //Z轴位置环P
}BSKLINK_PAYLOAD_PID_POS_OUTER_t;

typedef struct
{
	float   time;           //当地时间 单位：秒
    int8_t  numSV;          //卫星数量
    int16_t hAcc;           //水平定位精度 单位：m
    int16_t vAcc;           //垂直定位精度 单位：m
    float   latitude;       //纬度
    float   longitude;      //经度
    float   altitude;       //高度 单位：m
    int16_t velN;           //北向速度 单位：cm/s
    int16_t velE;           //东向速度 单位：cm/s
    int16_t velD;           //天向速度 单位：cm/s
}BSKLINK_PAYLOAD_GPS_t;

#pragma pack () 

void BsklinkMsgCalculateSum(BSKLINK_MSG_t* msg);
void BsklinkMsgFormat(BSKLINK_MSG_t msg, uint8_t* msgTemp);
bool BsklinkDecode(BSKLINK_MSG_t* msg, uint8_t data);

#endif


