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
    
    uint8_t recvStatus;                            //解析状态
    uint8_t payloadRecvCnt;                        //负载接收计数
}BSKLINK_MSG_t;

/********************************************消息类型定义**********************************************/
enum
{
    BSKLINK_MSG_ID_FLIGHT_DATA      = 0x01,     //基本飞行数据
    BSKLINK_MSG_ID_FLIGHT_STATUS    = 0x02,     //飞控状态信息
    BSKLINK_MSG_ID_SENSOR           = 0x03,     //传感器数据
    BSKLINK_MSG_ID_NAV_ERROR        = 0x05,     //导航相关误差数据
    BSKLINK_MSG_ID_CONTROL_ERROR    = 0x06,     //控制相关误差数据
    BSKLINK_MSG_ID_RC_DATA          = 0x08,     //遥控通道数据
    BSKLINK_MSG_ID_BATTERY          = 0x0A,     //电池信息
    BSKLINK_MSG_ID_PID_ATT          = 0x10,     //姿态PID参数
    BSKLINK_MSG_ID_PID_POS          = 0x11,     //位置PID参数
    BSKLINK_MSG_ID_GPS              = 0x20,     //GPS数据
};

/******************************************数据负载结构体定义******************************************/
//结构体按1字节方式对齐
#pragma pack (1) 

//基本飞行数据
typedef struct
{
    Vector3i_t angle;       //姿态角 单位：0.1°
    Vector3i_t accel;       //地理系运动加速度 单位：cm/s²
    Vector3i_t accelLpf;    //低通滤波后的地理系运动加速度 单位：cm/s²
    Vector3i_t velocity;    //速度估计值  单位：cm/s
    Vector3i_t velMeasure;  //速度观测值  单位：cm/s 
    Vector3l_t position;    //位置估计值  单位：cm
    Vector3l_t posMeasure;  //位置观测值  单位：cm
}BSKLINK_PAYLOAD_FLIGHT_DATA_t;

//飞控状态信息
typedef struct
{
	uint8_t flightMode;		//飞行模式
	uint8_t initStatus;		//初始化状态	
	uint8_t armedStatus;	//锁定状态
	uint8_t flightStatus;   //飞行状态
	uint8_t placeStatus;	//放置状态
	uint8_t altCtlStatus;	//高度控制状态
	uint8_t posCtlStatus;   //位置控制状态
}BSKLINK_PAYLOAD_FLIGHT_STATUS_t;

//传感器数据
typedef struct
{
	Vector3i_t gyro;        //角速度 单位：0.1°/s
    Vector3i_t gyroLpf;     //低通滤波后的角速度 单位：0.1°/s
	Vector3i_t acc;         //加速度 单位：0.001g
    Vector3i_t accLpf;      //低通滤波后的加速度 单位：0.001g
    int16_t    gyroTemp;    //陀螺仪温度 单位：0.01°
	Vector3i_t mag;         //磁场强度 单位：0.001gauss
    int32_t    baroAlt;     //气压高度 单位：cm 
    int16_t    baroTemp;    //气压计温度 单位：0.01°
}BSKLINK_PAYLOAD_SENSOR_t;

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

//电池信息
typedef struct
{
    int16_t voltage;        //电池电压 单位：0.01V
    int16_t current;        //电池电流 单位：0.01A
    int8_t  capacityPer;    //电池电量百分比（0xFF表示无电量数据）
    int16_t capacity;       //电池设计容量 单位：mah
    int16_t capRemain;      //电池当前剩余容量 单位：mah
    int16_t temperature;    //电池温度 单位：0.01°
    int8_t  cellNum;        //电芯节数
    int16_t cellVolt[6];    //电芯电压（最多6节） 单位：0.01V
}BSKLINK_PAYLOAD_BATTERY_t;

//姿态PID参数
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
    float rollAngle_kp;     //横滚角度环P
    float pitchAngle_kp;    //俯仰角度环P
    float yawAngle_kp;      //偏航角度环P
}BSKLINK_PAYLOAD_PID_ATT_t;

//位置PID参数
typedef struct
{
	float velX_kp;          //X轴速度环P
	float velX_ki;          //X轴速度环I
	float velX_kd;          //X轴速度环D
    float velY_kp;          //Y轴速度环P
    float velY_ki;          //Y轴速度环I
    float velY_kd;          //Y轴速度环D
    float velZ_kp;          //Z轴速度环P
    float velZ_ki;          //Z轴速度环I
    float velZ_kd;          //Z轴速度环D
    float posX_kp;          //X轴位置环P
    float posY_kp;          //Y轴位置环P
    float posZ_kp;          //Z轴位置环P
}BSKLINK_PAYLOAD_PID_POS_t;

//GPS数据
typedef struct
{
	float   time;           //当地时间 单位：秒
    int8_t  numSV;          //卫星数量
    int8_t  fixStatus;      //定位状态
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


