#ifndef _BSKLINK_H_
#define _BSKLINK_H_

#include "mathTool.h"

#define BSKLINK_MSG_HEAD_1    0xDE
#define BSKLINK_MSG_HEAD_2    0xED
#define BSKLINK_DEVICE_ID     0x02
#define BSKLINK_SYS_ID	      0x00

#define BSKLINK_MAX_PAYLOAD_LENGTH 100

/********************************************消息帧结构定义********************************************/
typedef struct
{
    uint8_t head1;                                 //帧头
    uint8_t head2;                                 //帧头
    uint8_t deviceid;                              //设备ID
    uint8_t sysid;								   //系统ID
    uint8_t msgid;                                 //消息ID
    uint8_t length;                                //数据负载长度
    uint8_t payload[BSKLINK_MAX_PAYLOAD_LENGTH];   //数据负载
    uint8_t checksum;                              //校验和

    uint8_t recvStatus;                            //解析状态
    uint8_t payloadRecvCnt;                        //负载接收计数
} BSKLINK_MSG_t;

/********************************************消息类型定义**********************************************/
enum
{
    BSKLINK_MSG_ID_FLIGHT_DATA      = 0x01,     //基本飞行数据
    BSKLINK_MSG_ID_FLIGHT_STATUS    = 0x02,     //飞控状态信息
    BSKLINK_MSG_ID_SENSOR           = 0x03,     //传感器数据
    BSKLINK_MSG_ID_SENSOR_CALI_DATA = 0x04,		//传感器校准数据
    BSKLINK_MSG_ID_SENSOR_CALI_CMD  = 0x05,		//传感器校准命令
    BSKLINK_MSG_ID_RC_DATA          = 0x08,     //遥控通道数据
    BSKLINK_MSG_ID_MOTOR            = 0x09,     //电机输出
    BSKLINK_MSG_ID_BATTERY          = 0x0A,     //电池信息
    BSKLINK_MSG_ID_PID_ATT          = 0x10,     //姿态PID参数
    BSKLINK_MSG_ID_PID_POS          = 0x11,     //位置PID参数
    BSKLINK_MSG_ID_PID_ACK          = 0x12,     //PID读写响应
    BSKLINK_MSG_ID_SETUP            = 0x15,     //飞控设置
    BSKLINK_MSG_ID_GPS              = 0x20,     //GPS数据
    BSKLINK_MSG_ID_SYS_ERROR 		= 0x25,		//系统错误信息
    BSKLINK_MSG_ID_SYS_WARNING 		= 0x26,		//系统警告信息
    BSKLINK_MSG_ID_ATT_ANALYSE      = 0x30,     //姿态估计与控制数据
    BSKLINK_MSG_ID_VEL_ANALYSE      = 0x31,     //速度估计与控制数据
    BSKLINK_MSG_ID_POS_ANALYSE      = 0x32,     //位置估计与控制数据
    BSKLINK_MSG_ID_USER_DEFINE      = 0x33,     //自定义数据
    BSKLINK_MSG_ID_FREQ_SETUP       = 0xF0,     //消息发送频率设置
    BSKLINK_MSG_ID_HEARTBEAT		= 0xFE		//心跳包
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
} BSKLINK_PAYLOAD_FLIGHT_DATA_t;

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
} BSKLINK_PAYLOAD_FLIGHT_STATUS_t;

//传感器数据
typedef struct
{
    Vector3i_t gyro;        //角速度 单位：0.1°/s
    Vector3i_t gyroLpf;     //低通滤波后的角速度 单位：0.1°/s
    int16_t    gyroTemp;    //陀螺仪温度 单位：0.01°
    int8_t	   gyro_offset; //角速度零偏 单位：%
    Vector3i_t acc;         //加速度 单位：0.001g
    Vector3i_t accLpf;      //低通滤波后的加速度 单位：0.001g
    int8_t	   acc_offset;  //加速度零偏 单位：%
    Vector3i_t mag;         //磁场强度 单位：0.001gauss
    int8_t	   mag_offset;  //罗盘零偏  单位：%
    int32_t    baroAlt;     //气压高度 单位：cm
    int16_t    baroTemp;    //气压计温度 单位：0.01°
} BSKLINK_PAYLOAD_SENSOR_t;

//传感器校准数据
typedef struct
{
    Vector3f_t gyro_offset; //角速度零偏误差 单位：1°/s
    Vector3f_t gyro_scale;  //角速度刻度误差
    Vector3f_t acc_offset;  //加速度零偏误差 单位：g
    Vector3f_t acc_scale;   //加速度刻度误差
    Vector3f_t mag_offset;  //磁力计零偏误差 单位：gauss
    Vector3f_t mag_scale;   //磁力计刻度误差
    Vector3f_t angle;		//IMU安装误差 单位：°
} BSKLINK_PAYLOAD_SENSOR_CALI_DATA_t;

//传感器校准命令
typedef struct
{
    uint8_t type;			//传感器类型
    uint8_t caliFlag;		//校准标志位
    uint8_t successFlag;    //成功标志位
    uint8_t step;			//校准步骤
} BSKLINK_PAYLOAD_SENSOR_CALI_CMD_t;

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
} BSKLINK_PAYLOAD_RC_DATA_t;

//电机输出
typedef struct
{
    int8_t  num;            //电机数量
    int16_t motorValue1;    //1号电机输出值
    int16_t motorValue2;    //2号电机输出值
    int16_t motorValue3;    //3号电机输出值
    int16_t motorValue4;    //4号电机输出值
    int16_t motorValue5;    //5号电机输出值
    int16_t motorValue6;    //6号电机输出值
    int16_t motorValue7;    //7号电机输出值
    int16_t motorValue8;    //8号电机输出值
} BSKLINK_PAYLOAD_MOTOR_t;

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
} BSKLINK_PAYLOAD_BATTERY_t;

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
} BSKLINK_PAYLOAD_PID_ATT_t;

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
} BSKLINK_PAYLOAD_PID_POS_t;

//PID读写响应
typedef struct
{
    uint8_t flag;           //读写标志
    uint8_t res;            //预留
} BSKLINK_PAYLOAD_PID_ACK_t;

enum
{
    PID_WRITE_FAILED = 1,	//PID写入失败
    PID_WRITE_SUCCESS,		//PID写入成功
};

//GPS数据
typedef struct
{
    float   time;           //当地时间 单位：秒
    int8_t  numSV;          //卫星数量
    int8_t  fixStatus;      //定位状态
    int16_t hAcc;           //水平定位精度 单位：cm
    int16_t vAcc;           //垂直定位精度 单位：cm
    float   latitude;       //纬度
    float   longitude;      //经度
    float   altitude;       //高度 单位：m
    int16_t velN;           //北向速度 单位：cm/s
    int16_t velE;           //东向速度 单位：cm/s
    int16_t velD;           //天向速度 单位：cm/s
} BSKLINK_PAYLOAD_GPS_t;

//系统错误信息
typedef struct
{
    uint8_t error[20];
} BSKLINK_PAYLOAD_SYS_ERROR_t;

//系统警告信息
typedef struct
{
    uint8_t warning[20];
} BSKLINK_PAYLOAD_SYS_WARNING_t;

//姿态估计与控制数据
typedef struct
{
    Vector3i_t gyro;            //角速度 单位：0.1°/s
    Vector3i_t gyroLpf;         //角速度(滤波) 单位：0.1°/s
    Vector3i_t gyroTarget;      //目标角速度 单位：0.1°/s
    Vector3i_t angle;           //姿态角 单位：0.1°
    Vector3i_t angleTarget;     //目标姿态角 单位：0.1°
    Vector3i_t angleMeasure;    //姿态角测量值 单位：0.1°
    Vector3i_t angleEstError;   //姿态角估计误差 单位：0.1°
    Vector3i_t angleCtlError;   //姿态角控制误差 单位：0.1°
} BSKLINK_MSG_ID_ATT_ANALYSE_t;

//速度估计与控制数据
typedef struct
{
    Vector3i_t accel;           //地理系运动加速度 单位：cm/s²
    Vector3i_t accelLpf;        //地理系运动加速度（滤波） 单位：cm/s²
    Vector3i_t velocity;        //速度估计值  单位：cm/s
    Vector3i_t velTarget;       //速度目标  单位：cm/s
    Vector3i_t gpsVel;          //GPS速度  单位：cm/s
    int16_t    opticalVelX;     //光流速度x轴  单位：cm/s
    int16_t    opticalVelY;     //光流速度y轴  单位：cm/s
    int16_t    baroVel;         //气压速度  单位：cm/s
    int16_t    tofVel;          //超声波/TOF速度  单位：cm/s
    Vector3i_t velEstError;     //速度估计误差 单位：cm/s
    Vector3i_t velCtlError;     //速度控制误差 单位：cm/s
} BSKLINK_MSG_ID_VEL_ANALYSE_t;

//位置估计与控制数据
typedef struct
{
    Vector3l_t position;        //位置估计值  单位：cm
    Vector3l_t posTarget;       //位置目标  单位：cm
    Vector3l_t gpsPos;          //GPS位置  单位：cm
    int32_t    opticalPosX;     //光流位置x轴  单位：cm
    int32_t    opticalPosY;     //光流位置y轴  单位：cm
    int32_t    baroAlt;         //气压高度  单位：cm
    int32_t    tofAlt;          //超声波/TOF高度  单位：cm
    Vector3i_t posEstError;     //位置估计误差 单位：cm
    Vector3i_t posCtlError;     //位置控制误差 单位：cm
} BSKLINK_MSG_ID_POS_ANALYSE_t;

//自定义数据
typedef struct
{
    int16_t data1;
    int16_t data2;
    int16_t data3;
    int16_t data4;
    int16_t data5;
    int16_t data6;
    int16_t data7;
    int16_t data8;
    int16_t data9;
    int16_t data10;
    int16_t data11;
    int16_t data12;
} BSKLINK_MSG_ID_USER_DEFINE_t;

//消息发送频率设置
typedef struct
{
    uint8_t flag;                   //设置返回标志位

    uint8_t flightData;
    uint8_t flightStatus;
    uint8_t sensor;
    uint8_t sensorCaliData;
    uint8_t rcData;
    uint8_t motor;
    uint8_t battery;
    uint8_t gps;
    uint8_t attAnalyse;
    uint8_t velAnalyse;
    uint8_t posAnalyse;
    uint8_t userDefine;
} BSKLINK_MSG_ID_FREQ_SETUP_t;

//心跳包
typedef struct
{
    uint8_t  type;			//硬件类型
    uint8_t  version_high;	//飞控版本号高位
    uint8_t  version_mid;	//飞控版本号中位
    uint8_t  version_low;	//飞控版本号低位
    int32_t  time;          //系统时间 单位：毫秒
    uint16_t freq;			//最大发送频率
} BSKLINK_PAYLOAD_HEARTBEAT_t;

#pragma pack ()

void BsklinkMsgCalculateSum(BSKLINK_MSG_t* msg);
void BsklinkMsgFormat(BSKLINK_MSG_t msg, uint8_t* msgTemp);
bool BsklinkParseChar(BSKLINK_MSG_t* msg, uint8_t data);

#endif


