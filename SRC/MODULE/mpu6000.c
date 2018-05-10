#include "mpu6000.h"
#include "drv_spi.h"

#define MPU_RA_XG_OFFS_TC       0x00    //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_YG_OFFS_TC       0x01    //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_ZG_OFFS_TC       0x02    //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_X_FINE_GAIN      0x03    //[7:0] X_FINE_GAIN
#define MPU_RA_Y_FINE_GAIN      0x04    //[7:0] Y_FINE_GAIN
#define MPU_RA_Z_FINE_GAIN      0x05    //[7:0] Z_FINE_GAIN
#define MPU_RA_XA_OFFS_H        0x06    //[15:0] XA_OFFS
#define MPU_RA_XA_OFFS_L_TC     0x07
#define MPU_RA_YA_OFFS_H        0x08    //[15:0] YA_OFFS
#define MPU_RA_YA_OFFS_L_TC     0x09
#define MPU_RA_ZA_OFFS_H        0x0A    //[15:0] ZA_OFFS
#define MPU_RA_ZA_OFFS_L_TC     0x0B
#define MPU_RA_PRODUCT_ID       0x0C    // Product ID Register
#define MPU_RA_XG_OFFS_USRH     0x13    //[15:0] XG_OFFS_USR
#define MPU_RA_XG_OFFS_USRL     0x14
#define MPU_RA_YG_OFFS_USRH     0x15    //[15:0] YG_OFFS_USR
#define MPU_RA_YG_OFFS_USRL     0x16
#define MPU_RA_ZG_OFFS_USRH     0x17    //[15:0] ZG_OFFS_USR
#define MPU_RA_ZG_OFFS_USRL     0x18
#define MPU_RA_SMPLRT_DIV       0x19
#define MPU_RA_CONFIG           0x1A
#define MPU_RA_GYRO_CONFIG      0x1B
#define MPU_RA_ACCEL_CONFIG     0x1C
#define MPU_RA_FF_THR           0x1D
#define MPU_RA_FF_DUR           0x1E
#define MPU_RA_MOT_THR          0x1F
#define MPU_RA_MOT_DUR          0x20
#define MPU_RA_ZRMOT_THR        0x21
#define MPU_RA_ZRMOT_DUR        0x22
#define MPU_RA_FIFO_EN          0x23
#define MPU_RA_I2C_MST_CTRL     0x24
#define MPU_RA_I2C_SLV0_ADDR    0x25
#define MPU_RA_I2C_SLV0_REG     0x26
#define MPU_RA_I2C_SLV0_CTRL    0x27
#define MPU_RA_I2C_SLV1_ADDR    0x28
#define MPU_RA_I2C_SLV1_REG     0x29
#define MPU_RA_I2C_SLV1_CTRL    0x2A
#define MPU_RA_I2C_SLV2_ADDR    0x2B
#define MPU_RA_I2C_SLV2_REG     0x2C
#define MPU_RA_I2C_SLV2_CTRL    0x2D
#define MPU_RA_I2C_SLV3_ADDR    0x2E
#define MPU_RA_I2C_SLV3_REG     0x2F
#define MPU_RA_I2C_SLV3_CTRL    0x30
#define MPU_RA_I2C_SLV4_ADDR    0x31
#define MPU_RA_I2C_SLV4_REG     0x32
#define MPU_RA_I2C_SLV4_DO      0x33
#define MPU_RA_I2C_SLV4_CTRL    0x34
#define MPU_RA_I2C_SLV4_DI      0x35
#define MPU_RA_I2C_MST_STATUS   0x36
#define MPU_RA_INT_PIN_CFG      0x37
#define MPU_RA_INT_ENABLE       0x38
#define MPU_RA_DMP_INT_STATUS   0x39
#define MPU_RA_INT_STATUS       0x3A
#define MPU_RA_ACCEL_XOUT_H     0x3B
#define MPU_RA_ACCEL_XOUT_L     0x3C
#define MPU_RA_ACCEL_YOUT_H     0x3D
#define MPU_RA_ACCEL_YOUT_L     0x3E
#define MPU_RA_ACCEL_ZOUT_H     0x3F
#define MPU_RA_ACCEL_ZOUT_L     0x40
#define MPU_RA_TEMP_OUT_H       0x41
#define MPU_RA_TEMP_OUT_L       0x42
#define MPU_RA_GYRO_XOUT_H      0x43
#define MPU_RA_GYRO_XOUT_L      0x44
#define MPU_RA_GYRO_YOUT_H      0x45
#define MPU_RA_GYRO_YOUT_L      0x46
#define MPU_RA_GYRO_ZOUT_H      0x47
#define MPU_RA_GYRO_ZOUT_L      0x48
#define MPU_RA_EXT_SENS_DATA_00 0x49
#define MPU_RA_MOT_DETECT_STATUS    0x61
#define MPU_RA_I2C_SLV0_DO      0x63
#define MPU_RA_I2C_SLV1_DO      0x64
#define MPU_RA_I2C_SLV2_DO      0x65
#define MPU_RA_I2C_SLV3_DO      0x66
#define MPU_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU_RA_SIGNAL_PATH_RESET    0x68
#define MPU_RA_MOT_DETECT_CTRL      0x69
#define MPU_RA_USER_CTRL        0x6A
#define MPU_RA_PWR_MGMT_1       0x6B
#define MPU_RA_PWR_MGMT_2       0x6C
#define MPU_RA_BANK_SEL         0x6D
#define MPU_RA_MEM_START_ADDR   0x6E
#define MPU_RA_MEM_R_W          0x6F
#define MPU_RA_DMP_CFG_1        0x70
#define MPU_RA_DMP_CFG_2        0x71
#define MPU_RA_FIFO_COUNTH      0x72
#define MPU_RA_FIFO_COUNTL      0x73
#define MPU_RA_FIFO_R_W         0x74
#define MPU_RA_WHO_AM_I         0x75

#define MPU_SMPLRT_DIV      0       // 8000Hz

#define BIT_GYRO                    3
#define BIT_ACC                     2
#define BIT_TEMP                    1

#define MPU_LPF_256HZ       0
#define MPU_LPF_188HZ       1
#define MPU_LPF_98HZ        2
#define MPU_LPF_42HZ        3
#define MPU_LPF_20HZ        4
#define MPU_LPF_10HZ        5
#define MPU_LPF_5HZ         6

#define MPU_A_2mg                ((float)0.00006103f)  // 0.00006250 g/LSB
#define MPU_A_4mg                ((float)0.00012207f)  // 0.00012500 g/LSB
#define MPU_A_8mg                ((float)0.00024414f)  // 0.00025000 g/LSB

#define MPU_G_s250dps            ((float)0.0076335f)  // 0.0087500 dps/LSB
#define MPU_G_s500dps            ((float)0.0152671f)  // 0.0175000 dps/LSB
#define MPU_G_s2000dps           ((float)0.0609756f)  // 0.0700000 dps/LSB


/**********************************************************************************************************
*函 数 名: MPU6000_Detect
*功能说明: 检测MPU6000是否存在
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
bool MPU6000_Detect(void)
{
	uint8_t who_am_i;
	
	Spi_GyroMultiRead(MPU_RA_WHO_AM_I, &who_am_i, 1);
	
	if(who_am_i == 0x68)
		return true;
	else 
		return false;
}

/**********************************************************************************************************
*函 数 名: MPU6000_Init
*功能说明: MPU6000寄存器配置初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MPU6000_Init(void)
{
	
	//检测传感器是否存在
	MPU6000_Detect();
	
	Spi_GyroSingleWrite(MPU_RA_PWR_MGMT_1, 0x80);
	SoftDelayMs(50);
	
	Spi_GyroSingleWrite(MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
	SoftDelayMs(50);
	
	Spi_GyroSingleWrite(MPU_RA_PWR_MGMT_1, 0x03);
	SoftDelayUs(5);
	
	Spi_GyroSingleWrite(MPU_RA_USER_CTRL, 0x10);
	SoftDelayUs(5);	
	
	Spi_GyroSingleWrite(MPU_RA_PWR_MGMT_2, 0x00);	
	SoftDelayUs(5);	
	
	//陀螺仪采样率0x00(1000Hz)   采样率 = 陀螺仪的输出率 / (1 + SMPLRT_DIV)
	Spi_GyroSingleWrite(MPU_RA_SMPLRT_DIV, (1000/1000 - 1));	
	SoftDelayUs(5);
	
	//i2c旁路模式
	// INT_PIN_CFG   -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS
	Spi_GyroSingleWrite(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);	
	SoftDelayUs(5);	
	
	//低通滤波频率
	Spi_GyroSingleWrite(MPU_RA_CONFIG, MPU_LPF_42HZ);		
	SoftDelayUs(5);		
	
	//陀螺仪自检及测量范围，典型值0x18(不自检，2000deg/s)
	Spi_GyroSingleWrite(MPU_RA_GYRO_CONFIG, 0x18);		
	SoftDelayUs(5);		
	
	//加速度自检、测量范围(不自检，+-8G)			
	Spi_GyroSingleWrite(MPU_RA_ACCEL_CONFIG, 2 << 3);		
	
	SoftDelayMs(5);		
}

/**********************************************************************************************************
*函 数 名: MPU6000_ReadAcc
*功能说明: MPU6000读取加速度传感器
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void MPU6000_ReadAcc(Vector3i_t* acc)
{
	uint8_t buffer[6];

	Spi_GyroMultiRead(MPU_RA_ACCEL_XOUT_H, buffer, 6);	
	acc->x = ((((int16_t)buffer[0]) << 8) | buffer[1]); 
	acc->y = ((((int16_t)buffer[2]) << 8) | buffer[3]);  
	acc->z = ((((int16_t)buffer[4]) << 8) | buffer[5]); 
	
	SoftDelayUs(1);
}

/**********************************************************************************************************
*函 数 名: MPU6000_ReadGyro
*功能说明: MPU6000读取陀螺仪传感器
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void MPU6000_ReadGyro(Vector3i_t* gyro)
{
	uint8_t buffer[6];
	
	Spi_GyroMultiRead(MPU_RA_GYRO_XOUT_H, buffer, 6);	
	gyro->x = ((((int16_t)buffer[0]) << 8) | buffer[1]);	
	gyro->y = ((((int16_t)buffer[2]) << 8) | buffer[3]);
	gyro->z = ((((int16_t)buffer[4]) << 8) | buffer[5]);
	
	SoftDelayUs(1);
}

/**********************************************************************************************************
*函 数 名: MPU6000_ReadTemp
*功能说明: MPU6000读取温度传感器
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void MPU6000_ReadTemp(float* temp)
{
	uint8_t buffer[2];
	static int16_t temperature_temp;
	
	Spi_GyroMultiRead(MPU_RA_TEMP_OUT_H, buffer, 2);	
	temperature_temp = ((((int16_t)buffer[0]) << 8) | buffer[1]);		
	*temp = 36.53f + (float)temperature_temp / 340;
	
	SoftDelayUs(1);
}

/**********************************************************************************************************
*函 数 名: MPU6000_GyroNormalize
*功能说明: 陀螺仪数据单位化
*形    参: 原始数据
*返 回 值: 单位化后的数据
**********************************************************************************************************/
Vector3f_t MPU6000_GyroNormalize(Vector3i_t raw)
{
    Vector3f_t temp;
    
    temp.x = raw.x * MPU_G_s2000dps;
    temp.y = raw.y * MPU_G_s2000dps;
    temp.z = raw.z * MPU_G_s2000dps;
    
    return temp;
}

/**********************************************************************************************************
*函 数 名: MPU6000_AccNormalize
*功能说明: 加速度数据单位化
*形    参: 原始数据
*返 回 值: 单位化后的数据
**********************************************************************************************************/
Vector3f_t MPU6000_AccNormalize(Vector3i_t raw)
{
    Vector3f_t temp;
    
    temp.x = raw.x * MPU_A_8mg;
    temp.y = raw.y * MPU_A_8mg;
    temp.z = raw.z * MPU_A_8mg;
    
    return temp;    
}

