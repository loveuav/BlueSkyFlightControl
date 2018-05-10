#include "qmc5883.h"
#include "drv_i2c_soft.h"

#define QMC5883L_Addr   0x1A     
#define QMC5883L_HX_L   0x00
#define QMC5883L_HX_H   0x01
#define QMC5883L_HY_L   0x02
#define QMC5883L_HY_H   0x03
#define QMC5883L_HZ_L   0x04 
#define QMC5883L_HZ_H   0x05
#define QMC5883L_CTR1   0x09
#define QMC5883L_SRPERIOD 0x0B

#define QMC_2G ((float)0.00008333f)  // 12000 LSB/G
#define QMC_8G ((float)0.00033333f)  // 3000  LSB/G

#define QMC5883_MAG_TO_GAUSS 0.0008333f

Vector3i_t magRaw;

static void QMC5883_WriteReg(u8 REG_Address,u8 REG_data)
{
    Soft_I2c_Single_Write(1, QMC5883L_Addr, REG_Address, REG_data);
}

static uint8_t QMC5883_ReadReg(u8 REG_Address)
{
    return Soft_I2C_Single_Read(1, QMC5883L_Addr, REG_Address);
}

bool QMC5883_Detect(void)
{
    QMC5883_WriteReg(QMC5883L_SRPERIOD,0x01);
	QMC5883_WriteReg(QMC5883L_CTR1,0x0D);  //2Guass 200Hz	
	SoftDelayMs(50);
	
	if(QMC5883_ReadReg(QMC5883L_CTR1) == 0x0D)
		return true;
	else
		return false;
}

void QMC5883_Init(void)
{
	QMC5883_WriteReg(QMC5883L_SRPERIOD, 0x01);
	SoftDelayMs(5);  
	QMC5883_WriteReg(QMC5883L_CTR1, 0x0D);  //2Guass 200Hz	
}

void QMC5883_Update(void)
{
	uint8_t buffer[6];

	buffer[1] = QMC5883_ReadReg(QMC5883L_HX_L);	
	buffer[0] = QMC5883_ReadReg(QMC5883L_HX_H);
	magRaw.x = (int16_t)buffer[0] << 8 | buffer[1];
	magRaw.x *= 0.1f;
	
	buffer[3] = QMC5883_ReadReg(QMC5883L_HY_L);
	buffer[2] = QMC5883_ReadReg(QMC5883L_HY_H);
	magRaw.y = (int16_t)buffer[2] << 8 | buffer[3];
	magRaw.y *= 0.1f;

	buffer[5] = QMC5883_ReadReg(QMC5883L_HZ_L);	
	buffer[4] = QMC5883_ReadReg(QMC5883L_HZ_H); 
	magRaw.z = (int16_t)buffer[4] << 8 | buffer[5];
	magRaw.z *= 0.1f;
}

void QMC5883_Read(Vector3i_t* mag)
{
	mag->x = magRaw.x;
	mag->y = magRaw.y;
	mag->z = magRaw.z;
}

Vector3f_t QMC5883_MagNormalize(Vector3i_t raw)
{
    Vector3f_t temp;
    
    temp.x = raw.x * QMC5883_MAG_TO_GAUSS;
    temp.y = raw.y * QMC5883_MAG_TO_GAUSS;
    temp.z = raw.z * QMC5883_MAG_TO_GAUSS;
    
    return temp;    
}



