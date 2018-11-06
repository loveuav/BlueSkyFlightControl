/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     qmc5883.c
 * @说明     QMC5883地磁传感器驱动
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05
**********************************************************************************************************/
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

#define QMC5883_MAG_TO_GAUSS QMC_2G

static Vector3i_t magRaw;

/**********************************************************************************************************
*函 数 名: QMC5883_WriteReg
*功能说明: 往QMC5883的寄存器写入一个字节的数据
*形    参: 寄存器地址 写入数据
*返 回 值: 无
**********************************************************************************************************/
static void QMC5883_WriteReg(uint8_t REG_Address, uint8_t REG_data)
{
    Soft_I2c_Single_Write(MAG_I2C, QMC5883L_Addr, REG_Address, REG_data);
}

/**********************************************************************************************************
*函 数 名: QMC5883_ReadReg
*功能说明: 读取QMC5883寄存器的数据
*形    参: 寄存器地址
*返 回 值: 寄存器数据
**********************************************************************************************************/
static uint8_t QMC5883_ReadReg(uint8_t REG_Address)
{
    return Soft_I2C_Single_Read(MAG_I2C, QMC5883L_Addr, REG_Address);
}

/**********************************************************************************************************
*函 数 名: QMC5883_MultiRead
*功能说明: 连续读取QMC5883寄存器的数据
*形    参: 寄存器地址 读出缓冲区 长度
*返 回 值: 成功标志位
**********************************************************************************************************/
static bool QMC5883_MultiRead(uint8_t REG_Address, uint8_t* buffer, uint8_t length)
{
    return Soft_I2C_Multi_Read(MAG_I2C, QMC5883L_Addr, REG_Address, buffer, length);
}

/**********************************************************************************************************
*函 数 名: QMC5883_Detect
*功能说明: 检测QMC5883是否存在
*形    参: 无
*返 回 值: 存在状态
**********************************************************************************************************/
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

/**********************************************************************************************************
*函 数 名: QMC5883_Init
*功能说明: QMC5883寄存器配置初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void QMC5883_Init(void)
{
    QMC5883_WriteReg(QMC5883L_SRPERIOD, 0x01);
    SoftDelayMs(5);
    QMC5883_WriteReg(QMC5883L_CTR1, 0x0D);  //2Guass 200Hz
}

/**********************************************************************************************************
*函 数 名: QMC5883_Update
*功能说明: QMC5883数据更新
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void QMC5883_Update(void)
{
    uint8_t buffer[6];

    if(!QMC5883_MultiRead(QMC5883L_HX_L, buffer, 6))
        return;
    
    magRaw.x = (int16_t)buffer[1] << 8 | buffer[0];
    magRaw.y = (int16_t)buffer[3] << 8 | buffer[2];
    magRaw.z = (int16_t)buffer[5] << 8 | buffer[4];   
    
    //统一传感器坐标系（并非定义安装方向）
    magRaw.x = magRaw.x;
    magRaw.y = -magRaw.y;
    magRaw.z = magRaw.z;
}

/**********************************************************************************************************
*函 数 名: QMC5883_Read
*功能说明: 读取地磁传感器数据,并转换为标准单位
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void QMC5883_Read(Vector3f_t* mag)
{
    mag->x = magRaw.x * QMC5883_MAG_TO_GAUSS;
    mag->y = magRaw.y * QMC5883_MAG_TO_GAUSS;
    mag->z = magRaw.z * QMC5883_MAG_TO_GAUSS;
}


