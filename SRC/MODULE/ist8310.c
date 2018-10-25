/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     IST8310.c
 * @说明     IST8310地磁传感器驱动
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.08
**********************************************************************************************************/
#include "ist8310.h"
#include "drv_i2c_soft.h"

/* IST8310的IIC地址选择
  CAD1  |  CAD0  |  地址 | 模拟IIC地址
------------------------------
  GND   |   GND  |  0CH  | 18H
  GND   |   VDD  |  0DH  | 1AH
  VDD   |   GND  |  0EH  | 1CH
  VDD   |   VDD  |  0FH  | 1EH
  如果CAD1和CAD0都悬空, 地址为0EH
 */
#define IST8310_ADDRESS                 0x1E

#define IST8310_REG_HX_L                0x03
#define IST8310_REG_HX_H                0x04
#define IST8310_REG_HY_L                0x05
#define IST8310_REG_HY_H                0x06
#define IST8310_REG_HZ_L                0x07
#define IST8310_REG_HZ_H                0x08
#define IST8310_REG_WHOAMI              0x00
#define IST8310_REG_CNTRL1              0x0A
#define IST8310_REG_CNTRL2              0x0B
#define IST8310_REG_AVERAGE             0x41
#define IST8310_REG_PDCNTL              0x42
#define IST8310_ODR_SINGLE              0x01
#define IST8310_ODR_10_HZ               0x03
#define IST8310_ODR_20_HZ               0x05
#define IST8310_ODR_50_HZ               0x07
#define IST8310_ODR_100_HZ              0x06
#define IST8310_ODR_200_HZ              0x0B
#define IST8310_CHIP_ID                 0x10
#define IST8310_AVG_16                  0x24
#define IST8310_PULSE_DURATION_NORMAL   0xC0
#define IST8310_CNTRL2_RESET            0x01
#define IST8310_CNTRL2_DRPOL            0x04
#define IST8310_CNTRL2_DRENA            0x08

#define IST8310_MAG_TO_GAUSS            0.0015f

static Vector3i_t magRaw;

/**********************************************************************************************************
*函 数 名: IST8310_WriteReg
*功能说明: 往IST8310的寄存器写入一个字节的数据
*形    参: 寄存器地址 写入数据
*返 回 值: 无
**********************************************************************************************************/
static void IST8310_WriteReg(u8 REG_Address,u8 REG_data)
{
    Soft_I2c_Single_Write(MAG_I2C, IST8310_ADDRESS, REG_Address, REG_data);
}

/**********************************************************************************************************
*函 数 名: IST8310_ReadReg
*功能说明: 读取IST8310寄存器的数据
*形    参: 寄存器地址
*返 回 值: 寄存器数据
**********************************************************************************************************/
static uint8_t IST8310_ReadReg(u8 REG_Address)
{
    return Soft_I2C_Single_Read(MAG_I2C, IST8310_ADDRESS, REG_Address);
}

/**********************************************************************************************************
*函 数 名: IST8310_Detect
*功能说明: 检测IST8310是否存在
*形    参: 无
*返 回 值: 存在状态
**********************************************************************************************************/
bool IST8310_Detect(void)
{
    if(IST8310_ReadReg(IST8310_REG_WHOAMI) == IST8310_CHIP_ID)
        return true;
    else
        return false;
}

/**********************************************************************************************************
*函 数 名: IST8310_Init
*功能说明: IST8310寄存器配置初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void IST8310_Init(void)
{
    IST8310_WriteReg(IST8310_REG_CNTRL1, IST8310_ODR_200_HZ);
    SoftDelayMs(5);
}

/**********************************************************************************************************
*函 数 名: IST8310_Update
*功能说明: IST8310数据更新
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void IST8310_Update(void)
{
    uint8_t buffer[6];

    buffer[1] = IST8310_ReadReg(IST8310_REG_HX_L);
    buffer[0] = IST8310_ReadReg(IST8310_REG_HX_H);
    magRaw.x = (int16_t)buffer[0] << 8 | buffer[1];

    buffer[3] = IST8310_ReadReg(IST8310_REG_HY_L);
    buffer[2] = IST8310_ReadReg(IST8310_REG_HY_H);
    magRaw.y = (int16_t)buffer[2] << 8 | buffer[3];

    buffer[5] = IST8310_ReadReg(IST8310_REG_HZ_L);
    buffer[4] = IST8310_ReadReg(IST8310_REG_HZ_H);
    magRaw.z = (int16_t)buffer[4] << 8 | buffer[5];

    //统一传感器坐标系（并非定义安装方向）
    magRaw.x = magRaw.x;
    magRaw.y = -magRaw.y;
    magRaw.z = magRaw.z;
}

/**********************************************************************************************************
*函 数 名: IST8310_Read
*功能说明: 读取地磁传感器数据,并转换为标准单位
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void IST8310_Read(Vector3f_t* mag)
{
    mag->x = magRaw.x * IST8310_MAG_TO_GAUSS;
    mag->y = magRaw.y * IST8310_MAG_TO_GAUSS;
    mag->z = magRaw.z * IST8310_MAG_TO_GAUSS;
}


