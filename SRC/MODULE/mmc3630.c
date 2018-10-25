/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     mmc3630.c
 * @说明     MMC3630地磁传感器驱动
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.10
**********************************************************************************************************/
#include "mmc3630.h"
#include "drv_i2c_soft.h"

//模拟IIC地址
#define MMC3630KJ_ADDRESS	        0x60
//硬件IIC地址
//#define MMC3630KJ_ADDRESS	        0x30

#define MMC3630KJ_REG_DATA			0x00
#define MMC3630KJ_REG_XL			0x00
#define MMC3630KJ_REG_XH			0x01
#define MMC3630KJ_REG_YL			0x02
#define MMC3630KJ_REG_YH			0x03
#define MMC3630KJ_REG_ZL			0x04
#define MMC3630KJ_REG_ZH			0x05
#define MMC3630KJ_REG_TEMP			0x06
#define MMC3630KJ_REG_STATUS		0x07
#define MMC3630KJ_REG_CTRL0			0x08
#define MMC3630KJ_REG_CTRL1			0x09
#define MMC3630KJ_REG_CTRL2			0x0A
#define MMC3630KJ_REG_X_THD			0x0B
#define MMC3630KJ_REG_Y_THD			0x0C
#define MMC3630KJ_REG_Z_THD			0x0D
#define MMC3630KJ_REG_SELFTEST		0x0E
#define MMC3630KJ_REG_PASSWORD		0x0F
#define MMC3630KJ_REG_OTPMODE		0x12
#define MMC3630KJ_REG_TESTMODE		0x13
#define MMC3630KJ_REG_SR_PWIDTH		0x20
#define MMC3630KJ_REG_OTP			0x2A
#define MMC3630KJ_REG_PRODUCTID		0x2F

#define MMC3630KJ_CMD_REFILL		0x20
#define MMC3630KJ_CMD_RESET         0x10
#define MMC3630KJ_CMD_SET			0x08
#define MMC3630KJ_CMD_TM_M			0x01
#define MMC3630KJ_CMD_TM_T			0x02
#define MMC3630KJ_CMD_START_MDT		0x04
#define MMC3630KJ_CMD_100HZ			0x00
#define MMC3630KJ_CMD_200HZ			0x01
#define MMC3630KJ_CMD_400HZ			0x02
#define MMC3630KJ_CMD_600HZ			0x03
#define MMC3630KJ_CMD_CM_14HZ		0x01
#define MMC3630KJ_CMD_CM_5HZ		0x02
#define MMC3630KJ_CMD_CM_1HZ		0x04
#define MMC3630KJ_CMD_SW_RST		0x80
#define MMC3630KJ_CMD_PASSWORD		0xE1
#define MMC3630KJ_CMD_OTP_OPER		0x11
#define MMC3630KJ_CMD_OTP_MR		0x80
#define MMC3630KJ_CMD_OTP_ACT		0x80
#define MMC3630KJ_CMD_OTP_NACT		0x00
#define MMC3630KJ_CMD_STSET_OPEN	0x02
#define MMC3630KJ_CMD_STRST_OPEN	0x04
#define MMC3630KJ_CMD_ST_CLOSE		0x00
#define MMC3630KJ_CMD_INT_MD_EN		0x40
#define MMC3630KJ_CMD_INT_MDT_EN	0x20

#define MMC3630KJ_PRODUCT_ID		0x0A
#define MMC3630KJ_OTP_READ_DONE_BIT	0x10
#define MMC3630KJ_PUMP_ON_BIT		0x08
#define MMC3630KJ_MDT_BIT			0x04
#define MMC3630KJ_MEAS_T_DONE_BIT	0x02
#define MMC3630KJ_MEAS_M_DONE_BIT	0x01

// 16-bit mode, null field output (32768)
#define MMC3630KJ_OFFSET			32768.0f
#define MMC3630KJ_SENSITIVITY		1024.0f
#define MMC3630KJ_T_ZERO			(-75.0f)
#define MMC3630KJ_T_SENSITIVITY		0.8

#define OTP_CONVERT(REG)		 ((float)((REG) >=32 ? (32 - (REG)) : (REG)) * 0.006f)

//默认传感器补偿值
static float fOtpMatrix[3] = {1.0f,1.0f,1.35f};

static Vector3i_t magRaw;

/**********************************************************************************************************
*函 数 名: MMC3630_WriteReg
*功能说明: 往MMC3630的寄存器写入一个字节的数据
*形    参: 寄存器地址 写入数据
*返 回 值: 无
**********************************************************************************************************/
static void MMC3630_WriteReg(u8 REG_Address,u8 REG_data)
{
    Soft_I2c_Single_Write(MAG_I2C, MMC3630KJ_ADDRESS, REG_Address, REG_data);
}

/**********************************************************************************************************
*函 数 名: MMC3630_ReadReg
*功能说明: 读取MMC3630寄存器的数据
*形    参: 寄存器地址
*返 回 值: 寄存器数据
**********************************************************************************************************/
static uint8_t MMC3630_ReadReg(u8 REG_Address)
{
    return Soft_I2C_Single_Read(MAG_I2C, MMC3630KJ_ADDRESS, REG_Address);
}

/**********************************************************************************************************
*函 数 名: MMC3630_Detect
*功能说明: 检测MMC3630是否存在
*形    参: 无
*返 回 值: 存在状态
**********************************************************************************************************/
bool MMC3630_Detect(void)
{
    //复位
    MMC3630_WriteReg(MMC3630KJ_REG_CTRL1, MMC3630KJ_CMD_SW_RST);

    SoftDelayMs(10);

    //检查OTP读取状态
    if((MMC3630_ReadReg(MMC3630KJ_REG_STATUS) & 0x10) != MMC3630KJ_OTP_READ_DONE_BIT)
        return false;

    if(MMC3630_ReadReg(MMC3630KJ_REG_PRODUCTID) == MMC3630KJ_PRODUCT_ID)
        return true;
    else
        return false;
}

/**********************************************************************************************************
*函 数 名: MMC3630_Init
*功能说明: MMC3630寄存器配置初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MMC3630_Init(void)
{
    uint8_t reg_data[2] = {0};

    /*读取OTP存储器中的数据，并计算传感器补偿值*/
    MMC3630_WriteReg(MMC3630KJ_REG_PASSWORD, MMC3630KJ_CMD_PASSWORD);
    MMC3630_WriteReg(MMC3630KJ_REG_OTPMODE, MMC3630KJ_CMD_OTP_OPER);
    MMC3630_WriteReg(MMC3630KJ_REG_TESTMODE, MMC3630KJ_CMD_OTP_MR);
    MMC3630_WriteReg(MMC3630KJ_REG_CTRL2, MMC3630KJ_CMD_OTP_ACT);

    reg_data[0] = MMC3630_ReadReg(MMC3630KJ_REG_OTP);
    reg_data[1] = MMC3630_ReadReg(MMC3630KJ_REG_OTP + 1);

    MMC3630_WriteReg(MMC3630KJ_REG_CTRL2, MMC3630KJ_CMD_OTP_NACT);

    fOtpMatrix[0] = 1.0f;
    fOtpMatrix[1] = OTP_CONVERT(reg_data[0] & 0x3f) + 1.0f;
    fOtpMatrix[2] = (OTP_CONVERT((reg_data[1] & 0x0f) << 2 | (reg_data[0] & 0xc0) >> 6) + 1.0f) * 1.35f;

    /*Change the SET/RESET pulse width*/
//	MMC3630_WriteReg(MMC3630KJ_REG_CTRL2, MMC3630KJ_CMD_OTP_NACT);
//	MMC3630_WriteReg(MMC3630KJ_REG_PASSWORD, MMC3630KJ_CMD_PASSWORD);
//
//	reg_data[0] = MMC3630_ReadReg(MMC3630KJ_REG_SR_PWIDTH) & 0xE7;
//
//	MMC3630_WriteReg(MMC3630KJ_REG_SR_PWIDTH, reg_data[0]);

//    /* SET operation when using dual supply */
//    MMC3630_WriteReg(MMC3630KJ_REG_CTRL0, MMC3630KJ_CMD_SET);

    /*设置输出分辨率*/
    MMC3630_WriteReg(MMC3630KJ_REG_CTRL1, MMC3630KJ_CMD_200HZ);

    /*使能测量*/
    MMC3630_WriteReg(MMC3630KJ_REG_CTRL0, MMC3630KJ_CMD_TM_M);

    SoftDelayMs(10);
}

/**********************************************************************************************************
*函 数 名: MMC3630_Update
*功能说明: MMC3630数据更新
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MMC3630_Update(void)
{
    uint8_t buffer[6];
    uint16_t dataTemp[3];
    uint16_t rawData[3];

    buffer[0] = MMC3630_ReadReg(MMC3630KJ_REG_XL);
    buffer[1] = MMC3630_ReadReg(MMC3630KJ_REG_XH);
    dataTemp[0] = (uint16_t)buffer[1] << 8 | buffer[0];

    buffer[2] = MMC3630_ReadReg(MMC3630KJ_REG_YL);
    buffer[3] = MMC3630_ReadReg(MMC3630KJ_REG_YH);
    dataTemp[1] = (uint16_t)buffer[3] << 8 | buffer[2];

    buffer[4] = MMC3630_ReadReg(MMC3630KJ_REG_ZL);
    buffer[5] = MMC3630_ReadReg(MMC3630KJ_REG_ZH);
    dataTemp[2] = (uint16_t)buffer[5] << 8 | buffer[4];

    rawData[0] = dataTemp[0];
    rawData[1] = dataTemp[1] - dataTemp[2] + 32768;
    rawData[2] = dataTemp[1] + dataTemp[2] - 32768;

    magRaw.x = rawData[0] - MMC3630KJ_OFFSET;
    magRaw.y = rawData[1] - MMC3630KJ_OFFSET;
    magRaw.z = rawData[2] - MMC3630KJ_OFFSET;

    //统一传感器坐标系（并非定义安装方向）
    magRaw.x = magRaw.x;
    magRaw.y = -magRaw.y;
    magRaw.z = magRaw.z;

    /*使能测量*/
    MMC3630_WriteReg(MMC3630KJ_REG_CTRL0, MMC3630KJ_CMD_TM_M);
}

/**********************************************************************************************************
*函 数 名: MMC3630_Read
*功能说明: 读取地磁传感器数据,并转换为标准单位
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void MMC3630_Read(Vector3f_t* mag)
{
    mag->x = magRaw.x / MMC3630KJ_SENSITIVITY * fOtpMatrix[0];
    mag->y = magRaw.y / MMC3630KJ_SENSITIVITY * fOtpMatrix[1];
    mag->z = magRaw.z / MMC3630KJ_SENSITIVITY * fOtpMatrix[2];
}


