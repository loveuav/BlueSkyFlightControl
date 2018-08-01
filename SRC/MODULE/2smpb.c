/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     2smpb.c
 * @说明     2smpb欧姆龙气压传感器驱动
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.06 
**********************************************************************************************************/
#include "2smpb.h"
#include "drv_i2c_soft.h"
#include "board.h"

#define _2SMPB_ADDR      	            0xE0 

#define _2SMPB02_NORMALMODE

#define ADDR_RESET_CMD				    0x1E	/* write to this address to reset chip */
#define ADDR_CMD_CONVERT_D1_OSR256		0x40	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR512		0x42	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR1024		0x44	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR2048		0x46	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR4096		0x48	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D2_OSR256		0x50	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR512		0x52	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR1024		0x54	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR2048		0x56	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR4096		0x58	/* write to this address to start temperature conversion */

#define ADDR_CMD_CONVERT_D1			    ADDR_CMD_CONVERT_D1_OSR1024
#define ADDR_CMD_CONVERT_D2			    ADDR_CMD_CONVERT_D2_OSR1024

#define ADDR_DATA					    0x00	/* address of 3 bytes / 32bit pressure data */
#define ADDR_PROM_SETUP				    0xA0	/* address of 8x 2 bytes factory and calibration data */
#define ADDR_PROM_C1					0xA2	/* address of 6x 2 bytes calibration data */

// 2SMPB-02 寄存器地址
/** 2SMPB-02 registers for reading temperature digits */
#define _2SMPB02_TEMP_TXD2              ((uint8_t)0xFA)
/** 2SMPB-02 registers for reading pressure digits */
#define _2SMPB02_PRES_TXD2              ((uint8_t)0xF7)
/** 2SMPB-02 registers to control sensor running */
#define _2SMPB02_CTRL_MEAS_REG          ((uint8_t)0xF4)
/** 2SMPB-02 register to get hard wired id */
#define _2SMPB02_CHIP_ID1_REG           ((uint8_t)0xD1)
/** 2SMPB-02 register to set the sensor period and SPI settings */
#define _2SMPB02_SETUP_REG              ((uint8_t)0xF5)
/** 2SMPB-02 register to get the sensor running status */
#define _2SMPB02_STATUS_REG             ((uint8_t)0xF3)
/** 2SMPB-02 register to get the sensor running status */
#define _2SMPB02_I2CHS_REG              ((uint8_t)0xF2)
/** 2SMPB-02 register to get the sensor running status */
#define _2SMPB02_IIR_REG                ((uint8_t)0xF1)
/** 2SMPB-02 register to get the sensor running status */
#define _2SMPB02_RESET_REG              ((uint8_t)0xE0)
/** 2SMPB-02 registers to get calculation parameters */
#define _2SMPB02_PROM_START_ADDR        ((uint8_t)0xA0)
/** 2SMPB-02 length of calculation parameters in bytes */
//#define _2SMPB02_PROM_DATA_LEN          ((int32_t)32)
#define _2SMPB02_PROM_DATA_LEN          ((int32_t)24)

//操作命令
#define _2SMPB02_NORMALMODE
/** 2SMPB-02 Hard wired value to get ID */
#define _2SMPB02_CHIP_ID1               ((uint8_t)0x5C)
/** Setup bits for 8sec period */
#define _2SMPB02_SETUP_STBY_8SEC        ((uint8_t)0xE0)
/** Setup bits for 4sec period */
#define _2SMPB02_SETUP_STBY_4SEC        ((uint8_t)0xC0)
/** Setup bits for 2sec period */
#define _2SMPB02_SETUP_STBY_2SEC        ((uint8_t)0xA0)
/** Setup bits for 1sec period */
#define _2SMPB02_SETUP_STBY_1SEC        ((uint8_t)0x80)
/** Setup bits for 500msec period */
#define _2SMPB02_SETUP_STBY_500MSEC     ((uint8_t)0x60)
/** Setup bits for 250msec period */
#define _2SMPB02_SETUP_STBY_250MSEC     ((uint8_t)0x40)
/** Setup bits for 125msec period */
#define _2SMPB02_SETUP_STBY_125MSEC     ((uint8_t)0x20)
/** Setup bits for 1msec period */
#define _2SMPB02_SETUP_STBY_1MSEC       ((uint8_t)0x00)
/** 2SMPB-02 Status bit of measurement running */
#define _2SMPB02_STATUS_MEAS            ((uint8_t)8)
/** 2SMPB-02 Status bit of otp reading */
#define _2SMPB02_STATUS_OTP             ((uint8_t)1)
/** for I2C High speed mode */
#define _2SMPB02_I2CHS_MASTERCODE       ((uint8_t)0x3)

/** IIR bits to set IIR off  */
#define _2SMPB02_IIR_OFF                ((uint8_t)0)
/** IIR bits to set repeating IIR 2times */
#define _2SMPB02_IIR_2                  ((uint8_t)1)
/** IIR bits to set repeating IIR 4times */
#define _2SMPB02_IIR_4                  ((uint8_t)2)
/** IIR bits to set repeating IIR 8times */
#define _2SMPB02_IIR_8                  ((uint8_t)3)
/** IIR bits to set repeating IIR 16times */
#define _2SMPB02_IIR_16                 ((uint8_t)4)
/** IIR bits to set repeating IIR 32times */
#define _2SMPB02_IIR_32                 ((uint8_t)5)

/** Reset hard-code for the reset register */
#define _2SMPB02_RESET_CODE             ((uint8_t)0xE6)

#ifdef _2SMPB02_NORMALMODE
    /** 2SMPB-02 Operation mode to normal mode (run periodical) */
    #define _2SMPB02_CTRL_MODE        3
#else
    /** 2SMPB-02 Operation mode to force mode (run once) */
    #define _2SMPB02_CTRL_MODE        1
#endif

/** compensation parameter values: _A_ is center, _S_ is range. */
#define _2SMPB02_S_AP           ((double)(1.0 / 3.0E-05 * 32767))
#define _2SMPB02_A_AP           ((double)(+0.0E+00             ))
#define _2SMPB02_S_BP           ((double)(1.0 / 1.0E+01 * 32767))
#define _2SMPB02_A_BP           ((double)(+3.0E+01             ))

#define _2SMPB02_S_AT           ((double)(1.0 / 8.0E-11 * 32767))
#define _2SMPB02_A_AT           ((double)(+0.0E+00             ))
#define _2SMPB02_S_BT           ((double)(1.0 / 1.6E-06 * 32767))
#define _2SMPB02_A_BT           ((double)(-6.6E-06             ))
#define _2SMPB02_S_CT           ((double)(1.0 / 8.5E-03 * 32767))
#define _2SMPB02_A_CT           ((double)(1.0+4.0E-02        ))

#define _2SMPB02_S_AA           ((double)(1.0 / 4.2E-04 * 32767))
#define _2SMPB02_A_AA           ((double)(+0.0E+00             ))
#define _2SMPB02_S_BA           ((double)(1.0 / 8.0E+00 * 32767))
#define _2SMPB02_A_BA           ((double)(-1.6E+02             ))
	
typedef struct{
  uint16_t PTAT3;
  uint16_t PTAT2;
  uint16_t PTAT1;
  uint16_t CEXPT;
  uint16_t TEMP3;
  uint16_t TEMP2;
  uint16_t TEMP1;
  uint16_t CEXPR;
  uint16_t PR3;
  uint16_t PR2;
  uint16_t PR1;
}SMPARAM_OBJ;

typedef struct{
  float press;	        //pressure
  float altitude;       //height 
  
  uint8_t  nRet;
  uint32_t nTemper;
  uint32_t nPressCoef;
  double fOrmTemp;
  double fOrmPress;
  int32_t nTemper1;
  uint32_t nPressPa;
  int32_t nPress1;
  uint32_t nRefValidDelay;
  double dbRefPress;
  uint8_t enabled;
  uint8_t initialized;
  volatile uint32_t lastUpdate;
  volatile float temp;	
  float rawTemp;
  uint8_t smpb02ReadInit;	
}smpb02Struct_t;

typedef struct{
    /** calculation parameters: pressure conversion */
    double AP, BP, CP;
    /** calculation parameters: temperature compensation */
    double AT, BT, CT;
    /** calculation parameters: temperature conversion */
    double AA, BA, CA;
}SMCOEF_OBJ;

static int32_t conv24sign(uint32_t d);
static int16_t conv16sign(uint16_t d);
static void JoinOtp(uint8_t *pReadData);
static void Otp2Coef(SMPARAM_OBJ* pPressParam);
static double _2SMPB02_convTx(SMCOEF_OBJ* coef, int32_t dt);
static double _2SMPB02_get_pressure(SMCOEF_OBJ* coef, int32_t dp, double tx);
static double Rel_AltitudeDouble(double Press, double Ref_P);

SMPARAM_OBJ pressParam;
SMCOEF_OBJ coefParam;
smpb02Struct_t _2smpbData;

void _2SMPB_Write_Reg(u8 REG_Address,u8 REG_data)
{
	Soft_I2c_Single_Write(2, _2SMPB_ADDR, REG_Address, REG_data);
}

uint8_t _2SMPB_Read_Reg(u8 REG_Address)
{
	return Soft_I2C_Single_Read(2, _2SMPB_ADDR, REG_Address);
}

void _2SMPB_MultiRead(u8 REG_Address,u8 * ptChar,u8 size)
{
	Soft_I2C_Multi_Read(2, _2SMPB_ADDR, REG_Address, ptChar, size);
}

/**********************************************************************************************************
*函 数 名: _2SMPB_Detect
*功能说明: 检测2SMPB是否存在
*形    参: 无
*返 回 值: 存在状态
**********************************************************************************************************/
bool _2SMPB_Detect(void)
{
    static uint8_t chipID; //0x5C 

    Soft_I2c_Open(2);
    SoftDelayMs(30);
    
	//软件复位
	_2SMPB_Write_Reg(_2SMPB02_RESET_REG, _2SMPB02_RESET_CODE);
	SoftDelayMs(10);
    
	//读取ID
	chipID = _2SMPB_Read_Reg(_2SMPB02_CHIP_ID1_REG);    
    
    if(chipID == 0x5C)
        return true;
    else
        return false;
}

/**********************************************************************************************************
*函 数 名: _2SMPB_Init
*功能说明: 2SMPB寄存器配置初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void _2SMPB_Init(void)
{
    uint8_t cmdBuf[32];
	
	//软件复位
	_2SMPB_Write_Reg(_2SMPB02_RESET_REG, _2SMPB02_RESET_CODE);
	SoftDelayMs(10);

	//设置IIR滤波器
	_2SMPB_Write_Reg(_2SMPB02_IIR_REG, _2SMPB02_IIR_OFF);
	
	//设置待机时间1ms
	_2SMPB_Write_Reg(_2SMPB02_SETUP_REG, _2SMPB02_SETUP_STBY_1MSEC);		
	
	//读取补偿参数
	_2SMPB_MultiRead(_2SMPB02_PROM_START_ADDR, cmdBuf, _2SMPB02_PROM_DATA_LEN);	
	JoinOtp(cmdBuf);					
	Otp2Coef(&pressParam);
	
	//温度平均测量时间，气压平均测量时间，电源模式
	_2SMPB_Write_Reg(_2SMPB02_CTRL_MEAS_REG, ((1 << 5) | (4 << 2) | _2SMPB02_CTRL_MODE));
}

/**********************************************************************************************************
*函 数 名: _2SMPB_Temperature
*功能说明: 2SMPB读取温度
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void _2SMPB_ReadTemperature(void)
{
	uint8_t cmdBuf[8] = {0};
	_2SMPB_MultiRead(_2SMPB02_TEMP_TXD2, cmdBuf, 3);
	
	_2smpbData.nTemper = ((cmdBuf[0]<<16) | (cmdBuf[1]<<8) | cmdBuf[2]);
	_2smpbData.nTemper1 = (int32_t) _2smpbData.nTemper-0x800000;
	_2smpbData.fOrmTemp = _2SMPB02_convTx(&coefParam, _2smpbData.nTemper1);
	_2smpbData.rawTemp = _2smpbData.fOrmTemp / 256.0;	
}

/**********************************************************************************************************
*函 数 名: _2SMPB_ReadPressure
*功能说明: 2SMPB读取气压
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void _2SMPB_ReadPressure(void)
{
	uint8_t cmdBuf[8] = {0};
	
	_2SMPB_MultiRead(_2SMPB02_PRES_TXD2, cmdBuf, 3);
	
	_2smpbData.nPressPa = ((cmdBuf[0]<<16) | (cmdBuf[1]<<8) | cmdBuf[2]);
	_2smpbData.nPress1 = (int32_t)_2smpbData.nPressPa - 0x800000;
				
	_2smpbData.fOrmPress = _2SMPB02_get_pressure(&coefParam, _2smpbData.nPress1, _2smpbData.fOrmTemp);
	
	_2smpbData.altitude = Rel_AltitudeDouble(_2smpbData.fOrmPress / 1000, 101325.0f / 1000) * 100;
}

/**********************************************************************************************************
*函 数 名: _2SMPB_Update
*功能说明: 2SMPB数据更新
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void _2SMPB_Update(void)
{
	_2SMPB_ReadTemperature();
	
	_2SMPB_ReadPressure();
}

/**********************************************************************************************************
*函 数 名: _2SMPB_Read
*功能说明: 读取2SMPB的气压高度值
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void _2SMPB_Read(int32_t* baroAlt)
{
    *baroAlt = _2smpbData.altitude;
}

/**********************************************************************************************************
*函 数 名: _2SMPB_ReadTemp
*功能说明: 读取2SMPB的温度值
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void _2SMPB_ReadTemp(float* temp)
{
    *temp = _2smpbData.rawTemp;
}

static void JoinOtp(uint8_t *pReadData) 
{
    pressParam.CEXPR = (pReadData[ 0] << 8) | 0x00;
    pressParam.PR1   = (pReadData[ 1] << 8) | pReadData[ 2];
    pressParam.PR2   = (pReadData[ 3] << 8) | pReadData[ 4];
    pressParam.PR3   = (pReadData[ 5] << 8) | pReadData[ 6];
    pressParam.TEMP1 = (pReadData[ 7] << 8) | pReadData[ 8];
    pressParam.TEMP2 = (pReadData[ 9] << 8) | pReadData[10];
    pressParam.TEMP3 = (pReadData[11] << 8) | pReadData[12];
    pressParam.CEXPT = (pReadData[13] << 8) | 0x00;
    pressParam.PTAT1 = (pReadData[14] << 8) | pReadData[15];
    pressParam.PTAT2 = (pReadData[17] << 8) | pReadData[18];
    pressParam.PTAT3 = (pReadData[19] << 8) | pReadData[20];
}

static void Otp2Coef(SMPARAM_OBJ* pPressParam)
{
    uint16_t ex;

    ex = (pPressParam->CEXPR & 0xFF00) >> 8;
    coefParam.CP = (double)conv24sign((ex << 16) | pPressParam->PR1);
    coefParam.BP = (double)conv16sign(pPressParam->PR2) / _2SMPB02_S_BP + _2SMPB02_A_BP;
    coefParam.AP = (double)conv16sign(pPressParam->PR3) / _2SMPB02_S_AP + _2SMPB02_A_AP;

    coefParam.CT = (double)conv16sign(pPressParam->TEMP1) / _2SMPB02_S_CT + _2SMPB02_A_CT;
    coefParam.BT = (double)conv16sign(pPressParam->TEMP2) / _2SMPB02_S_BT + _2SMPB02_A_BT;
    coefParam.AT = (double)conv16sign(pPressParam->TEMP3) / _2SMPB02_S_AT + _2SMPB02_A_AT;

    ex = (pPressParam->CEXPT & 0xFF00) >> 8;
    coefParam.CA = (double)conv24sign((ex << 16) | pPressParam->PTAT1);
    coefParam.BA = (double)conv16sign(pPressParam->PTAT2) / _2SMPB02_S_BA + _2SMPB02_A_BA;
    coefParam.AA = (double)conv16sign(pPressParam->PTAT3) / _2SMPB02_S_AA + _2SMPB02_A_AA;
}

static int32_t conv24sign(uint32_t d)
{
    int32_t ret;

    if ((d & 0x800000) != 0) 
    {
        ret = (int32_t)d - 0x1000000;
    } 
    else 
    {
        ret = (int32_t)d;
    }
    return ret;
}

static int16_t conv16sign(uint16_t d)
{
    int32_t ret;

    if ((d & 0x8000) != 0) 
    {
        ret = (int32_t)d - 0x10000;
    }
    else
    {
        ret = d;
    }

    return ret;
}

static double _2SMPB02_convTx(SMCOEF_OBJ* coef, int32_t dt)
{
    double tx, wk;
    double ddt;

    ddt = dt;

    wk = coef->BA * coef->BA - (4 * coef->AA * (coef->CA - ddt));
    if(coef->AA == 0.0) 
    {
        tx = 0.0;
    }
    else if (wk < 0.0) 
    {
        tx = 0.0;
    }
    else 
    {
        tx = (-1.0 * coef->BA - sqrt(wk)) / (2 * coef->AA);
    }
    return tx;
}

static double _2SMPB02_convPl(SMCOEF_OBJ* coef, int32_t dp)
{
    double pl, wk;
    double ddp;

    ddp = dp;

    wk = coef->BP * coef->BP - (4 * coef->AP * (coef->CP - ddp));
    if (coef->AP == 0.0)
    {
        pl = 0.0;
    }
    else if (wk < 0.0)
    {
        pl = 0.0;
    }
    else 
    {
        pl = (-1.0 * coef->BP + sqrt(wk)) / (2 * coef->AP);
    }
    return pl;
}

static double _2SMPB02_get_pressure(SMCOEF_OBJ* coef, int32_t dp, double tx)
{
    double pl, po;
    pl = _2SMPB02_convPl(coef, dp);
    po = (coef->AT * tx * tx) + (coef->BT * tx) + coef->CT;
    if (po == 0.0)
    {
        return 0.0;
    }
    po = pl / po;
    return po;
}

static double Rel_AltitudeDouble(double Press, double Ref_P)
{										
    const double T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
    const double a  = -6.5 / 1000.0;	/* temperature gradient in degrees per metre */
    const double g  = 9.80665;	        /* gravity constant in m/s/s */
    const double R  = 287.05;	        /* ideal gas constant in J/kg/K */

    return (((pow((Press / Ref_P), (-(a * R) / g))) * T1) - T1)/a;
}























