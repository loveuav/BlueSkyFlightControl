/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     parameter.c
 * @说明     飞控参数
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "parameter.h"
#include "drv_flash.h"
#include "mathTool.h"
#include "flightStatus.h"

//参数字符标识，不能超过16个字符
const char* paramStrings[] = 
{
    "GYRO_OFFSET_X",
    "GYRO_OFFSET_Y",
    "GYRO_OFFSET_Z",
    "GYRO_SCALE_X",
    "GYRO_SCALE_Y",
    "GYRO_SCALE_Z",
    "ACC_OFFSET_X",
    "ACC_OFFSET_Y",
    "ACC_OFFSET_Z",
    "ACC_SCALE_X",
    "ACC_SCALE_Y",
    "ACC_SCALE_Z",
    "MAG_OFFSET_X",
    "MAG_OFFSET_Y",
    "MAG_OFFSET_Z",
    "MAG_SCALE_X",
    "MAG_SCALE_Y",
    "MAG_SCALE_Z",
    "MAG_EARTH_MAG",
    "IMU_LEVEL_X",
    "IMU_LEVEL_Y",
    "IMU_LEVEL_Z",	
    "ATT_INNER_X_KP",
    "ATT_INNER_X_KI",
    "ATT_INNER_X_KD",   
    "ATT_INNER_Y_KP",
    "ATT_INNER_Y_KI",
    "ATT_INNER_Y_KD",   
    "ATT_INNER_Z_KP",
    "ATT_INNER_Z_KI",
    "ATT_INNER_Z_KD", 
    "ATT_OUTER_X_KP",
    "ATT_OUTER_Y_KP",
    "ATT_OUTER_Z_KP",     
    "POS_INNER_X_KP",
    "POS_INNER_X_KI",
    "POS_INNER_X_KD",   
    "POS_INNER_Y_KP",
    "POS_INNER_Y_KI",
    "POS_INNER_Y_KD",   
    "POS_INNER_Z_KP",
    "POS_INNER_Z_KI",
    "POS_INNER_Z_KD", 
    "POS_OUTER_X_KP",
    "POS_OUTER_Y_KP",
    "POS_OUTER_Z_KP"
};

static void ParamReadFromFlash(void);

//参数读出缓冲区
static uint8_t param_data[PARAM_NUM*4];	
//参数写入计数器
static uint16_t param_save_cnt = 0;	

/**********************************************************************************************************
*函 数 名: ParamInit
*功能说明: 参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ParamInit(void)
{
	ParamReadFromFlash();
}

/**********************************************************************************************************
*函 数 名: ParamDataReset
*功能说明: 参数置零
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void ParamDataReset(void)
{
    uint32_t i;
    
    for(i=0;i<PARAM_NUM*4;i++)
    {
        param_data[i] = 0;
    }
}

/**********************************************************************************************************
*函 数 名: ParamReadFromFlash
*功能说明: 把飞控参数存储区的内容读取出来
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void ParamReadFromFlash(void)
{
	uint32_t i = 0;
    uint32_t dataSum = 0, checkNum = 0, checkSum = 0;
    
	for (i=0;i<PARAM_NUM*4;i++)
		param_data[i] = Flash_ReadByte(FLASH_USER_PARA_START_ADDR, i);

    ParamGetData(PARAM_CHECK_NUM, &checkNum, 4);  
    checkNum = ConstrainInt32(checkNum, 0, PARAM_NUM);    
    ParamGetData(PARAM_CHECK_SUM, &checkSum, 4);
    
    //计算参数和
    for(i=8;i<checkNum*4;i++)
    {
        dataSum += param_data[i];
    }
    
    //和保存的校验和进行对比，如果不符合则重置所有参数
    if(checkSum != dataSum)
    {
        ParamDataReset();
    }
}

/**********************************************************************************************************
*函 数 名: ParamSaveToFlash
*功能说明: 把飞控参数写入存储区 运行频率20Hz
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ParamSaveToFlash(void)
{
    uint32_t i = 0;
    uint32_t dataSum = 0;
    uint32_t dataNum = 0;
    
    //解锁后不得保存参数
    if(GetArmedStatus() == ARMED)
    {
        param_save_cnt = 0;
        return;
    }
    
    if(param_save_cnt == 1)
    {
        //保存参数数量
        dataNum = PARAM_NUM;
        memcpy(param_data+PARAM_CHECK_NUM*4, &dataNum, 4);
        
        //计算参数和并保存
        for(i=8;i<PARAM_NUM*4;i++)
        {
            dataSum += param_data[i];
        }              
        memcpy(param_data+PARAM_CHECK_SUM*4, &dataSum, 4);
    
        Flash_WriteByte(FLASH_USER_PARA_START_ADDR, param_data, PARAM_NUM*4);
    }
	
	if(param_save_cnt > 0)
		param_save_cnt--;
}

/**********************************************************************************************************
*函 数 名: ParamUpdateData
*功能说明: 更新飞控参数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ParamUpdateData(uint16_t dataNum, const void *data)
{
	memcpy(param_data+dataNum*4, data, 4);
	//参数更新的3秒后刷新一次Flash
	param_save_cnt = 60;
}

/**********************************************************************************************************
*函 数 名: ParamGetData
*功能说明: 获取飞控参数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ParamGetData(uint16_t dataNum, void *data, uint8_t length)
{
	memcpy(data, param_data+dataNum*4, length);
}

/**********************************************************************************************************
*函 数 名: ParamGetString
*功能说明: 获取参数标识符
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
const char* ParamGetString(uint8_t paramNum)
{
	return paramStrings[paramNum];
}


