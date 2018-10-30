#ifndef __PARAMETER_H__
#define __PARAMETER_H__

#include "board.h"

enum PARAM_TYPE
{
    PARAM_CHECK_NUM,
    PARAM_CHECK_SUM,
    /*******陀螺仪校准参数*********/
    PARAM_GYRO_OFFSET_X,
    PARAM_GYRO_OFFSET_Y,
    PARAM_GYRO_OFFSET_Z,
    PARAM_GYRO_SCALE_X,
    PARAM_GYRO_SCALE_Y,
    PARAM_GYRO_SCALE_Z,
    /*******加速度校准参数*********/
    PARAM_ACC_OFFSET_X,
    PARAM_ACC_OFFSET_Y,
    PARAM_ACC_OFFSET_Z,
    PARAM_ACC_SCALE_X,
    PARAM_ACC_SCALE_Y,
    PARAM_ACC_SCALE_Z,
    /*******磁力计校准参数*********/
    PARAM_MAG_OFFSET_X,
    PARAM_MAG_OFFSET_Y,
    PARAM_MAG_OFFSET_Z,
    PARAM_MAG_SCALE_X,
    PARAM_MAG_SCALE_Y,
    PARAM_MAG_SCALE_Z,
    PARAM_MAG_EARTH_MAG,
    /*********水平校准参数*********/
    PARAM_IMU_LEVEL_X,
    PARAM_IMU_LEVEL_Y,
    PARAM_IMU_LEVEL_Z,
    /*********姿态PID参数*********/
    PARAM_PID_ATT_INNER_X_KP,
    PARAM_PID_ATT_INNER_X_KI,
    PARAM_PID_ATT_INNER_X_KD,
    PARAM_PID_ATT_INNER_Y_KP,
    PARAM_PID_ATT_INNER_Y_KI,
    PARAM_PID_ATT_INNER_Y_KD,
    PARAM_PID_ATT_INNER_Z_KP,
    PARAM_PID_ATT_INNER_Z_KI,
    PARAM_PID_ATT_INNER_Z_KD,
    PARAM_PID_ATT_OUTER_X_KP,
    PARAM_PID_ATT_OUTER_Y_KP,
    PARAM_PID_ATT_OUTER_Z_KP,
    /*********位置PID参数*********/
    PARAM_PID_POS_INNER_X_KP,
    PARAM_PID_POS_INNER_X_KI,
    PARAM_PID_POS_INNER_X_KD,
    PARAM_PID_POS_INNER_Y_KP,
    PARAM_PID_POS_INNER_Y_KI,
    PARAM_PID_POS_INNER_Y_KD,
    PARAM_PID_POS_INNER_Z_KP,
    PARAM_PID_POS_INNER_Z_KI,
    PARAM_PID_POS_INNER_Z_KD,
    PARAM_PID_POS_OUTER_X_KP,
    PARAM_PID_POS_OUTER_Y_KP,
    PARAM_PID_POS_OUTER_Z_KP,
    /*********电调校准标志*********/
    PARAM_ESC_CALI_FLAG,
    /******************************/
    PARAM_NUM
};

void ParamInit(void);
void ParamSaveToFlash(void);
void ParamUpdateData(uint16_t dataNum, const void * data);
void ParamGetData(uint16_t dataNum, void *data, uint8_t length);
const char* ParamGetString(uint8_t paramNum);

#endif

