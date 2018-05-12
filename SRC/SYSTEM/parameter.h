#ifndef __PARAMETER_H__
#define __PARAMETER_H__

#include "board.h"

enum {
    PARAM_CHECK_NUM,    
    PARAM_CHECK_SUM,
    PARAM_GYRO_OFFSET_X,
    PARAM_GYRO_OFFSET_Y,
    PARAM_GYRO_OFFSET_Z,
    PARAM_GYRO_SCALE_X,
    PARAM_GYRO_SCALE_Y,
    PARAM_GYRO_SCALE_Z,
    PARAM_ACC_OFFSET_X,
    PARAM_ACC_OFFSET_Y,
    PARAM_ACC_OFFSET_Z,
    PARAM_ACC_SCALE_X,
    PARAM_ACC_SCALE_Y,
    PARAM_ACC_SCALE_Z,
    PARAM_MAG_OFFSET_X,
    PARAM_MAG_OFFSET_Y,
    PARAM_MAG_OFFSET_Z,
    PARAM_MAG_SCALE_X,
    PARAM_MAG_SCALE_Y,
    PARAM_MAG_SCALE_Z,
    PARAM_MAG_EARTH_MAG,
    PARAM_IMU_LEVEL_X,
    PARAM_IMU_LEVEL_Y,
    PARAM_IMU_LEVEL_Z,	
    PARAM_NUM
};

void ParamInit(void);
void ParamSaveToFlash(void);
void ParamUpdateData(uint16_t dataNum, const void * data);
void ParamGetData(uint16_t dataNum, void *data, uint8_t length);

#endif

