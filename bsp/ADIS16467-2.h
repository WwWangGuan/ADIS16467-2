
#ifndef ADIS16467_ADIS16467_2_H
#define ADIS16467_ADIS16467_2_H

#include "main.h"

#define DIAG_STAT_REG 0x02

#define X_GYRO_LOW_REG 0x04
#define X_GYRO_OUT_REG 0x06
#define Y_GYRO_LOW_REG 0x08
#define Y_GYRO_OUT_REG 0x0A
#define Z_GYRO_LOW_REG 0x0C
#define Z_GYRO_OUT_REG 0x0E

#define X_ACCL_LOW_REG 0x10
#define X_ACCL_OUT_REG 0x12
#define Y_ACCL_LOW_REG 0x14
#define Y_ACCL_OUT_REG 0x16
#define Z_ACCL_LOW_REG 0x18
#define Z_ACCL_OUT_REG 0x1A

#define TEMP_OUT_REG 0x1C
#define TIME_STAMP_REG 0x1E

#define DATA_CNTR_REG 0x22

#define X_DELTANG_LOW_REG 0x24
#define X_DELTANG_OUT_REG 0x26
#define Y_DELTANG_LOW_REG 0x28
#define Y_DELTANG_OUT_REG 0x2A
#define Z_DELTANG_LOW_REG 0x2C
#define Z_DELTANG_OUT_REG 0x2E

#define X_DELTVEL_LOW_REG 0x30
#define X_DELTVEL_OUT_REG 0x32
#define Y_DELTVEL_LOW_REG 0x34
#define Y_DELTVEL_OUT_REG 0x36
#define Z_DELTVEL_LOW_REG 0x38
#define Z_DELTVEL_OUT_REG 0x3A

#define XG_BIAS_LOW_REG 0x40
#define XG_BIAS_HIGH_REG 0x42
#define YG_BIAS_LOW_REG 0x44
#define YG_BIAS_HIGH_REG 0x46
#define ZG_BIAS_LOW_REG 0x48
#define ZG_BIAS_HIGH_REG 0x4A

#define XA_BIAS_LOW_REG 0x4C
#define XA_BIAS_HIGH_REG 0x4E
#define YA_BIAS_LOW_REG 0x50
#define YA_BIAS_HIGH_REG 0x52
#define ZA_BIAS_LOW_REG 0x54
#define ZA_BIAS_HIGH_REG 0x56

#define FILT_CTRL_REG 0x5C
#define RANG_MDL_REG 0x5E
#define MSC_CTRL_REG 0x60
#define UP_SCALE_REG 0x62
#define DEC_RATE_REG 0x64
#define NULL_CNFG_REG 0x66
#define GLOB_CMD_REG 0x68

#define FIRM_REV_REG 0x6C
#define FIRM_DM_REG 0x6E
#define FIRM_Y_REG 0x70
#define PROD_ID_REG 0x72
#define SERIAL_NUM_REG 0x74

#define USER_SCR_1_REG 0x76
#define USER_SCR_2_REG 0x78
#define USER_SCR_3_REG 0x7A

#define FLSHCNT_LOW_REG 0x7C
#define FLSHCNT_HIGH_REG 0x7E

typedef struct
{
    SPI_HandleTypeDef *hspi; //SPI handle
    GPIO_TypeDef *GPIOx;     //CS pin GPIOx
    uint16_t GPIO_PIN;       //CS pin.

    uint16_t status;
    uint16_t rangeModel; //Measurement range (model specific) identifier.
    uint16_t prodId; //Identification, device number,default 0x4053.

    uint16_t firm_rev; //Identification, firmware revision
    uint16_t firm_dm;  //Identification, date code, day and month`
    uint16_t firm_y;  //Identification, date code, year
    uint16_t serial_num; //Identification, serial number


    int32_t Accel_X_RAW;
    int32_t Accel_Y_RAW;
    int32_t Accel_Z_RAW;

    float Ax;
    float Ay;
    float Az;

    int32_t Gyro_X_RAW;
    int32_t Gyro_Y_RAW;
    int32_t Gyro_Z_RAW;
    float Gx;
    float Gy;
    float Gz;

    float Temperature; 
} ADIS16467_t;

#endif //ADIS16467_ADIS16467_2_H
