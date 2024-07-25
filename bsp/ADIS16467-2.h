
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

#define Gravity 9.80665
#define ThetaMax 720
#define DeltaVelCof 0.01221
typedef struct {
    /* RAW register data */
    int32_t Gyro_X_RAW;
    int32_t Gyro_Y_RAW;
    int32_t Gyro_Z_RAW;
    /* 16 bit data */
    float Gyro_X;
    float Gyro_Y;
    float Gyro_Z;
    /*32 bit data, additional */
    float HP_Gyro_X;
    float HP_Gyro_Y;
    float HP_Gyro_Z;
} Gyro_T;
typedef struct {
    /* RAW register data */
    int32_t Accel_X_RAW;
    int32_t Accel_Y_RAW;
    int32_t Accel_Z_RAW;
    /* 16 bit data */
    float Accel_X;
    float Accel_Y;
    float Accel_Z;
    /*32 bit data, additional */
    float HP_Accel_X;
    float HP_Accel_Y;
    float HP_Accel_Z;
} Accel_T;
typedef struct {
    /* RAW register data */
    int32_t DeltaAngle_X_RAW;
    int32_t DeltaAngle_Y_RAW;
    int32_t DeltaAngle_Z_RAW;
    /* 16 bit data */
    float DeltaAngle_X;
    float DeltaAngle_Y;
    float DeltaAngle_Z;
    /*32 bit data, additional */
    float HP_DeltaAngle_X;
    float HP_DeltaAngle_Y;
    float HP_DeltaAngle_Z;
} DeltaAngle_T;
typedef struct {
    /* RAW register data */
    int32_t DeltaVelocity_X_RAW;
    int32_t DeltaVelocity_Y_RAW;
    int32_t DeltaVelocity_Z_RAW;
    /* 16 bit data */
    float DeltaVelocity_X;
    float DeltaVelocity_y;
    float DeltaVelocity_Z;
    /*32 bit data, additional */
    float HP_DeltaVelocity_X;
    float HP_DeltaVelocity_Y;
    float HP_DeltaVelocity_Z;
} DeltaVelocity_T;
typedef struct {
    SPI_HandleTypeDef *hspi; //SPI handle
    GPIO_TypeDef *GPIOx;     //CS pin GPIOx
    uint16_t GPIO_PIN;       //CS pin.

    uint8_t K_G; //scale factor for Gyroscope ,ADSI16467-2 default 40.
    float KG_Reciprocal; //the reciprocal of scale factor,to accelerate calculate.
    float g; //liner



    uint16_t status;
    uint16_t RangeModel; //Measurement range (model specific) identifier.
    uint16_t ProdId; //Identification, device number,default 0x4053.

    uint16_t FirmRev; //Identification, firmware revision.
    uint16_t Firm_Month; //Factory configuration month,HEX.
    uint16_t Firm_Day; //Factory configuration day, HEX.
    uint16_t Firm_Year; //Identification, date code, year, HEX.
    uint16_t Serial_Num; //Lot specific serial number.

    Accel_T ADIS_Accel;

    Gyro_T ADIS_Gyro;

    DeltaAngle_T ADIS_DeltaAng;

    DeltaVelocity_T ADIS_DeltaVel;

    float Temperature;
} ADIS16467_T;

void ADIS16467_Init(ADIS16467_T *device);

int ADIS16467_Check(ADIS16467_T *device);

void ADIS16467_imuInfo(ADIS16467_T *device);

void ADIS16467_Read_Accel(ADIS16467_T *device);

void ADIS16467_Read_Gyro(ADIS16467_T *device);

void ADIS16467_Read_Temp(ADIS16467_T *device);

void ADIS16467_Read_DeltaAngle(ADIS16467_T *imu);

void ADIS16467_Read_DeltaVel(ADIS16467_T *imu);

int8_t ADI_Read_Reg(ADIS16467_T *device, uint8_t addr, uint16_t *receive, uint8_t num);

int8_t ADI_Write_Reg(ADIS16467_T *device, uint8_t addr, uint8_t value);

uint16_t ADI_flame_TandR(ADIS16467_T *device, uint16_t trans);

void sb_delay(volatile uint32_t t);

#endif //ADIS16467_ADIS16467_2_H
