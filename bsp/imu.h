//
// Created by wangguan on 24-5-4.
//

#ifndef ADIS16467_IMU_H
#define ADIS16467_IMU_H
#include "stm32g474xx.h"
#include "arm_math.h"


#include "spi.h"
#include "stm32g4xx_hal.h"

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

// ADIS16467 structure
typedef struct
{
    SPI_HandleTypeDef *hspi; //SPI handle
    GPIO_TypeDef *GPIOx;     //CS pin GPIOx
    uint16_t GPIO_PIN;       //CS pin number

    uint16_t status;
    uint16_t rangeModel;
    uint16_t prodId;
    uint16_t firm_rev;
    uint16_t firm_dm;
    uint16_t firm_y;
    uint16_t serial_num;


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

void ADIS16467_Init(ADIS16467_t *device);
int ADIS16467_Check(ADIS16467_t *device);
void ADIS16467_DeviceInfo(ADIS16467_t *device);
void ADIS16467_Read_Accel(ADIS16467_t *device);
void ADIS16467_Read_Gyro(ADIS16467_t *device);
void ADIS16467_Read_Temp(ADIS16467_t *device);
int8_t ADI_Read_Reg(ADIS16467_t *device, uint8_t addr, uint16_t *receive, uint8_t num);
int8_t ADI_Write_Reg(ADIS16467_t *device, uint8_t addr, uint8_t value);
uint16_t ADI_flame_TandR(ADIS16467_t *device, uint16_t trans);
void sb_delay(volatile uint32_t t);

#endif //ADIS16467_IMU_H
