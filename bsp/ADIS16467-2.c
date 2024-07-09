//
// Created by wangguan on 24-5-5.
//

#include "ADIS16467-2.h"

static int16_t Burst_Cmd = 0x6800;

void ADIS16467_Init(ADIS16467_T *imu) {
    ADI_Write_Reg(imu, GLOB_CMD_REG, 0x80); //software reset
    ADI_Write_Reg(imu, GLOB_CMD_REG + 1, 0x80);
    imu->K_G = 40;
    imu->KG_Reciprocal = (float) (1 / imu->K_G);
    HAL_Delay(400); // wait for reboot
}

int ADIS16467_Check(ADIS16467_T *imu) {
    uint16_t imu_number;
    ADI_Read_Reg(imu, PROD_ID_REG, &imu_number, 1); //read imu ID
    imu->ProdId = imu_number;
    if (imu_number == 0x4053)
        return 1;
    else
        return 0;
}

void ADIS16467_imuInfo(ADIS16467_T *imu) {
    uint16_t data[5];
    ADI_Read_Reg(imu, RANG_MDL_REG, data, 1); //read range model
    imu->RangeModel = data[0];

    ADI_Read_Reg(imu, FIRM_REV_REG, &data[1], 1); //read firmware revision
    imu->FirmRev = data[1];

    ADI_Read_Reg(imu, FIRM_DM_REG, &data[2], 1); //read fir
    imu->Firm_Month = data[2] >> 8;
    imu->Firm_Day = data[2] & 0x00FF;

    ADI_Read_Reg(imu, FIRM_Y_REG, &data[3], 1); //read firmware revision
    imu->Firm_Year = data[3];

    ADI_Read_Reg(imu, SERIAL_NUM_REG, &data[4], 1); //read firmware revision
    imu->Serial_Num = data[4];
}

void ADIS16467_Read_Accel(ADIS16467_T *imu) {
    uint16_t AccelData[6] = {0};

    ADI_Read_Reg(imu, X_ACCL_LOW_REG, AccelData, 2);
    imu->ADIS_Accel.Accel_X_RAW = (int32_t) ((AccelData[1] << 16) & 0xFFFF0000 | AccelData[0]);
    imu->ADIS_Accel.HP_Accel_X = (float) (imu->ADIS_Accel.Accel_X_RAW * 0.01 * 1.25 * Gravity / (1 << 16));

    ADI_Read_Reg(imu, Y_ACCL_LOW_REG, &AccelData[2], 2);
    imu->ADIS_Accel.Accel_Y_RAW = (int32_t) ((AccelData[3] << 16) & 0xFFFF0000 | AccelData[2]);
    imu->ADIS_Accel.HP_Accel_Y = (float) (imu->ADIS_Accel.Accel_Y_RAW * 0.01 * 1.25 * Gravity / (1 << 16));

    ADI_Read_Reg(imu, Z_ACCL_LOW_REG, &AccelData[4], 2);
    imu->ADIS_Accel.Accel_Z_RAW = (int32_t) ((AccelData[5] << 16) & 0xFFFF0000 | AccelData[4]);
    imu->ADIS_Accel.HP_Accel_Z = (float) (imu->ADIS_Accel.Accel_Z_RAW * 1.25 * 0.01 * Gravity / (1 << 16));
}

void ADIS16467_Read_Gyro(ADIS16467_T *imu) {
    uint16_t GyroData[6] = {0};

    ADI_Read_Reg(imu, X_GYRO_LOW_REG, GyroData, 2);
    imu->ADIS_Gyro.Gyro_X_RAW = (int32_t) ((GyroData[1] << 16) & 0xFFFF0000 | GyroData[0]);
    imu->ADIS_Gyro.HP_Gyro_X = (float) (imu->ADIS_Gyro.Gyro_X_RAW * (imu->KG_Reciprocal / (1 << 16)));

    ADI_Read_Reg(imu, Y_GYRO_LOW_REG, &GyroData[2], 2);
    imu->ADIS_Gyro.Gyro_Y_RAW = (int32_t) ((GyroData[3] << 16) & 0xFFFF0000 | GyroData[2]);
    imu->ADIS_Gyro.HP_Gyro_Y = (float) (imu->ADIS_Gyro.Gyro_Y_RAW * (imu->KG_Reciprocal / (1 << 16)));

    ADI_Read_Reg(imu, Z_GYRO_LOW_REG, &GyroData[4], 2);
    imu->ADIS_Gyro.Gyro_Z_RAW = (int32_t) ((GyroData[5] << 16) & 0xFFFF0000 | GyroData[4]);
    imu->ADIS_Gyro.HP_Gyro_Z = (float) (imu->ADIS_Gyro.Gyro_Z_RAW * (imu->KG_Reciprocal / (1 << 16)));
}

void ADIS16467_Read_DeltaAngle(ADIS16467_T *imu) {
    uint16_t DeltaAngleData[6] = {0};

    ADI_Read_Reg(imu, X_DELTANG_LOW_REG, DeltaAngleData, 2);
    imu->ADIS_DeltaAng.DeltaAngle_X_RAW = (int32_t) ((DeltaAngleData[1] << 16) & 0xFFFF0000 | DeltaAngleData[0]);
    imu->ADIS_Gyro.HP_Gyro_X = (float) (imu->ADIS_DeltaAng.DeltaAngle_X_RAW * ThetaMax / (1 << 15));

    ADI_Read_Reg(imu, Y_DELTANG_LOW_REG, &DeltaAngleData[2], 2);
    imu->ADIS_DeltaAng.DeltaAngle_Y_RAW = (int32_t) ((DeltaAngleData[3] << 16) & 0xFFFF0000 | DeltaAngleData[2]);
    imu->ADIS_DeltaAng.DeltaAngle_Y = (float) (imu->ADIS_DeltaAng.DeltaAngle_Y_RAW * ThetaMax / (1 << 15));

    ADI_Read_Reg(imu, Z_GYRO_LOW_REG, &DeltaAngleData[4], 2);
    imu->ADIS_DeltaAng.DeltaAngle_Z_RAW = (int32_t) ((DeltaAngleData[5] << 16) & 0xFFFF0000 | DeltaAngleData[4]);
    imu->ADIS_DeltaAng.DeltaAngle_Z = (float) (imu->ADIS_DeltaAng.DeltaAngle_Z_RAW * ThetaMax / (1 << 15));
}

void ADIS16467_Read_DeltaVel(ADIS16467_T *imu) {
    uint16_t DeltaVelData[6] = {0};

    ADI_Read_Reg(imu, X_DELTANG_LOW_REG, DeltaVelData, 2);
    imu->ADIS_DeltaVel.DeltaVelocity_X_RAW = (int32_t) ((DeltaVelData[1] << 16) & 0xFFFF0000 | DeltaVelData[0]);
    imu->ADIS_DeltaVel.HP_DeltaVelocity_X = (float) (imu->ADIS_DeltaVel.DeltaVelocity_X_RAW * DeltaVelCof);

    ADI_Read_Reg(imu, Y_DELTANG_LOW_REG, &DeltaVelData[2], 2);
    imu->ADIS_DeltaVel.DeltaVelocity_Y_RAW = (int32_t) ((DeltaVelData[3] << 16) & 0xFFFF0000 | DeltaVelData[2]);
    imu->ADIS_DeltaVel.HP_DeltaVelocity_Y = (float) (imu->ADIS_DeltaVel.DeltaVelocity_Y_RAW * DeltaVelCof);

    ADI_Read_Reg(imu, Z_GYRO_LOW_REG, &DeltaVelData[4], 2);
    imu->ADIS_DeltaVel.DeltaVelocity_Z_RAW = (int32_t) ((DeltaVelData[5] << 16) & 0xFFFF0000 | DeltaVelData[4]);
    imu->ADIS_DeltaVel.HP_DeltaVelocity_Z = (float) (imu->ADIS_DeltaVel.DeltaVelocity_Z_RAW * DeltaVelCof);
}

void ADIS16467_Read_Temp(ADIS16467_T *imu) {
    uint16_t TempData;
    ADI_Read_Reg(imu, TEMP_OUT_REG, &TempData, 1);
    imu->Temperature = (int16_t) TempData * 0.1;
}

int8_t ADIS16467_Burst_Read(ADIS16467_T *imu) {
    HAL_GPIO_WritePin(imu->GPIOx, imu->GPIO_PIN, 0);
    int16_t parity = 0; //checksum
    HAL_StatusTypeDef stat = HAL_OK;
    uint16_t tmpRx = 0;
    uint16_t tmpTx[10] = {0};
    uint16_t data[10];
    stat |= HAL_SPI_TransmitReceive(imu->hspi, (uint8_t *) &Burst_Cmd, (uint8_t *) &tmpRx, 1,
                                    0xff); //first 16bit is useless
    if (stat != HAL_OK)
        while (1);
    sb_delay(100);
    stat |= HAL_SPI_TransmitReceive(imu->hspi, (uint8_t *) &tmpTx, (uint8_t *) data, 10,
                                    0xff); //receive data of 20 Bytes
    if (stat != HAL_OK)
        while (1);
    HAL_GPIO_WritePin(imu->GPIOx, imu->GPIO_PIN, 1);
    sb_delay(150);
    for (uint8_t i = 0; i < 9; i++)
        parity += ((data[i] >> 8) & 0x00FF) + (data[i] & 0x00FF);
    if (parity != data[9])
        return 0;

    //save data
//    imu->Ax = (int16_t) data[4] * 0.00125;
//    imu->Ay = (int16_t) data[5] * 0.00125;
//    imu->Az = (int16_t) data[6] * 0.00125;
    imu->Gx = (int16_t) data[1] / 40;
    imu->Gy = (int16_t) data[2] / 40;
    imu->Gz = (int16_t) data[3] / 40;
    imu->Temperature = (int16_t) data[7] * 0.1;
    imu->status = data[0];
    return 1;
}

// read data from register, recommend using continuous transfer
/*
@parameter:
    imu  imu imu data structure
    addr    target register address
    receive save data to this var
    num     number of register to read
*/
int8_t ADI_Read_Reg(ADIS16467_T *imu, uint8_t addr, uint16_t *receive, uint8_t num) {
    uint16_t Tx_tmp = 0, Rx_tmp = 0;

    //first frame only transmit
    Tx_tmp = (addr << 8) & 0xFF00;
    Rx_tmp = ADI_flame_TandR(imu, Tx_tmp);
    for (uint8_t i = 1; i < num; i++) {
        Tx_tmp = ((addr + 2 * i) << 8) & 0xFF00;
        Rx_tmp = ADI_flame_TandR(imu, Tx_tmp);
        receive[i - 1] = Rx_tmp;
    }
    //last frame only receive
    Tx_tmp = 0;
    receive[num - 1] = ADI_flame_TandR(imu, Tx_tmp);

    return 0;
}

/*
@parameter:
    imu  imu imu data structure
    addr    target register address
    value   the data that you want to transfer
*/
int8_t ADI_Write_Reg(ADIS16467_T *imu, uint8_t addr, uint8_t value) {
    addr |= 0x80; //写数据的掩码
    uint16_t Tx_tmp = ((addr << 8) & 0xFF00) | value;
    ADI_flame_TandR(imu, Tx_tmp);
    return 0;
}

uint16_t ADI_flame_TandR(ADIS16467_T *imu, uint16_t trans) {
    HAL_GPIO_WritePin(imu->GPIOx, imu->GPIO_PIN, 0);
    uint16_t result;
    static HAL_StatusTypeDef state;
    state = HAL_SPI_TransmitReceive(imu->hspi, (uint8_t *) &trans, (uint8_t *) &result, 1, 0xFFFF);
    if (state != HAL_OK) {
        while (1);
    }
    HAL_GPIO_WritePin(imu->GPIOx, imu->GPIO_PIN, 1);
    sb_delay(150);
    return result;
}

void sb_delay(volatile uint32_t t) {
    while (t--);
}

/*
void ADIS16467_Init(ADIS16467_t *imu)
{
    HAL_GPIO_WritePin(imu->GPIOx, imu->GPIO_PIN, 0);
    HAL_SPI_TransmitReceive(imu->hspi, RANG_MDL_REG, &(imu->rangeModel), 1, 0xFFFF);
    HAL_GPIO_WritePin(imu->GPIOx, imu->GPIO_PIN, 1);
    HAL_GPIO_WritePin(imu->GPIOx, imu->GPIO_PIN, 0);
    HAL_SPI_TransmitReceive(imu->hspi, PROD_ID_REG, &(imu->prodId), 1, 0xFFFF);
    HAL_GPIO_WritePin(imu->GPIOx, imu->GPIO_PIN, 1);
}

void ADIS16467_Read_Accel(ADIS16467_t *imu)
{
    uint16_t low, high;
    HAL_StatusTypeDef res;
    HAL_GPIO_WritePin(imu->GPIOx, imu->GPIO_PIN, 0);
    HAL_SPI_TransmitReceive(imu->hspi, X_ACCL_LOW_REG, &low, 1, 0xFFFF);
    HAL_SPI_TransmitReceive(imu->hspi, X_ACCL_OUT_REG, &high, 1, 0xFFFF);
    HAL_GPIO_WritePin(imu->GPIOx, imu->GPIO_PIN, 1);
    imu->Ax = (int32_t)((high << 16) & 0xFFFF0000 | low) * (0.00125 / (1 << 16));

    HAL_GPIO_WritePin(imu->GPIOx, imu->GPIO_PIN, 0);
    HAL_SPI_TransmitReceive(imu->hspi, Y_ACCL_LOW_REG, &low, 1, 0xFFFF);
    HAL_SPI_TransmitReceive(imu->hspi, Y_ACCL_OUT_REG, &high, 1, 0xFFFF);
    HAL_GPIO_WritePin(imu->GPIOx, imu->GPIO_PIN, 1);
    imu->Ay = (int32_t)((high << 16) & 0xFFFF0000 | low) * (0.00125 / (1 << 16));

    HAL_GPIO_WritePin(imu->GPIOx, imu->GPIO_PIN, 0);
    HAL_SPI_TransmitReceive(imu->hspi, Z_ACCL_LOW_REG, &low, 1, 0xFFFF);
    HAL_SPI_TransmitReceive(imu->hspi, Z_ACCL_OUT_REG, &high, 1, 0xFFFF);
    HAL_GPIO_WritePin(imu->GPIOx, imu->GPIO_PIN, 1);
    imu->Az = (int32_t)((high << 16) & 0xFFFF0000 | low) * (0.00125 / (1 << 16));
}

void ADIS16467_Read_Gyro(ADIS16467_t *imu)
{
    double defScale = 1.0;
    defScale = (model == 0x3) ? (0.00625 / (1 << 16)) : (model == 0x7) ? (0.025 / (1 << 16)) : (model == 0xF)   ? (0.1 / (1 << 16)) : defScale;

    uint16_t low, high;
    HAL_StatusTypeDef res;
    HAL_GPIO_WritePin(imu->GPIOx, imu->GPIO_PIN, 0);
    HAL_SPI_TransmitReceive(imu->hspi, X_ACCL_LOW_REG, &low, 1, 0xFFFF);
    HAL_SPI_TransmitReceive(imu->hspi, X_ACCL_OUT_REG, &high, 1, 0xFFFF);
    HAL_GPIO_WritePin(imu->GPIOx, imu->GPIO_PIN, 1);
    imu->Ax = (int32_t)((high << 16) & 0xFFFF0000 | low) * defScale;

    HAL_GPIO_WritePin(imu->GPIOx, imu->GPIO_PIN, 0);
    HAL_SPI_TransmitReceive(imu->hspi, Y_ACCL_LOW_REG, &low, 1, 0xFFFF);
    HAL_SPI_TransmitReceive(imu->hspi, Y_ACCL_OUT_REG, &high, 1, 0xFFFF);
    HAL_GPIO_WritePin(imu->GPIOx, imu->GPIO_PIN, 1);
    imu->Ay = (int32_t)((high << 16) & 0xFFFF0000 | low) * defScale;

    HAL_GPIO_WritePin(imu->GPIOx, imu->GPIO_PIN, 0);
    HAL_SPI_TransmitReceive(imu->hspi, Z_ACCL_LOW_REG, &low, 1, 0xFFFF);
    HAL_SPI_TransmitReceive(imu->hspi, Z_ACCL_OUT_REG, &high, 1, 0xFFFF);
    HAL_GPIO_WritePin(imu->GPIOx, imu->GPIO_PIN, 1);
    imu->Az = (int32_t)((high << 16) & 0xFFFF0000 | low) * defScale;
}

void ADIS16467_Read_Temp(ADIS16467_t *imu)
{
    HAL_GPIO_WritePin(imu->GPIOx, imu->GPIO_PIN, 0);
    HAL_SPI_TransmitReceive(imu->hspi, TEMP_OUT_REG, &(imu->prodId), 1, 0xFFFF);
    HAL_GPIO_WritePin(imu->GPIOx, imu->GPIO_PIN, 1);
}
*/