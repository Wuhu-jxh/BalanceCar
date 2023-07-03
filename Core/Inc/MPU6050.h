//
// Created by mulai on 2023/6/18.
//

#ifndef BALANCECAR_V1_0_MPU6050_H
#define BALANCECAR_V1_0_MPU6050_H
#include "main.h"
//#include <stdint-gcc.h>
#include "settings.h"
//原先的所有定义均已移动至settings.h中

typedef struct{
    // 角速度
    float Accel_X;
    float Accel_Y;
    float Accel_Z;
    // 角度
    float Gyro_X;
    float Gyro_Y;
    float Gyro_Z;
    // 温度
    float Temp;
    // DMP
    float DMP_Pitch;
    float DMP_Roll;
    float DMP_Yaw;
}_MPU6050_DATA;

extern _MPU6050_DATA MPU6050_Data;


int16_t I2C_Serch(void);
int8_t MPU6050_Init(int16_t Addr);

int8_t I2C_Read(uint16_t DevAddr, uint16_t MemAddr, uint8_t *oData, uint8_t Datalen);
int8_t I2C_Write(uint16_t DevAddr, uint16_t MemAddr, uint8_t *iData, uint8_t Datalen);

void MPU6050_Read_Accel(void);
void MPU6050_Read_Gyro(void);
void MPU6050_Read_Temp(void);
void MPU6050_Read_DMP(void);
#endif //BALANCECAR_V1_0_MPU6050_H
