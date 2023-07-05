//
// Created by mulai on 2023/6/18.
//

#ifndef BALANCECAR_V1_0_MPU6050_H
#define BALANCECAR_V1_0_MPU6050_H
#include "main.h"
//#include <stdint-gcc.h>
#include "settings.h"
//ԭ�ȵ����ж�������ƶ���settings.h��

typedef struct{
    // ���ٶ�
    float Accel_X;
    float Accel_Y;
    float Accel_Z;
    // �Ƕ�
    float Gyro_X;
    float Gyro_Y;
    float Gyro_Z;
    // �¶�
    float Temp;
    // DMP
    float DMP_Pitch;
    float DMP_Roll;
    float DMP_Yaw;
    //��Ԫ��
    float Quat_W;
    float Quat_X;
    float Quat_Y;
    float Quat_Z;
    float DMP_Temp;

    unsigned long Time;

    short Sensors;
    unsigned char More;


}_MPU6050_DATA;

extern _MPU6050_DATA MPU6050_Data;


/**��ֲ�����м�ṹ��**/
struct platform_data_s {
    signed char orientation[9];
};

int16_t I2C_Serch(void);
int8_t MPU6050_Init(int16_t Addr);

int8_t I2C_Read(uint16_t DevAddr, uint16_t MemAddr, uint8_t *oData, uint8_t Datalen);
int8_t I2C_Write(uint16_t DevAddr, uint16_t MemAddr, uint8_t *iData, uint8_t Datalen);

int8_t I2C_Readn(unsigned char slave_addr, unsigned char reg_addr, unsigned short len,
                 unsigned char *data_ptr);
int8_t I2C_Writen(unsigned char slave_addr, unsigned char reg_addr, unsigned short len,
                  unsigned char *data_ptr);

void MPU6050_Read_Accel(void);
void MPU6050_Read_Gyro(void);
void MPU6050_Read_Temp(void);

void MPU6050_Read_DMP(void);
#endif //BALANCECAR_V1_0_MPU6050_H
