//
// Created by mulai on 2023/6/18.
//

#ifndef BALANCECAR_V1_0_MPU6050_H
#define BALANCECAR_V1_0_MPU6050_H
#include "main.h"

#define SMPLRT_DIV   0x19  // 采样率分频，典型值：0x07(125Hz) */
#define CONFIG       0x1A  // 低通滤波频率，典型值：0x06(5Hz) */
#define GYRO_CONFIG  0x1B  // 陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s) */
#define ACCEL_CONFIG 0x1C  // 加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz) */

#define ACCEL_XOUT_H 0x43  // 存储最近的X轴、Y轴、Z轴加速度感应器的测量值 */
#define ACCEL_XOUT_L 0x44
#define ACCEL_YOUT_H 0x45
#define ACCEL_YOUT_L 0x46
#define ACCEL_ZOUT_H 0x47
#define ACCEL_ZOUT_L 0x48

#define TEMP_OUT_H   0x41  // 存储的最近温度传感器的测量值 */
#define TEMP_OUT_L   0x42

#define GYRO_XOUT_H  0x3B  // 存储最近的X轴、Y轴、Z轴陀螺仪感应器的测量值 */
#define GYRO_XOUT_L  0x3C
#define GYRO_YOUT_H  0x3D
#define GYRO_YOUT_L  0x3E
#define GYRO_ZOUT_H  0x3F
#define GYRO_ZOUT_L  0x40

#define PWR_MGMT_1   0x6B   // 电源管理，典型值：0x00(正常启用) */
#define	PWR_MGMT_2		0x6C	//电源管理
#define WHO_AM_I     0x75 	// IIC地址寄存器(默认数值0x68，只读) */
#define MPU6050_ADDR 0xD0	// MPU6050手册上的地址，这里也可以使用serch函数去搜索

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
}_MPU6050_DATA;

extern _MPU6050_DATA MPU6050_Data;

int16_t I2C_Serch(void);
int8_t MPU6050_Init(int16_t Addr);

int8_t I2C_Read(uint16_t DevAddr, uint16_t MemAddr, uint8_t *oData, uint8_t Datalen);
int8_t I2C_Write(uint16_t DevAddr, uint16_t MemAddr, uint8_t *iData, uint8_t Datalen);

void MPU6050_Read_Accel(void);
void MPU6050_Read_Gyro(void);
void MPU6050_Read_Temp(void);

#endif //BALANCECAR_V1_0_MPU6050_H
