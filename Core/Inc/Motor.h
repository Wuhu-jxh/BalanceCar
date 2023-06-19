//
// Created by mulai on 2023/6/18.
//

#ifndef BALANCECAR_V1_0_MOTOR_H
#define BALANCECAR_V1_0_MOTOR_H
#include "main.h"
//定义目的车轮
#define W1 1
#define W2 2

typedef struct
{
    short Temp_W1;//暂存值，用于调试滤波
    short Temp_W2;

    float M1_ActualSpeed;
    float M2_ActualSpeed;//接收来自电机的原始速度

    float Error;
}_Motor;
extern _Motor Motor;//此处定义了一个全局变量无需再去定义main.c新的变量



void Motor_Init(void);
void W1_Control(int32_t Speed);
void W2_Control(int32_t Speed);
void W3_Control(int32_t Speed);
void W4_Control(int32_t Speed);
void Servo1_Set(uint16_t Angle);
void GetSpeed(_Motor *speed);
void Encode_CallBack(_Motor *speed);
#endif //BALANCECAR_V1_0_MOTOR_H
