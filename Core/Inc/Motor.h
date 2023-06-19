//
// Created by mulai on 2023/6/18.
//

#ifndef BALANCECAR_V1_0_MOTOR_H
#define BALANCECAR_V1_0_MOTOR_H
#include "main.h"
//����Ŀ�ĳ���
#define W1 1
#define W2 2

typedef struct
{
    short Temp_W1;//�ݴ�ֵ�����ڵ����˲�
    short Temp_W2;

    float M1_ActualSpeed;
    float M2_ActualSpeed;//�������Ե����ԭʼ�ٶ�

    float Error;
}_Motor;


void Motor_Init(void);
void W1_Control(int32_t Speed);
void W2_Control(int32_t Speed);
void W3_Control(int32_t Speed);
void W4_Control(int32_t Speed);
void Servo1_Set(uint16_t Angle);
void GetSpeed(_Motor *speed);
void Encode_CallBack(_Motor *speed);
#endif //BALANCECAR_V1_0_MOTOR_H
