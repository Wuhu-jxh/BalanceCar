//
// Created by 神奇bug在哪里 on 6/20/23.
//

#ifndef BALANCECAR_PIDCONTORL_H
#define BALANCECAR_PIDCONTORL_H

#include "MPU6050.h"

#define LIMIT(x,y) if(x>y)x=y;else if(x<-y)x=-y;else x=x
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float Error;
    float LastError;
    float Integral;
    float Derivative;
    float Output;
    float Target;
    float Actual;
} PID;
typedef struct
{
    long Left;
    long Right;
}result;
typedef struct
{
    float x;
    float y;
    float z;
    float pitch;
    float roll;
    float yaw;
}Angle;
void pid_init(PID *pid, float Kp, float Ki, float Kd);
float pid_calc(PID *pid, float target, float actual);
///以上PID计算方法已弃用
result PID_Cycal(_MPU6050_DATA mpuData,float encoder_L,float encoder_R,float targetSpeed,float targetAngle,Angle angleData);
Angle offsetAngleCal(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ);
#endif //BALANCECAR_PIDCONTORL_H
