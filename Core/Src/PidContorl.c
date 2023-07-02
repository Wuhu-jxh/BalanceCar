//
// Created by 神奇bug在哪里 on 6/20/23.
//

#include <math.h>
#include "PidContorl.h"
#include "settings.h"


/**
 * @brief 初始化pid参数
 * @param pid
 * @param Kp
 * @param Ki
 * @param Kd
 */
void pid_init(PID *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->LastError = 0;
    pid->Integral = 0;
    pid->Derivative = 0;
    pid->Output = 0;
    pid->Target = 0;
    pid->Actual = 0;
}
float pid_calc(PID *pid, float target, float actual)
{
    pid->Target = target;
    pid->Actual = actual;
    pid->Error = pid->Target - pid->Actual;
    pid->Integral += pid->Error;
    pid->Derivative = pid->Error - pid->LastError;
    pid->Output = pid->Kp * pid->Error + pid->Ki * pid->Integral + pid->Kd * pid->Derivative;
    pid->LastError = pid->Error;
    return pid->Output;
}

Angle offsetAngleCal(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ)
{
    Angle  ang;
    ang.pitch = atan2f(accY, accZ) * 180 / PI;
    ang.roll = atan2f(-accX, accZ) * 180 / PI;
    ang.yaw = atan2f(accX, accY) * 180 / PI;
    double gyroXangle = gyroX * 0.0000611;
    double gyroYangle = gyroY * 0.0000611;
    double gyroZangle = gyroZ * 0.0000611;
    ang.x = 0.98 * (gyroXangle + gyroX) + 0.02 * ang.pitch;
    ang.y = 0.98 * (gyroYangle + gyroY) + 0.02 * ang.roll;
    ang.z = 0.98 * (gyroZangle + gyroZ) + 0.02 * ang.yaw;
    return ang;
}
/**
 * @brief 直立环PD控制器:Kp*Ek+Kd*Ek_D
 * @param Med 机械中值
 * @param Angle 真实偏差角度
 * @param gyro_Y 真实角速度
 * @return
 */
float PID_POSITION(float Med, float Angle, float gyro_Y)
{

    return POSITION_PID_KP*(Angle-Med)+POSITION_PID_KD*(gyro_Y-0);
}

/**
 * @brief 速度环PI控制器:Kp*Ek+Ki*Ek_S(Ek_S：偏差的积分)
 * @param Target 速度的目标值
 * @param encoder_left 编码器_左
 * @param encoder_right 编码器_右
 * @return
 */
float PID_SPEED(float Target, float encoder_left, float encoder_right)
{
    static float PWM_out,Encoder_Err,Encoder_S,EnC_Err_Lowout,EnC_Err_Lowout_last;
    float a=0.7f;
    // 1.计算速度偏差
    Encoder_Err = ((encoder_left+encoder_right)-Target);
    // 2.对速度偏差进行低通滤波
    // low_out = (1-a)*Ek+a*low_out_last
    EnC_Err_Lowout = (1-a)*Encoder_Err + a*EnC_Err_Lowout_last; // 使得波形更加平滑，滤除高频干扰，放置速度突变
    EnC_Err_Lowout_last = EnC_Err_Lowout;   // 防止速度过大影响直立环的正常工作
    // 3.对速度偏差积分出位移
    Encoder_S+=EnC_Err_Lowout;
    // 4.积分限幅
    LIMIT(Encoder_S,1000);
    // 5.速度环控制输出
    PWM_out = SPEED_PID_KP*EnC_Err_Lowout+SPEED_PID_KI*Encoder_S;

    return PWM_out;
}

/**
 * @brief 转向环PD控制器:Kp*Ek
 * @param angleOffset
 * @return
 */
float PID_TURN(int angleOffset)
{
    int PWM_out;
    PWM_out = ANGLE_PID_KP * angleOffset;
    return PWM_out;
}

result PID_Cycal(_MPU6050_DATA mpuData,float encoder_L,float encoder_R,float targetSpeed,float targetAngle,Angle angleData)
{
    float speedOut = PID_SPEED(targetSpeed,encoder_L,encoder_R);
    float positionOut = PID_POSITION(Med_Value,angleData.roll,mpuData.Gyro_Y);
    float turnOut = PID_TURN(mpuData.Gyro_Z-targetAngle);
    result res;
    res.Left = speedOut + positionOut -turnOut;
    res.Right = speedOut + positionOut +turnOut;
    return res;
}