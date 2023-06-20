//
// Created by 神奇bug在哪里 on 6/20/23.
//

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
/**
 * @brief 三个pid一起计算
 * @param pid1 --
 */
result pid_cycle(PID *pid1, PID *pid2, PID *pid3, float target1, float target2, float target3,
          float actual1, float actual2, float actual3)
{
    float result1=0, result2=0, result3=0;
    if (CORE_PID_SPEED_EN) {
        result1 = pid_calc(pid1, target1, actual1);
    }
    LIMIT(result1, 100);
    if (CORE_PID_ANGLE_EN) {
        result2 = pid_calc(pid2, target2, actual2);
    }
    LIMIT(result2,100);
    if (CORE_PID_POSITION_EN) {
        result3 = pid_calc(pid3, target3, actual3);
    }
    LIMIT(result3,100);
    float left = result1+result2+result3;
    float right = result1+result2-result3;
    LIMIT(left,100);
    LIMIT(right,100);
    result res = {left,right};
    return res;
}