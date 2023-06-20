//
// Created by 神奇bug在哪里 on 6/20/23.
//

#ifndef BALANCECAR_PIDCONTORL_H
#define BALANCECAR_PIDCONTORL_H
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
    float Left;
    float Right;
}result;
void pid_init(PID *pid, float Kp, float Ki, float Kd);
float pid_calc(PID *pid, float target, float actual);
result
pid_cycle(PID *pid1, PID *pid2, PID *pid3, float target1, float target2, float target3, float actual1, float actual2,
          float actual3);
#endif //BALANCECAR_PIDCONTORL_H
