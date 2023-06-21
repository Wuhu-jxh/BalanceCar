//
// Created by 神奇bug在哪里 on 6/20/23.
//

#include "fliter.h"
#include "stdlib.h"
void lag_fliter_init(Lag * lag,float alpha)
{
    lag->alpha = alpha;
    lag->last_value = 0;
}
void lag_fliter_cal(Lag * lag,float in, float *out)
{
    *out = lag->alpha * in + (1 - lag->alpha) * lag->last_value;
    lag->last_value = *out;
}

void kalman_fliter_init(Kalman *kalman, float Q, float R, float K)
{
    kalman->Q = Q;
    kalman->R = R;
    kalman->Last_P = 0;
    kalman->Now_P = 0;
    kalman->K = K;
    kalman->last_value = 0;
}
void kalman_fliter_cal(Kalman * kalman,float in, float *out)
{
    kalman->Now_P = kalman->Last_P + kalman->Q;
    kalman->K = kalman->Now_P / (kalman->Now_P + kalman->R);
    *out = kalman->last_value + kalman->K * (in - kalman->last_value);
    kalman->Last_P = (1 - kalman->K) * kalman->Now_P;
    kalman->last_value = *out;
}
void round_fliter_init(RoundFliter *roundFliter, int size)
{
    roundFliter->buf = (float *)malloc(sizeof(float) * size);
    roundFliter->size = size;
    roundFliter->index = 0;
    roundFliter->sum = 0;
}
void round_fliter_cal(RoundFliter *roundFliter, float in, float *out)
{
    roundFliter->sum -= roundFliter->buf[roundFliter->index];
    roundFliter->buf[roundFliter->index] = in;
    roundFliter->sum += roundFliter->buf[roundFliter->index];
    roundFliter->index++;
    if (roundFliter->index >= roundFliter->size)
    {
        roundFliter->index = 0;
    }
    *out = roundFliter->sum / roundFliter->size;
}
