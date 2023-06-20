//
// Created by 神奇bug在哪里 on 6/20/23.
//

#include "fliter.h"
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
