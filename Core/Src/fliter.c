//
// Created by 神奇bug在哪里 on 6/20/23.
//
#include "settings.h"
#include "fliter.h"
#include "stdlib.h"
#include <assert.h>
#if CORE_PID_FILTER_MODE == 1  || CORE_PID_FILTER_MODE == 3
extern Lag MPU_accX;
extern Lag MPU_accY;
extern Lag MPU_accZ;
extern Lag MPU_gyroX;
extern Lag MPU_gyroY;
extern Lag MPU_gyroZ;
#elif CORE_PID_FILTER_MODE == 2
extern KalmanMPU_accX;
extern Kalman MPU_accY;
extern Kalman MPU_accZ;
extern Kalman MPU_gyroX;
extern Kalman MPU_gyroY;
extern Kalman MPU_gyroZ;
#elif CORE_PID_FILTER_MODE == 4
extern RoundFliter MPU_accX;
extern RoundFliter MPU_accY;
extern RoundFliter MPU_accZ;
extern RoundFliter MPU_gyroX;
extern RoundFliter MPU_gyroY;
extern RoundFliter MPU_gyroZ;
#endif
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

void any_fliter_run(void * anyStruct,float in,float *out)
{
#if CORE_PID_FILTER_MODE == 1  || CORE_PID_FILTER_MODE == 3
    lag_fliter_cal((Lag *)anyStruct,in,out);
#elif CORE_PID_FILTER_MODE == 2
    kalman_fliter_cal((Kalman * )anyStruct,in,out);
#elif CORE_PID_FILTER_MODE == 4
    round_fliter_cal((RoundFliter *)anyStruct,in,out);
#endif
}
void MPU6050_filter(_MPU6050_DATA *mpu6050_data_in, _MPU6050_DATA *mpu6050_data_out)
{
    assert(mpu6050_data_in!=NULL);
    assert(mpu6050_data_out!=NULL);
#if CORE_PID_FILTER_MODE !=0
    any_fliter_run(&MPU_accX, mpu6050_data_in->Accel_X, &mpu6050_data_out->Accel_X);
    any_fliter_run(&MPU_accY, mpu6050_data_in->Accel_Y, &mpu6050_data_out->Accel_Y);
    any_fliter_run(&MPU_accZ, mpu6050_data_in->Accel_Z, &mpu6050_data_out->Accel_Z);
    any_fliter_run(&MPU_gyroX, mpu6050_data_in->Gyro_X, &mpu6050_data_out->Gyro_X);
    any_fliter_run(&MPU_gyroY, mpu6050_data_in->Gyro_Y, &mpu6050_data_out->Gyro_Y);
    any_fliter_run(&MPU_gyroZ, mpu6050_data_in->Gyro_Z, &mpu6050_data_out->Gyro_Z);
#else
    mpu6050_data_out->Accel_X = mpu6050_data_in->Accel_X;
    mpu6050_data_out->Accel_Y = mpu6050_data_in->Accel_Y;
    mpu6050_data_out->Accel_Z = mpu6050_data_in->Accel_Z;
    mpu6050_data_out->Gyro_X = mpu6050_data_in->Gyro_X;
    mpu6050_data_out->Gyro_Y = mpu6050_data_in->Gyro_Y;
    mpu6050_data_out->Gyro_Z = mpu6050_data_in->Gyro_Z;
#endif
}