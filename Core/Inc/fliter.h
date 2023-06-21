//
// Created by 神奇bug在哪里 on 6/20/23.
//

#include "MPU6050.h"

#ifndef BALANCECAR_FLITER_H
#define BALANCECAR_FLITER_H
typedef struct
{
    float last_value;
    float alpha; //滤波系数
}Lag;
typedef struct {
    float last_value;
    float Q; //过程噪声
    float R; //测量噪声
    float Last_P; //估计误差_上一个
    float Now_P; //估计误差_下一个
    float K; //卡尔曼增益

}Kalman;
/**滚动均值滤波**/
typedef struct {
    float *buf;
    int size;
    int index;
    float sum;
}RoundFliter;
/**
 * @brief 一阶滞后滤波初始化
 * @param lag 滤波器结构体
 * @param alpha 滤波系数
 */
void lag_fliter_init(Lag * lag,float alpha);
/**
 * @brief 一阶滞后滤波
 * @param lag 滤波器结构体
 * @param in 输入
 * @param out 输出
 *
 */
void lag_fliter_cal(Lag * lag,float in, float *out);

/**
 * @brief 卡尔曼滤波初始化
 * @param kalman 滤波器结构体
 * @param Q 过程噪声
 * @param R 测量噪声
 * @param K 卡尔曼增益
 */
void kalman_fliter_init(Kalman *kalman, float Q, float R, float K);
/**
 * @brief 卡尔曼滤波
 * @param kalman 滤波器结构体
 * @param in 输入
 * @param out 输出
 */
void kalman_fliter_cal(Kalman * kalman,float in, float *out);

void round_fliter_init(RoundFliter *roundFliter, int size);
void round_fliter_cal(RoundFliter *roundFliter, float in, float *out);

void any_fliter_run(void * anyStruct,float in,float *out);
void MPU6050_filter(_MPU6050_DATA *mpu6050_data_in, _MPU6050_DATA *mpu6050_data_out);
#endif //BALANCECAR_FLITER_H
