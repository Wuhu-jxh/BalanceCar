//
// Created by 神奇bug在哪里 on 6/20/23.
//

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
    float P; //估计误差
    float K; //卡尔曼增益
}Kalman;
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
 */
void kalman_fliter_init(Kalman * kalman,float Q,float R);
/**
 * @brief 卡尔曼滤波
 * @param kalman 滤波器结构体
 * @param in 输入
 * @param out 输出
 */
#endif //BALANCECAR_FLITER_H
