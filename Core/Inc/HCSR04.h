//
// Created by mulai on 2023/6/19.
//

#ifndef BALANCECAR_V1_0_HCSR04_H
#define BALANCECAR_V1_0_HCSR04_H
extern uint64_t time;

void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s(uint32_t s);//延时函数

void HC_SR04_Init(void);
void HCSR04_Callback(void);//外部时间调用函数
int16_t GetDistance(void);
float ScanDistance_mm(void);//返回单位为毫米的距离结果
float ScanDistance_cm(void);//返回单位为厘米的距离结果
float ScanDistance_m(void);//返回单位为米的距离结果
#endif //BALANCECAR_V1_0_HCSR04_H
