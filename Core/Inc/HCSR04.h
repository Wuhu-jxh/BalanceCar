//
// Created by mulai on 2023/6/19.
//

#ifndef BALANCECAR_V1_0_HCSR04_H
#define BALANCECAR_V1_0_HCSR04_H
extern uint64_t time;

void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s(uint32_t s);//��ʱ����

void HC_SR04_Init(void);
void HCSR04_Callback(void);//�ⲿʱ����ú���
int16_t GetDistance(void);
float ScanDistance_mm(void);//���ص�λΪ���׵ľ�����
float ScanDistance_cm(void);//���ص�λΪ���׵ľ�����
float ScanDistance_m(void);//���ص�λΪ�׵ľ�����
#endif //BALANCECAR_V1_0_HCSR04_H
