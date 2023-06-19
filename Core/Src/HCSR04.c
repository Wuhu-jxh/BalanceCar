/**
  ************************************* Copyright ******************************
  *

  *                 (C) Copyright 2023,--,China, CUIT.
  *                            All Rights Reserved
  *                     By(JCML)
  * FileName   : HCSR04.c
  * Version    : v1.0
  * Author     : JCML
  * Date       : 2023-02-10
  * Description: HCSR04���ģ�飬 ��������ʹ����һ��timer��Ҫ10΢�����time��1
******************************************************************************
 */
#define Timer	htim3
#include "main.h"
#include "tim.h"
#include "HCSR04.h"

uint64_t time=0;			//����������������ʱ
uint64_t time_end=0;		//�����������洢�ز��ź�ʱ��
uint32_t Distance;

/***********************************************************
*@���� 	:HC_SR04_Init
*@����	:��ʼ�� ����TIM�ж�
*@����	:��
*@����ֵ	:��
*@����	:JCML
*@����	:2023-02-10
***********************************************************/

void HC_SR04_Init(void)
{
//  HAL_TIM_Base_Start_IT(&Timer);��ǰ������Motor.c�Ѿ��������˴������ظ���
  HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_RESET);						//����͵�ƽ
  Delay_us(15);											//��ʱ15΢��
}

/***********************************************************
*@���� 	:GetDistance
*@����	:��ȡ����
*@����	:��
*@����ֵ	:Distance��ʼֵ
*@����	:JCML
*@����	:2023-02-10
***********************************************************/

int16_t GetDistance(void)									//��ಢ���ص�λΪ���׵ľ�����
{
  HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_SET);//����ߵ�ƽ
  Delay_us(15);										//��ʱ15΢��
  HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_RESET);//����͵�ƽ
  while (HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == 0);//�ȴ��͵�ƽ������תΪ�ߵ�ƽ��ʼ��ʱ
  time=0;												//��ʱ����
  while(HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == 1);		//�ȴ��ߵ�ƽ������תΪ�͵�ƽ������ʱ
  time_end=time;										//��¼����ʱ��ʱ��
  if(time_end/100<38)									//�ж��Ƿ�С��38���룬����38����ľ��ǳ�ʱ��ֱ�ӵ������淵��0
  {
    Distance=(time_end*346)/2;						//������룬25��C�����е�����Ϊ346m/s
  }
  return Distance;									//���ز����
}

/***********************************************************
*@���� 	:ScanDistance_mm
*@����	:ת���õ��ľ���ֵ
*@����	:��
*@����ֵ	:��
*@����	:JCML
*@����	:2023-02-10
***********************************************************/

float ScanDistance_mm(void)//���ص�λΪ���׵ľ�����
{
  return GetDistance() / 100.0;
}

float ScanDistance_cm(void)//���ص�λΪ���׵ľ�����
{
  return GetDistance() / 1000.0;
}
float ScanDistance_m(void)//���ص�λΪ�׵ľ�����
{
  return GetDistance() / 100000.0;
}

void HCSR04_Callback(void)//�ⲿʱ����ú���
{
  time++;
}

/***********************************************************
*@���� 	:HAL_TIM_PeriodElapsedCallback
*@����	:�жϻص������˴����ڼ�ʱ������ռ�ÿ������ⲿ�Լ�
*@����	:��
*@����ֵ	:��
*@����	:JCML
*@����	:2023-02-10
***********************************************************/
//
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)			//�����жϺ�����������ʱ��ÿ10΢�����time��1
//{
//  if (htim == &htim3)		//��ȡTIM3��ʱ���ĸ����жϱ�־λ
//  {
//    HCSR04_Callback();
//  }
//}


/**
  * @brief  ΢�뼶��ʱ
  * @param  xus ��ʱʱ������Χ��0~233015
  * @retval ��
  */
void Delay_us(uint32_t xus)
{
  SysTick->LOAD = 72 * xus;				//���ö�ʱ����װֵ
  SysTick->VAL = 0x00;					//��յ�ǰ����ֵ
  SysTick->CTRL = 0x00000005;				//����ʱ��ԴΪHCLK��������ʱ��
  while(!(SysTick->CTRL & 0x00010000));	//�ȴ�������0
  SysTick->CTRL = 0x00000004;				//�رն�ʱ��
}

/**
  * @brief  ���뼶��ʱ
  * @param  xms ��ʱʱ������Χ��0~4294967295
  * @retval ��
  */
void Delay_ms(uint32_t xms)
{
  while(xms--)
  {
    Delay_us(1000);
  }
}

/**
  * @brief  �뼶��ʱ
  * @param  xs ��ʱʱ������Χ��0~4294967295
  * @retval ��
  */
void Delay_s(uint32_t xs)
{
  while(xs--)
  {
    Delay_ms(1000);
  }
}
