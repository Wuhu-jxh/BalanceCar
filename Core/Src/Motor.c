//
// Created by mulai on 2023/6/18.
//
#include "tim.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_gpio.h"
#include "Motor.h"
/*��������PWM����Timer �Լ���ת�������Ľӿ�Timer�Լ�����

 */
#define	Motor_Timer1	htim1
#define Encoder_Timer1 htim2
#define Encoder_Timer2 htim3
#define Encoder_TimeCounter htim4//����Encoder_TimeCounter�����ڼ����ٶȵ�
#define	Motor1	TIM_CHANNEL_1
#define	Motor2	TIM_CHANNEL_4

_Motor _motor;


/***********************************************************
*@���� :Motor_Init
*@����	:Motor_Init����������PWM�ĳ�ʼ���Լ���ת��������ʱ���ĳ�ʼ��
*@����	:��
*@����ֵ	:��
*@����	:JCML
*@����	:2023-04-06
***********************************************************/

void Motor_Init(void)
{
  HAL_TIM_Base_Start_IT(&Motor_Timer1);
  HAL_TIM_PWM_Start(&Motor_Timer1, Motor1);
  HAL_TIM_PWM_Start(&Motor_Timer1, Motor2);
  HAL_TIM_Encoder_Start(&Encoder_Timer1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&Encoder_Timer2, TIM_CHANNEL_ALL);//������ʱ������
  HAL_TIM_Base_Start_IT(&Encoder_TimeCounter);//�������ڻ�ȡ�����Ķ�ʱ��
}

/***********************************************************
*@���� :W1_Control  &	W2_Control
*@����	:����1��2����
*@����	:Speed���������� ��Χ-100~100
*@����ֵ	:��
*@����	:JCML
*@����	:2023-04-06
***********************************************************/



void W1_Control(int32_t Speed)
{
  if (Speed >= 0)
  {
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SetCompare(&Motor_Timer1, Motor1, Speed);}//��ֵ
  else
  {
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    __HAL_TIM_SetCompare(&Motor_Timer1, Motor1,-Speed);}
}
void W2_Control(int32_t Speed)
{
  if (Speed >= 0)
  {
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SetCompare(&Motor_Timer1, Motor2, Speed);}//��ֵ
  else
  {
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
    __HAL_TIM_SetCompare(&Motor_Timer1, Motor2, -Speed);}
}

/***********************************************************
*@���� :GetSpeed
*@����	:��ȡ��ת���������ٶ�ֵ
*@����	:_Motor�ṹ�壬�ú�������whileѭ����ȡ
*@����ֵ	:��
*@����	:JCML
*@����	:2023-06-18
***********************************************************/
void GetSpeed(_Motor *speed)
{
  speed->M1_ActualSpeed = -(float )_motor.Temp_W1;
  speed->M2_ActualSpeed = (float )_motor.Temp_W2;//��ȡ��ǰ���ӵ��ٶ�,�˴�ֱ�ӽ��ٶ�תΪfloat�ͣ����ں�������˲�
}


void Encode_CallBack(_Motor *speed)
{
  _motor.Temp_W1 = ((short)__HAL_TIM_GET_COUNTER(&Encoder_Timer1));//��ȡM1����ת����,�˴�ȡ���ǻ���������һ�ܼ�����һ
  _motor.Temp_W2 = ((short)__HAL_TIM_GET_COUNTER(&Encoder_Timer2));//��ȡM2����ת����
  __HAL_TIM_SET_COUNTER(&Encoder_Timer1, 0);
  __HAL_TIM_SET_COUNTER(&Encoder_Timer2, 0);//����������������
}
/***********************************************************
*@���� :HAL_TIM_PeriodElapsedCallback
*@����	:��ʱ���Ļص�����,����г�ͻ�뽫�����main.c�У�������Encode_CallBack���¼���ֵ
*@����	:TIM_HandleTypeDef
*@����ֵ	:��
*@����	:JCML
*@����	:2023-06-18
***********************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//ÿ100ms��һ���ж�
{
  if(htim==(&Encoder_TimeCounter))
  {
    _motor.Temp_W1 = ((short)__HAL_TIM_GET_COUNTER(&Encoder_Timer1));//��ȡM1����ת����,�˴�ȡ���ǻ���������һ�ܼ�����һ
    _motor.Temp_W2 = ((short)__HAL_TIM_GET_COUNTER(&Encoder_Timer2));//��ȡM2����ת����
    __HAL_TIM_SET_COUNTER(&Encoder_Timer1, 0);
    __HAL_TIM_SET_COUNTER(&Encoder_Timer2, 0);//����������������
  }
}