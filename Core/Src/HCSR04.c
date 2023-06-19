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
  * Description: HCSR04测距模块， 本程序中使用了一个timer需要10微秒变量time加1
******************************************************************************
 */
#define Timer	htim3
#include "main.h"
#include "tim.h"
#include "HCSR04.h"

uint64_t time=0;			//声明变量，用来计时
uint64_t time_end=0;		//声明变量，存储回波信号时间
uint32_t Distance;

/***********************************************************
*@名称 	:HC_SR04_Init
*@描述	:初始化 开启TIM中断
*@参数	:无
*@返回值	:无
*@作者	:JCML
*@日期	:2023-02-10
***********************************************************/

void HC_SR04_Init(void)
{
//  HAL_TIM_Base_Start_IT(&Timer);当前工程中Motor.c已经开启，此处不在重复打开
  HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_RESET);						//输出低电平
  Delay_us(15);											//延时15微秒
}

/***********************************************************
*@名称 	:GetDistance
*@描述	:获取距离
*@参数	:无
*@返回值	:Distance初始值
*@作者	:JCML
*@日期	:2023-02-10
***********************************************************/

int16_t GetDistance(void)									//测距并返回单位为毫米的距离结果
{
  HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_SET);//输出高电平
  Delay_us(15);										//延时15微秒
  HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_RESET);//输出低电平
  while (HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == 0);//等待低电平结束，转为高电平开始计时
  time=0;												//计时清零
  while(HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == 1);		//等待高电平结束，转为低电平结束计时
  time_end=time;										//记录结束时的时间
  if(time_end/100<38)									//判断是否小于38毫秒，大于38毫秒的就是超时，直接调到下面返回0
  {
    Distance=(time_end*346)/2;						//计算距离，25°C空气中的音速为346m/s
  }
  return Distance;									//返回测距结果
}

/***********************************************************
*@名称 	:ScanDistance_mm
*@描述	:转换得到的距离值
*@参数	:无
*@返回值	:无
*@作者	:JCML
*@日期	:2023-02-10
***********************************************************/

float ScanDistance_mm(void)//返回单位为毫米的距离结果
{
  return GetDistance() / 100.0;
}

float ScanDistance_cm(void)//返回单位为厘米的距离结果
{
  return GetDistance() / 1000.0;
}
float ScanDistance_m(void)//返回单位为米的距离结果
{
  return GetDistance() / 100000.0;
}

void HCSR04_Callback(void)//外部时间调用函数
{
  time++;
}

/***********************************************************
*@名称 	:HAL_TIM_PeriodElapsedCallback
*@描述	:中断回调函数此处用于计时，如有占用可以在外部自加
*@参数	:无
*@返回值	:无
*@作者	:JCML
*@日期	:2023-02-10
***********************************************************/
//
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)			//更新中断函数，用来计时，每10微秒变量time加1
//{
//  if (htim == &htim3)		//获取TIM3定时器的更新中断标志位
//  {
//    HCSR04_Callback();
//  }
//}


/**
  * @brief  微秒级延时
  * @param  xus 延时时长，范围：0~233015
  * @retval 无
  */
void Delay_us(uint32_t xus)
{
  SysTick->LOAD = 72 * xus;				//设置定时器重装值
  SysTick->VAL = 0x00;					//清空当前计数值
  SysTick->CTRL = 0x00000005;				//设置时钟源为HCLK，启动定时器
  while(!(SysTick->CTRL & 0x00010000));	//等待计数到0
  SysTick->CTRL = 0x00000004;				//关闭定时器
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
  while(xms--)
  {
    Delay_us(1000);
  }
}

/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
  while(xs--)
  {
    Delay_ms(1000);
  }
}
