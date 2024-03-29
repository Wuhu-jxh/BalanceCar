/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <malloc.h>
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Motor.h"
#include "OLED.h"
#include "MPU6050.h"
#include "Serial.h"
#include "settings.h"
#include "PidContorl.h"
#include "fliter.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
    STATE_BLANCE,
    STATE_FLLOW,
    STATE_PICKUP
}State;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#if CORE_PID_FILTER_MODE == 1  || CORE_PID_FILTER_MODE == 3
Lag lag; //For test
Lag MPU_accX;
Lag MPU_accY;
Lag MPU_accZ;
Lag MPU_gyroX;
Lag MPU_gyroY;
Lag MPU_gyroZ;

#elif CORE_PID_FILTER_MODE == 2
Kalman kalman; //For test
KalmanMPU_accX;
Kalman MPU_accY;
Kalman MPU_accZ;
Kalman MPU_gyroX;
Kalman MPU_gyroY;
Kalman MPU_gyroZ;

#elif CORE_PID_FILTER_MODE == 4
RoundFliter roundFliter;
RoundFliter MPU_accX;
RoundFliter MPU_accY;
RoundFliter MPU_accZ;
RoundFliter MPU_gyroX;
RoundFliter MPU_gyroY;
RoundFliter MPU_gyroZ;

#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/**�������**/
extern _Motor _motor; //����ṹ��
_MPU6050_DATA _mpu_filtered;
extern struct inv_sensor_cal_t sensors;
/**״̬��**/
State globalState;
//全局控制量
float targetSpeed=0,targetAngle=0;
int isAlarm=0;

/**����ͨѶ��ʱ������**/
char serialBuffer[256];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#define BYTE0(dwTemp) (*(char *)(&dwTemp))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))
void get_ms_user(unsigned long *count);
_Bool uartDataProcess(char * rawData);
#define get_tick_count get_ms_user
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
_Motor Motor;
volatile uint32_t hal_timestamp = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  Motor_Init();
  OLED_Init();
  USART1_Init();
  MPU6050_DMP_init();
//  MPU6050_Init(I2C_Serch())

//  OLED_ShowString(0,0,"Temp:",16);//���Գ���
//  OLED_ShowString(0,2,"AglX:",16);
//  OLED_ShowString(0,4,"AglY:",16);
//  OLED_ShowString(0,6,"AglZ:",16);
///��PID���������ã������µķ���
//    /**PID**/
//    pid_init(&velocity,SPEED_PID_KP,SPEED_PID_KI,0);
//    pid_init(&vertical,POSITION_PID_KP,0,POSITION_PID_KD);
//    pid_init(&turn,ANGLE_PID_KP,0,0);

    /**�˲���ʼ��**/
#if CORE_PID_FILTER_MODE == 1  || CORE_PID_FILTER_MODE == 3
    lag_fliter_init(&lag,CORE_PID_FILTER_LAG);
#elif CORE_PID_FILTER_MODE == 2
    kalman_fliter_init(&kalman,CORE_PID_FILTER_KALMAN_Q,CORE_PID_FILTER_KALMAN_R,CORE_PID_FILTER_KALMAN_K);
#elif CORE_PID_FILTER_MODE == 4
    round_fliter_init(&roundFliter,CORE_PID_FILTER_BUFFER_SIZE);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
      MPU6050_Read_Accel();
      MPU6050_Read_Gyro();
      GetSpeed(&Motor);
      if(GetRxFlag())
      {
          uartDataProcess(RxDataStr);
      }
      /****���ݶ�ȡ�������****/
//��������Ҫ�˲�������Ϊ: �Ƕȣ����ٶȡ�
      /****�˲�����****/
      MPU6050_filter(&MPU6050_Data, &_mpu_filtered);
///��Ҫ��������ݲ���
      /****�ǶȻ���****/
      Angle offset_angle = offsetAngleCal(_mpu_filtered.Accel_X, _mpu_filtered.Accel_Y, _mpu_filtered.Accel_Z,
                                          _mpu_filtered.Gyro_X, _mpu_filtered.Gyro_Y, _mpu_filtered.Gyro_Z);
///��Ҫȷ��-��е��ֵ
///������PID����Ҫ�޸�ĳһ�����ƫ�����
      /**״̬�ж�**/
      if (globalState == STATE_BLANCE)
      {
          targetSpeed=0;
          targetAngle=0;
      }
      //
      /***PID��������***/
      result pidOut = PID_Cycal(_mpu_filtered,Motor.M1_ActualSpeed,Motor.M2_ActualSpeed,targetSpeed,targetAngle,offset_angle);
///��Ҫȷ��-��е��ֵ��offset
      W1_Control(pidOut.Left);
      W2_Control(pidOut.Right);
//      Myprintf("Left:%f\tRight:%f\r\n",pidOut.Left,pidOut.Right);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim==(&Encoder_TimeCounter))
  {//ÿ100ms��һ���ж�
      Encode_CallBack(&Motor);
  }
}


_Bool uartDataProcess(char * rawData)
{
    //例如：2个连续的float，2个int,长度为4，小端序
    char template[4];
    for (int i = 0; i < 4; ++i) {
        template[i] = rawData[i];
    }
    //假设第一个数字是目标速度
    targetSpeed = *(float *) template;
    for (int i = 0; i < 4; ++i) {
        template[i] = rawData[i + 4];
    }
    //假设第二个数字是目标角度
    targetAngle = *(float *) template;
    for (int i = 0; i < 4; ++i) {
        template[i] = rawData[i + 8];
    }
    //假设第三个数字是是否alarm
    isAlarm = *(int *) template;
    //假设第四个数字是校验码，固定为1234，在数据结束时必须发送一个这个
    for (int i = 0; i < 4; ++i) {
        template[i] = rawData[i + 12];
    }
    if (*(int *)template == 1234)
    {
        memset(RxDataStr,0,20);
        return 0;
    } else
        return 1;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    OLED_ShowString(0,40,"Assert Error",16);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
