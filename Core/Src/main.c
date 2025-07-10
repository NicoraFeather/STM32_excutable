/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "control.h"
#include "mpu6050.h"
#include "callback.h"
#include "encoder.h"
#include "vbat.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "../Inc/wireless.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/********************灰度传感器配置********************/
char rx_buff[256]="";
unsigned short Anolog[8]={0};
unsigned short white[8]={1600,1600,1600,1600,1600,1600,1600,1600};
unsigned short black[8]={100,100,100,100,100,100,100,100};
unsigned short Normal[8];
unsigned char Digtal;

/********************extern数组********************/
extern DMA_HandleTypeDef hdma_usart1_rx;
extern PID pid_l_speed, pid_l_position, pid_r_speed, pid_r_position; //PID结构体
extern uint8_t DataBuff[128]; //串口接收数据缓存
extern uint8_t command_received[128]; //蓝牙接收数据缓存
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void uart1_send_string(char* str)
{
  while(*str!=0&&str!=0)
  {
    HAL_UART_Transmit(&huart1,*str++,1,HAL_MAX_DELAY);
  }
}
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  DWT_InitMicros(); //初始化DWT计数器，程序正式计数
  Vbat_Init();//初始化电池电压采集

  /************串口接收初始化************/
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3,command_received, 128);
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, DataBuff, 128);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);

  /**********************灰度传感器初始化**********************/
   // No_MCU_Sensor sensor;
   // sprintf((char *)rx_buff,"hello_world!\r\n");
   // HAL_UART_Transmit_DMA(&huart1, rx_buff, strlen((char *)rx_buff));
   //
   // //初始化传感器，不带黑白
   // No_MCU_Ganv_Sensor_Init_Frist(&sensor);//结构体归零
   // No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);//无校准读取数据
   // Get_Anolog_Value(&sensor,Anolog);
   // sprintf(rx_buff,"Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n",Anolog[0],Anolog[1],Anolog[2],Anolog[3],Anolog[4],Anolog[5],Anolog[6],Anolog[7]);
   // HAL_UART_Transmit_DMA(&huart1, rx_buff, strlen(rx_buff));
   // HAL_Delay(100);

   // //得到黑白校准值之后，初始化传感器
   // No_MCU_Ganv_Sensor_Init(&sensor,white,black);
   // HAL_Delay(100);
  /*******************电机控制初始化*******************/
  Motor_Init();//电机速度环初始化
  Control_Init();//电机平衡初始化
  MPU6050_Init(); //初始化MPU6050

  PID_Set_General(&pid_l_speed, 0.4f, 10.0f, 0.0f);//速度环设定值
  PID_Set_General(&pid_r_speed, 0.4f, 10.0f, 0.0f);

  //HAL_TIM_Base_Start_IT(&GAP_TIM);//10ms定时器开启
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // //无时基传感器常规任务，包含模拟量，数字量，归一化量
    // No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
    // //有时基传感器常规任务，包含模拟量，数字量，归一化量
    // //			No_Mcu_Ganv_Sensor_Task_With_tick(&sensor)
    // //获取传感器数字量结果(只有当有黑白值传入进去了之后才会有这个)
    // Digtal=Get_Digtal_For_User(&sensor);
    // sprintf(rx_buff,"Digtal %d-%d-%d-%d-%d-%d-%d-%d\r\n",(Digtal>>0)&0x01,(Digtal>>1)&0x01,(Digtal>>2)&0x01,(Digtal>>3)&0x01,(Digtal>>4)&0x01,(Digtal>>5)&0x01,(Digtal>>6)&0x01,(Digtal>>7)&0x01);
    // HAL_UART_Transmit(&huart1, rx_buff, strlen(rx_buff), HAL_MAX_DELAY);
    //
    // //获取传感器模拟量结果(有黑白初始化后返回1 没有返回 0)
    // if(Get_Anolog_Value(&sensor,Anolog)){
    //   sprintf(rx_buff,"Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n",Anolog[0],Anolog[1],Anolog[2],Anolog[3],Anolog[4],Anolog[5],Anolog[6],Anolog[7]);
    //   HAL_UART_Transmit(&huart1, rx_buff, strlen(rx_buff), HAL_MAX_DELAY);
    // }
    //
    // //获取传感器归一化结果(只有当有黑白值传入进去了之后才会有这个结果！！有黑白值初始化后返1 没有返回 0)
    // if(Get_Normalize_For_User(&sensor,Normal)){
    //   sprintf(rx_buff,"Normalize %d-%d-%d-%d-%d-%d-%d-%d\r\n",Normal[0],Normal[1],Normal[2],Normal[3],Normal[4],Normal[5],Normal[6],Normal[7]);
    //   HAL_UART_Transmit(&huart1, rx_buff, strlen(rx_buff), HAL_MAX_DELAY);
    // }
    //
    // HAL_Delay(10);//最短延时时间
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
