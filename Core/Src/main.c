/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "iwdg.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bujing.h"
#include "motor.h"
#include <math.h>
#include <stdlib.h>
#include "logic.h"
#include "sincostable.h"
#include "protect.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
volatile uint16_t adc_buf[7];		

extern float sin_table[4000];
extern float cos_table[4000];
extern MCUART_Type SerUart3;
//测试
volatile int16_t text1=0;
volatile int16_t text2=0;
volatile int16_t  Spd_Set = 0;
volatile int direction = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
 {
  /* USER CODE BEGIN 1 */
//  for (int i = 0; i < ENCODER_STEPS; i++) {
//	  float arg = i * 2 * NR * M_PI / ENCODER_STEPS;							//计算每一个角度存进去并且得到对应的sin/cos值
//	  sin_table[i] = (float)sin(arg);					//把对应的弧度值存入表中
//	  cos_table[i] = (float)cos(arg);				  //把对应的弧度值存入表中
//  }
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM16_Init();
  MX_DAC1_Init();

  /* USER CODE BEGIN 2 */
	PumpSoftwareVersion_Init(); /* 在功率输出、运行定时器和看门狗启动前完成版本CRC计算。 */
	Hardware_Init();	//硬件初始化，各种打开
	Soft_Init();			//软件初始化
	Motor_AlignInit(); //启动非阻塞校准门禁，未验证编码器响应前保持PWM关闭。
	//使能时基定时器中断
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim16);		//1ms
  HAL_TIM_Base_Start_IT(&htim6);		//10us
	//变量临时简单的初始化
	App.Spd.En_Cnt_Last = 0;
	App.Spd.Now = 0;
	App.Spd.EnTime_Cnt = 1;
  /* USER CODE END 2 */
  MX_IWDG_Init();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Motor_Ctl();								//电机状态机
		Ctl_Logic();								//逻辑
		Protect();									//保护
		UartDealResponse();					//串口解析函数
		Motor_AlignProcess();                //处理新命令后推进定位，通信、保护和喂狗不被定位等待阻塞。
		HAL_IWDG_Refresh(&hiwdg);		//喂狗
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim16) {
		App.Log.Tim_Triggered ++;
		App.Prot.Protect_1msFg =1;	//保护1ms
		App.Log.StaLedCyc ++;
		
		//错误重启时间计数
		if(App.Err > 0 && MCPara[53] == 1){
			App.Log.ErrReturnCnt++;
		}
		
		if(App.AllRun ==1){
			PI_SPD();		//速度环计算1ms一次
			App.Spd.Spd_Cal_10ms_Cnt++;
			if(App.Spd.Spd_Cal_10ms_Cnt >=10){
				App.Spd.Spd_Cal_10ms_Flag = 1;
			}
		}else{
			Spd_Init();	//停止输出
		}
	}
	if(htim == &htim6){			//10us
		if(App.AllRun ==1){								//允许计时
			App.Spd.EnTime_Cnt++;
			if(App.Spd.EnTime_Cnt>10000){	//6s没有清		
				App.Spd.EnTime_Cnt =1;
				App.Spd.Now = 0;
			}
		}else{
			App.Spd.EnTime_Cnt = 1;
			App.Spd.Now = 0;

				
				App.Log.Offset_Encode_Now = 0;
				App.Log.Offset_Encode_Set = 0;
				App.Cal.cnt = TIM2->CNT;

				App.Spd.En_Err = 0;
				App.Spd.En_Cnt = TIM2->CNT;
				App.Spd.En_Cnt_Last = TIM2->CNT;			//计算差值

			
			
				if(App.Cal.cnt > ENCODER_STEPS){
					App.Cal.cnt -= ENCODER_STEPS;
				}else if(App.Cal.cnt < 0){
					App.Cal.cnt += ENCODER_STEPS;
				}			

		}
	}
	if(htim == &htim1){			//20K---0.00005s触发一下
		PWM_InterruptService();
	}
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




void delay_us(uint32_t us)
{
    // 根据系统时钟计算循环次数
    // 公式：循环次数 = us * (时钟频率 / 每个循环的时钟周期)
    // 对于STM32L431 @80MHz，每个nop约2-4个时钟周期
    uint32_t loops = us * (8000000 / 4000000); // 经验值，需要校准
    
    for(volatile uint32_t i = 0; i < loops; i++)
    {
        __NOP(); // 空指令，消耗时钟周期
    }
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
