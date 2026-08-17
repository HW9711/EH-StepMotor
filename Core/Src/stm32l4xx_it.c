/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bujing.h"
#include "logic.h"
#include "protect.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
extern unsigned char MyRxbuf[10];
volatile unsigned char recv_end_flag = 0 ;
volatile unsigned char rx_len = 0;
extern MCUART_Type SerUart3;
/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim16;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */










void TIM1_BRK_TIM15_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_IRQn 0 */
  
  /* 检查刹车中断标志 */
  if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) != RESET)
  {
    if (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_BREAK) != RESET)
    {
      __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_BREAK);
      
      /* 这里添加刹车中断处理代码 */
      /* 例如：关闭电机、记录故障、点亮故障灯等 */
      Ctl_HardStop(E_Hardware);	//BREAK中断立即清目标、积分、CCR并锁存硬件故障。
      // 清除刹车状态（可选，根据需要）
				__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
    }
  }
  
  /* USER CODE END TIM1_BRK_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_BRK_IRQn 1 */

  /* USER CODE END TIM1_BRK_IRQn 1 */
}




void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
  /* USER CODE END DMA1_Channel1_IRQn 0 */
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	uint32_t error_flags = 0U;              //汇总ORE、NE和FE，错误帧统一丢弃并重建DMA。
	uint32_t remaining = 0U;                //保存IDLE时DMA尚未搬运的字节数。
	uint16_t rx_length = 0U;                //保存本次IDLE区间实际接收长度。
	uint8_t index = 0U;                     //逐字节封存固定6字节控制帧。

	if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE) != RESET){
		error_flags = 1U;
		__HAL_UART_CLEAR_OREFLAG(&huart3);    //清除溢出错误，当前活动帧不得继续解析。
	}
	if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_NE) != RESET){
		error_flags = 1U;
		__HAL_UART_CLEAR_NEFLAG(&huart3);     //清除噪声错误，当前活动帧不得继续解析。
	}
	if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_FE) != RESET){
		error_flags = 1U;
		__HAL_UART_CLEAR_FEFLAG(&huart3);     //清除帧错误，当前活动帧不得继续解析。
	}

	if(error_flags != 0U){
		SerUart3.RxErrorCnt++;                //记录现场串口错误次数，供调试器判断干扰强度。
		if((HAL_UART_AbortReceive(&huart3) != HAL_OK) ||
		   (HAL_UART_Receive_DMA(&huart3, SerUart3.R_DATA, BUFFER_SIZE) != HAL_OK)){
			Ctl_HardStop(E_COMM5);             //接收状态无法恢复时立即硬停，不能保留最后速度。
		}
		return;                               //错误已处理，禁止HAL再次消费同一错误标志。
	}

	if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET){
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);   //IDLE只表示本段数据结束，长度仍必须严格校验。
		remaining = hdma_usart3_rx.Instance->CNDTR; //在中止DMA前锁存剩余计数。
		if(HAL_UART_AbortReceive(&huart3) != HAL_OK){
			SerUart3.RxErrorCnt++;
			Ctl_HardStop(E_COMM5);             //DMA无法中止时保持功率关闭，等待复位恢复接收外设。
			return;
		}

		rx_length = (remaining <= BUFFER_SIZE) ? (uint16_t)(BUFFER_SIZE - remaining) : 0U; //防止异常CNDTR产生下溢长度。
		if(SerUart3.RxEnd != 0U){
			SerUart3.RxDropCnt++;               //主循环未取走旧帧时以最新帧替换，并记录一次积压。
		}
		SerUart3.RxLen = rx_length;           //先发布真实长度，短帧和长帧由主循环拒绝。
		if(rx_length == UART_COMMAND_FRAME_SIZE){
			for(index = 0U; index < UART_COMMAND_FRAME_SIZE; index++){
				SerUart3.RxFrame[index] = SerUart3.R_DATA[index]; //DMA停止后封存不可变6字节快照。
			}
		}
		__DMB();                              //确保帧数据和长度先于RxEnd对主循环可见。
		SerUart3.RxEnd = 1U;                  //最后置位完成标志，发布一个完整邮箱事件。

		if(HAL_UART_Receive_DMA(&huart3, SerUart3.R_DATA, BUFFER_SIZE) != HAL_OK){
			SerUart3.RxErrorCnt++;
			Ctl_HardStop(E_COMM5);             //重启DMA失败时立即硬停并要求零速重新授权。
		}
		return;                               //IDLE接收已经完成，不再交给通用HAL重复处理。
	}

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC channel1 and channel2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  HAL_DAC_IRQHandler(&hdac1);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
