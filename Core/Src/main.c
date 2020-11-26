/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "led_fade.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFF_SZ		40		// Size of command buffer
#define MSG_BUFF_SZ  150	// buffer to send from main
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile uint16_t dma_occupied = 0;				// remaining space in DMA buffer

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void proccesDmaData(uint8_t sign);		// callback to process one character of data

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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // init USART
  USART2_RegisterCallback(proccesDmaData);
  DMA_init_for_USART2();

  // init global led fade state
  fade_init((LedFadeState*)&led_fade_state);

  //init timer
  LL_TIM_ClearFlag_UPDATE(TIM2);	//set by init function
  set_duty_cycle(0);				//initial PWM value
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);	//enable PWM channels
  LL_TIM_EnableAllOutputs(TIM2);		//enable outputs
  LL_TIM_EnableIT_UPDATE(TIM2);			//enable update interrupt
  LL_TIM_EnableCounter(TIM2);			//enable timer
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t buff[MSG_BUFF_SZ];
  memset(buff, 0, MSG_BUFF_SZ);		// create buffer and set all to 0


  while (1)
  {

	LL_mDelay(200);

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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_Init1msTick(8000000);
  LL_SetSystemCoreClock(8000000);
}

/* USER CODE BEGIN 4 */

void processCmd(uint8_t *buff, uint16_t len) {
	if(strcmp(buff, "manual") == 0) {
		fade_manual((LedFadeState*)&led_fade_state);
	}
	else if(strncmp(buff, "PWM", 3) == 0) {	// compare only first 3 characters
		int t = atol(buff + 3);	// get number from command
		fade_set_target((LedFadeState*)&led_fade_state, (uint8_t)t);
	}
	else if(strcmp(buff, "auto") == 0) {
		fade_auto((LedFadeState*)&led_fade_state);
	}
}

// ignore or keep processing
typedef enum STATES_ {
	RECEIVING,
	IGNORING
}STATES;

void proccesDmaData(uint8_t sign)
{
	/* Process received data */
	static uint8_t buffer[BUFF_SZ];		//receive buffer
	static uint16_t ind = 0;			//current position
	static STATES state_machine = IGNORING;

	// check for overflow
	if(ind >= BUFF_SZ) {
		state_machine = IGNORING;
		ind = 0;
	}

	// start and ending '$' is not saved to buffer
	if(sign == '$' && state_machine != RECEIVING) {
		// upon receive of '$', start saving the characters
		state_machine = RECEIVING;
		memset(buffer, 0, BUFF_SZ);
		ind = 0;
	}
	else if((sign == '$' && state_machine == RECEIVING)) {
		// terminating '$', ignore rest of chars, process the commands
		state_machine = IGNORING;
		processCmd(buffer, ind);
	}
	else if(state_machine == RECEIVING) {
		// if in state RECEIVING, save to buffer, note: also saves begin/end characters
		buffer[ind++] = sign;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
