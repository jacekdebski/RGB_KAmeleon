/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PWM_FREQUENCY 50
#define MAX_BRIGHTNESS 100 //in percentage
#define MIN_BRIGHTNESS 0 //in percentage
#define STEP_IN_SET_BRIGHTNESS 10
#define MAX_COLOR_LED 7
#define MIN_COLOR_LED 0
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
	static uint16_t counter = 0;
	static uint8_t color_led = MAX_COLOR_LED;
	static uint8_t brightness_led = MIN_BRIGHTNESS;
	static bool is_button_pressed = false;

	float pwm_period = (float)1/PWM_FREQUENCY;
	uint16_t max_counter = (uint16_t)(pwm_period/0.001);
	uint16_t led_on = (uint16_t)(brightness_led * ((float)max_counter / 100));
  /* USER CODE END SysTick_IRQn 0 */
	HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_SET
		&& HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) == GPIO_PIN_SET
		&& HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_SET
		&& HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_SET
		&& HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == GPIO_PIN_SET)
	{
			is_button_pressed = false;
	}

	//change led by right button
	if(is_button_pressed == false && HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET)
	{
		if(color_led == MAX_COLOR_LED){
			color_led = MIN_COLOR_LED;
		}
		else
		{
			color_led++;
		}
		is_button_pressed = true;
	}
	//change led by right button
	if(is_button_pressed == false && HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) == GPIO_PIN_RESET)
	{
		if(color_led == MIN_COLOR_LED){
			color_led = MAX_COLOR_LED;
		}
		else
		{
			color_led--;
		}
		is_button_pressed = true;
	}
	//increase brightness led by down button
	if(is_button_pressed == false && HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_RESET)
	{
		if(brightness_led < MAX_BRIGHTNESS){
			brightness_led += STEP_IN_SET_BRIGHTNESS;
		}
		is_button_pressed = true;
	}
	//decrease brightness led by up button
	if(is_button_pressed == false && HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET)
	{
		if(brightness_led > MIN_BRIGHTNESS){
			brightness_led -= STEP_IN_SET_BRIGHTNESS;
		}
		is_button_pressed = true;
	}
	//set initial value by push button
	if(is_button_pressed == false && HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == GPIO_PIN_RESET)
	{
		color_led = MAX_COLOR_LED;
		brightness_led = MAX_BRIGHTNESS;
		is_button_pressed = true;
	}

  	//PWM
	if(counter < led_on)
	{
			//led 0
			if((color_led & 1) == 1)
			{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
			}

			//led 1
			if((color_led & 2) == 2)
			{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
			}

			//led 2
			if((color_led & 4) == 4)
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
			}
	}
	else if (counter >= led_on && counter < max_counter)
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	}
	else if (counter == max_counter)
	{
		counter = 0;
	}

  	counter++;

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
