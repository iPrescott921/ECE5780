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

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void TIM2_IRQHandler(void){
		// toggle green and orange LEDs when the interrupt is triggered
	GPIOC->ODR ^= GPIO_ODR_8;
	GPIOC->ODR ^= GPIO_ODR_9;

	// clear the pending flag in SR (3.1 Q6)
	TIM2->SR &= ~0x00000001;
}

void SET_PINS(void){
	// enable the GPIOC peripheral clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	// set the general purpose output for the LEDs
	// bits = 01
	GPIOC->MODER |= GPIO_MODER_MODER8_0;
	GPIOC->MODER |= GPIO_MODER_MODER9_0;
	// configure red and blue pins to be in alternate function mode
	GPIOC->MODER |= GPIO_MODER_MODER6_1;
	GPIOC->MODER |= GPIO_MODER_MODER7_1;

	// LEDs have push-pull output type = both bits cleared
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8);
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_9);
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6);
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_7);

	// low speed = both bits cleared
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR8);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR9);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR6);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR7);

	// no pull-up/down resistors = both bits cleared
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR9);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR7);

	// set green LED (PC9) to high
	GPIOC->ODR &= ~(GPIO_ODR_8);
	GPIOC->ODR |= GPIO_ODR_9;	
	
	// select the appropriate function numbers for said pins
	GPIOC->AFR[0] |= (0x00 << GPIO_AFRL_AFRL6_Pos) | (0x00 << GPIO_AFRL_AFRL7_Pos);	
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
}
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
	RCC->APB1ENR |= 1;
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

	SET_PINS();
	
	// configure timer2 to trigger an update event at 4Hz (default processor frequency = 8MHz)
	TIM2->PSC = 7999;			
	TIM2->ARR = 250;			
	TIM2->DIER |= 0x00000001;			
	
	TIM2->CR1 |= TIM_CR1_CEN_Msk;			// enable the timer

	// find the IRQn_Type for TIM2_IRQn, enable it
	NVIC_EnableIRQ(TIM2_IRQn);
	
//----------------------------------------------------------------------------------------------------------
	// timer 3
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; 
	
	// configure timer 3 to 800Hz UEV period
	int targetClk = 800;
	int timerClk = 8000000;
	int prescl = 4;
	int autoreload = timerClk/((prescl + 1) * targetClk);
	
	TIM3->PSC = prescl;				
	TIM3->ARR = autoreload;
	// do not enable update interrupt or setup the interrupt handler
	
	// (3.2 Q3)
	// 0x6878
	TIM3->CCMR1 |= 0x0008			
							|  0x0070			
							|  0x0800			
							|  0x6000;		

	// enable the channel registers
	TIM3->CCER |= 0x01				// enable channel 1
						 |  0x10;				// enable channel 2
	
	// set channels to 10% of ARR value
	float counter = autoreload * 0.15;
	TIM3->CCR1 = counter;
	TIM3->CCR2 = counter;

	TIM3->CR1 |= TIM_CR1_CEN_Msk;
 

  /* Infinite loop */
  while (1) {}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

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

