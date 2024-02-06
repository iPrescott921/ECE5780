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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
int main(void) {
	HAL_Init(); // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config(); //Configure the system clock
	/* This example uses HAL library calls to control
	the GPIOC peripheral. You�ll be redoing this code
	with hardware register access. */
	RCC->AHBENR |= (1 << 19);
	RCC->AHBENR |= (1 << 17);
	
	volatile int i;
	for(i=0; i < 100000; i++){}
	
	HAL_Delay(200); // Delay 200ms
		
	//LED PC6-PC9 General Purpose Output
	GPIOC->MODER |= (1 << 18);
	GPIOC->MODER |= (0 << 19);
	GPIOC->MODER |= (1 << 16);
	GPIOC->MODER |= (0 << 17);
	GPIOC->MODER |= (1 << 14);
	GPIOC->MODER |= (0 << 15);
	GPIOC->MODER |= (1 << 12);
	GPIOC->MODER |= (0 << 13);
	//LED PC6-PC9 Push-pull output
	GPIOC->OTYPER |= (0 << 8);	
	GPIOC->OTYPER |= (0 << 9);
	GPIOC->OTYPER |= (0 << 7);	
	GPIOC->OTYPER |= (0 << 6);
	//LED PC6-PC9 Low speed
	GPIOC->OSPEEDR |= (0 << 18);
	GPIOC->OSPEEDR |= (0 << 16);
	GPIOC->OSPEEDR |= (0 << 14);
	GPIOC->OSPEEDR |= (0 << 12);
	//LED PC6-PC9 No pull-up/down resistors
	GPIOC->PUPDR |= (0 << 12);
	GPIOC->PUPDR |= (0 << 13);
	GPIOC->PUPDR |= (0 << 14);
	GPIOC->PUPDR |= (0 << 15);
	GPIOC->PUPDR |= (0 << 16);
	GPIOC->PUPDR |= (0 << 17);
	GPIOC->PUPDR |= (0 << 18);
	GPIOC->PUPDR |= (0 << 19);
	GPIOC->ODR = 1 << 7;
	//USER Button Digital Input
	GPIOA->MODER |= (0 << 0);
	GPIOA->MODER |= (0 << 1);
	//USER Button Low Speed
	GPIOA->OSPEEDR |= (0 << 0);
	//USER Button Pull-down resistor
	GPIOA->PUPDR |= (1 << 1);
	GPIOA->PUPDR |= (0 << 0);
	
	GPIOA->MODER=0;
	GPIOA->OSPEEDR = 0;
	GPIOA->PUPDR = 2;
	
	// Set up a configuration struct to pass to the initialization function
	//GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9,
	//														GPIO_MODE_OUTPUT_PP,
	//														GPIO_SPEED_FREQ_LOW,
	//														GPIO_NOPULL};
	//HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC8 & PC9
	uint32_t debouncer = 0;
	while(1) {
	debouncer = (debouncer << 1); // Always shift every loop iteration
	if (GPIOA->IDR & 1) { // If input signal is set/high
		debouncer |= 0x01; // Set lowest bit of bit-vector
	}
	if (debouncer == 0xFFFFFFFF) {
	// This code triggers repeatedly when button is steady high!
	}
	if (debouncer == 0x00000000) {
	// This code triggers repeatedly when button is steady low!
	}
	if (debouncer == 0x7FFFFFFF) {
		GPIOC->ODR ^= (1 << 6) | (1 << 7);
	// This code triggers only once when transitioning to steady high!
	}
	// When button is bouncing the bit-vector value is random since bits are set when
	//the button is high and not when it bounces low.
	
	}
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
