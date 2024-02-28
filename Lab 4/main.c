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
int car;
int character;
int newCharacter;
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
//Transmit a single char
void singleCharacter(char c){
	
	while(!(USART3->ISR & (1 << 7))){
	
	}
	USART3->TDR = c;
}

//Transmit a whole string
void wholeString(char* c){
	int i = 0;
	char character = *(c+i);
	
	while(character != 0){	
		singleCharacter(character);
		i++;
		character = *(c+i);
	} 
}
//Part one for TA Check off
void partOne(){
		char c;
		if(USART3->ISR & (1 << 5)) {
			c = USART3->RDR;
			if(c == 'r' ) {
				GPIOC->ODR ^= (1 << 6);
			}else if(c == 'b' ) {
				GPIOC->ODR ^= (1 << 7);
			}else if(c== 'o' ) {
				GPIOC->ODR ^= (1 << 8);
			}else if(c== 'g' ) {
				GPIOC->ODR ^= (1 << 9);
			}else{
				wholeString("error");
			}
		}
}

void USART3_4_IRQHandler() {
	if(USART3->ISR &= (1 << 5)) {
		if(!newCharacter) {
			character = USART3->RDR;
			newCharacter = 1;
		} else {
			int correct = 0;
			char* errorMessage = "UNRECOGNIZED COMMAND!";
			uint16_t string = USART3->RDR;;
			if((char)character == 'r' ) {
				if((char)string == '0') {
					GPIOC->ODR &= ~(1 << 6);
					correct = 1;
					errorMessage = "Red off";
				} else if((char)string == '1') {
					GPIOC->ODR |= (1 << 6);
					correct = 1;
					errorMessage = "Red  on";
				} else if((char)string == '2') {
					GPIOC->ODR ^= (1 << 6);
					correct = 1;
					errorMessage = "Red toggled";
				}
			}
			if((char)character == 'b' ) {
				if((char)string == '0') {
					GPIOC->ODR &= ~(1 << 7);
					correct = 1;
					errorMessage = "Blue off";
				} else if((char)string == '1') {
					GPIOC->ODR |= (1 << 7);
					correct = 1;
					errorMessage = "Blue on";
				} else if((char)string == '2') {
					GPIOC->ODR ^= (1 << 7);
					correct = 1;
					errorMessage = "Blue toggled";
				}
			}
			if((char)character == 'o' ) {
				if((char)string == '0') {
					GPIOC->ODR &= ~(1 << 8);
					correct = 1;
					errorMessage = "Orange off";
				} else if((char)string == '1') {
					GPIOC->ODR |= (1 << 8);
					correct = 1;
					errorMessage = "Orange on";
				} else if((char)string == '2') {
					GPIOC->ODR ^= (1 << 8);
					correct = 1;
					errorMessage = "Orange toggled";
				}
			}
			if((char)character == 'g' ) {
				if((char)string == '0') {
					GPIOC->ODR &= ~(1 << 9);
					correct = 1;
					errorMessage = "Green off";
				} else if((char)string == '1') {
					GPIOC->ODR |= (1 << 9);
					correct = 1;
					errorMessage = "Green on";
				} else if((char)string == '2') {
					GPIOC->ODR ^= (1 << 9);
					correct = 1;
					errorMessage = "Green toggled";
				}
			}
			char text[] = {(char)character, (char)string, '\r', '\n'};
			wholeString(text);
			wholeString(errorMessage);
			wholeString("\r\n");
			wholeString("CMD?\r\n");
			newCharacter = 0;
		}
	}
}
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
	/* Configure the system clock */
  SystemClock_Config();
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
	RCC->AHBENR |= (1<<18)|(1<<19);
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
  /* MCU Configuration--------------------------------------------------------*/



  /* USER CODE BEGIN Init */
	GPIOB->MODER |= (1 << 21);
	GPIOB->MODER |= (0 << 20);
	GPIOB->MODER |= (1 << 23);
	GPIOB->MODER |= (0 << 22);
	
	GPIOC->MODER = (1 << 12) | (1 << 14) | (1 << 16) | (1 << 18);
  /* USER CODE END Init */

	GPIOB->AFR[1] = (4 << 8) | (4 << 12);
	
	RCC->CFGR3 |= (0 <<19);
	RCC->CFGR3 |= (1 <<18);
	
	USART3->BRR |= HAL_RCC_GetHCLKFreq()/(115200);
	USART3->CR1 = USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE;
	
  
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 0);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
		//partOne();
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
