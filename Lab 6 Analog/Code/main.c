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
int main(void)
{
	HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
	//////////////////////////////////////////////////////Part 1
	//RCC->APB2ENR|= RCC_APB2ENR_ADC1EN;
	//RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  /* USER CODE BEGIN 1 */
	// configure red and blue pins to be in alternate function mode
	//LED PC6-PC9 General Purpose Output
	//LED 9
	//GPIOC->MODER |= (1 << 18);
	//GPIOC->MODER |= (0 << 19);
	//LED 8
	//GPIOC->MODER |= (1 << 16);
	//GPIOC->MODER |= (0 << 17);
	//LED 7
	//GPIOC->MODER |= (1 << 14);
	//GPIOC->MODER |= (0 << 15);
	//LED 6
	//GPIOC->MODER |= (1 << 12);
	//GPIOC->MODER |= (0 << 13);
	// Configure PC1 as an analog input
	//GPIOC->MODER |= GPIO_MODER_MODER1; // Set PC1 to analog mode

	// Calibrate and enable ADC
	//ADC1->CR |= ADC_CR_ADCAL; // Start ADC calibration
	//while (ADC1->CR & ADC_CR_ADCAL); // Wait for calibration to finish

	// Set ADC to 8-bit resolution, continuous conversion mode, software trigger
	//ADC1->CFGR1 |= ADC_CFGR1_RES_1; // Set to 8-bit resolution
	//ADC1->CFGR1 |= ADC_CFGR1_CONT;  // Set to continuous conversion mode
	//ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN; // Ensure hardware trigger is disabled

	// Select ADC channel 11 corresponding to PC1
	//ADC1->CHSELR = ADC_CHSELR_CHSEL11; 

	// Enable ADC
	//ADC1->CR |= ADC_CR_ADEN; 
	//while (!(ADC1->ISR & ADC_ISR_ADRDY)); // Wait for ADC to be ready

	// Start ADC conversion
	//ADC1->CR |= ADC_CR_ADSTART;
	// USER CODE END 1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //while (1)
  //{
    /* USER CODE END WHILE */
	// Control LEDs based on ADC value thresholds
	//		if (ADC1->ISR & ADC_ISR_EOC) { // Check if conversion is complete
  //      uint8_t ADCValue = ADC1->DR; // Read ADC value (8-bit resolution assumed)

        // Control LEDs based on ADC value thresholds
  //      if (ADCValue < 32) {
  //          GPIOC->BSRR = (1 << 6); // Turn off all LEDs except PC6
  //          GPIOC->BSRR = (1 << 7) << 16;
  //          GPIOC->BSRR = (1 << 8) << 16;
  //          GPIOC->BSRR = (1 << 9) << 16;
  //      } else if (ADCValue < 64) {
  //         GPIOC->BSRR = (1 << 7); // Turn off all LEDs except PC7
  //         GPIOC->BSRR = (1 << 6) << 16;
  //          GPIOC->BSRR = (1 << 8) << 16;
  //          GPIOC->BSRR = (1 << 9) << 16;
  //      } else if (ADCValue < 96){
	//					GPIOC->BSRR = (1 << 8); // Turn off all LEDs except PC8
  //          GPIOC->BSRR = (1 << 6) << 16;
  //          GPIOC->BSRR = (1 << 7) << 16;
  //          GPIOC->BSRR = (1 << 9) << 16;
	//			} else if (ADCValue < 130){
	//					GPIOC->BSRR = (1 << 9); // Turn off all LEDs except PC9
  //          GPIOC->BSRR = (1 << 6) << 16;
  //          GPIOC->BSRR = (1 << 8) << 16;
  //          GPIOC->BSRR = (1 << 7) << 16;
	//			}
	/////////////////////////////////////////////////////////////////////////////Part 2
  // Select PA4 as DAC output
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  GPIOA->MODER |= GPIO_MODER_MODER4;

  // Set PA4 to DAC output mode
  GPIOA->MODER |= (1<<8);
  GPIOA->MODER |= (1<<9);

  GPIOA->PUPDR  &= ~(1<<8);
  GPIOA->PUPDR &= ~(1<<9);

  // Enable DAC channel 1
  RCC->APB1ENR |= RCC_APB1ENR_DACEN;
  // DAC->CR |= DAC_CR_EN1;
  DAC->CR |= (1<<0);

  // Set DAC channel 1 to software trigger mode
  // DAC->CR |= DAC_CR_TEN1;
  // DAC->CR &= ~DAC_CR_TSEL1;
  DAC->CR |= (1<<3);
  DAC->CR |= (1<<4);
  DAC->CR |= (1<<5);

  // Sine Wave: 8-bit, 32 samples/cycle
  const uint8_t sine_wave[32] = {127,151,175,197,216,232,244,251,254,251,244,
  232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};

  uint8_t index = 0;

  while (1) {
    // Write the next value from the wave-table to DAC channel 1 data register
    DAC->DHR8R1 = sine_wave[index];

    // Increment index 
    index = (index + 1);
    if(index == 32)
    {
      index = 0;
    }

    // 1ms delay
    HAL_Delay(1);
    }
    /* USER CODE BEGIN 3 */
  }

void part_one(void){

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
