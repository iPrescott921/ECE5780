/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
void _Error_Handler(char * file, int line);

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void write(char val);
char read();
void stop();
int16_t readXAxis();
int16_t readYAxis();

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();               // Reset of all peripherals, init the Flash and Systick
  SystemClock_Config();     // Configure the system clock

  // Enable GPIOB and GPIOC clocks in the RCC
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

  // Set up LEDs and C5
  GPIOC->MODER |= (1 << 18) | (1 << 16) | (1 << 14) | (1 << 12) | (1 << 0);    // Set PC9-PC6 and PC0 to output mode
  GPIOC->BSRR = (1 << 0);    // Set PC0 (CS) line high, selects I2C mode on gyro

  GPIOB->MODER  |= (1 << 23) | (1 << 27) | (1 << 28);  // Set PB11 & PB13 to AF Mode, PB14 to ouput
  GPIOB->OTYPER |= (1 << 11) | (1 << 13);              // Set PB11 & PB13 to open-drain output type
  GPIOB->PUPDR  |= (1 << 22) | (1 << 26);              // Set internal pull-up resistors on PB 11 & PB13
  GPIOB->AFR[1] = 0x00501000;                          // Set AF1 on PB11(I2C2_SDA) & AF5 on PB13(I2C2_SCL)
  GPIOB->BSRR = (1 << 14);                             // Set PB14 (address select) line high

  // Enable I2C2 peripheral in the RCC
  RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

  // Configure I2C2 parameters for 100 kHz standard-mode
  I2C2->TIMINGR = (1 << 28) | (0x13 << 0) | (0xF << 8) | (0x2 << 16) | (0x4 << 20);    // From table

  // Enable the I2C2 peripheral using the PE bit in the CR1 register
  I2C2->CR1 |= I2C_CR1_PE;

  // // Set the transaction parameters in the CR2 register
  // // For write operation (sending register address)
  // I2C2->CR2 = (0x69 << 1) | (1 << 16);

  // // Set the START bit to begin the address frame
  // I2C2->CR2 |= I2C_CR2_START;

  // // Wait until either TXIS or NACKF flags are set
  // while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)))
  // {
  //   GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
  //   // Handle the error
  // }

  // // Check if NACKF flag is set (slave did not respond)
  // if (I2C2->ISR & I2C_ISR_NACKF)
  // {
  //   GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
  //   // Handle the error
  // }

  // // Write the address of the "WHO_AM_I" register into the I2C transmit register (TXDR)
  // I2C2->TXDR |= 0x0F;

  // // Wait until TC (Transfer Complete) flag is set
  // while (!(I2C2->ISR & (I2C_ISR_TC | I2C_ISR_NACKF)))
  // {
  //   GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
  //   // Handle the error
  // }
    
  // // Check if NACKF flag is set (slave did not respond)
  // if (I2C2->ISR & I2C_ISR_NACKF)
  // {
  //   GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
  //   // Handle the error
  // }

  // // Reload CR2 register with the same parameters but set RD_WRN for read operation
  // I2C2->CR2 = (0x69 << 1) | (1 << 16) | I2C_CR2_RD_WRN | I2C_CR2_START;

  // // Wait until either RXNE or NACKF flags are set
  // while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)))
  // {
  //   GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
  //   // Handle the error
  // }

  // // Check if NACKF flag is set (slave did not respond)
  // if (I2C2->ISR & I2C_ISR_NACKF)
  // {
  //   GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
  //   // Handle the error
  // }

  // // Wait until TC (Transfer Complete) flag is set
  // while (!(I2C2->ISR & (I2C_ISR_TC | I2C_ISR_NACKF)))
  // {
  //   GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
  //   // Handle the error
  // }

  // // Set the STOP bit in the CR2 register to release the I2C bus
  // I2C2->CR2 |= I2C_CR2_STOP;

  // // Check the contents of the RXDR register to see if it matches the expected value (0xD4)
  // if (I2C2->RXDR != 0xD3) // I2C2->RXDR == 0x69
  // {
  //   GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
  //   // Handle the error
  // }

  // GPIOC->BSRR |= (1 << (22)); // Clear PC6 to turn off the red LED

  // // Shows it made it through while and if statements
  // GPIOC->BSRR |= (1 << 7); // Set PC7 to turn on the blue LED

  write(0x20);

  // Write ctrlReg1Value to the CTRL_REG1 register of the gyroscope
  I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0)); // Clear SADD and NBYTES
	I2C2->CR2 &= ~(1 << 10); 
  I2C2->CR2 |= (0x69 << 1) | (2 << 16); // Addressing the gyroscope

  // Set the START bit to begin the address frame
  I2C2->CR2 |= I2C_CR2_START;

  while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))) 
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle the error
  }
  
  if (I2C2->ISR & I2C_ISR_NACKF) 
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle NACK error
  }

  I2C2->TXDR = 0x20; // Register address of CTRL_REG1

  while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))) 
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle the error
  }
  

  if (I2C2->ISR & I2C_ISR_NACKF) 
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle NACK error
  }
  
  //bit pattern to turn on Xen, Yen, and PD/Noraml mode 
  I2C2->TXDR = 0x0B; // 0x0B => 0000 1011

  while (!(I2C2->ISR & (I2C_ISR_TC | I2C_ISR_NACKF)))
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle the error
  }

  if (I2C2->ISR & I2C_ISR_NACKF) 
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle NACK error
  }

  // read CTRL_REG1 to make sure data is set correctly
	write(0x20);
	if (read() != 0x0b) {
		GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED;
	}

  // Clear all LEDs
  GPIOC->BSRR |= (1 << (6 + 16)); // Clear PC6 to turn off the red LED
  GPIOC->BSRR |= (1 << (7 + 16)); // Clear PC7 to turn off the blue LED
  GPIOC->BSRR |= (1 << (8 + 16)); // Clear PC8 to turn off the orange LED
  GPIOC->BSRR |= (1 << (9 + 16)); // Clear PC9 to turn off the green LED

  int16_t xAxis = 0;
	int16_t yAxis = 0;
	const int16_t threshold = 0x01FF;

  while (1) {
		xAxis = readXAxis();
		yAxis = readYAxis();
		
		if (xAxis > threshold) {
			GPIOC->BSRR |= (1 << 6); // Set PC6 to turn on the red LED
		}
		else {
			GPIOC->BSRR |= (1 << (6 + 16)); // Clear PC6 to turn off the red LED
		}
		
		if (yAxis < 0 - threshold) {
			GPIOC->BSRR |= (1 << 7); // Set PC7 to turn on the blue LED
		}
		else {
			GPIOC->BSRR |= (1 << (7 + 16)); // Clear PC7 to turn off the blue LED
		}
		
		if (xAxis < 0 - threshold) {
			GPIOC->BSRR |= (1 << 8); // Set PC8 to turn on the orange LED
		}
		else {
			GPIOC->BSRR |= (1 << (8 + 16)); // Clear PC8 to turn off the orange LED
		}
		
		if (yAxis > threshold) {
			GPIOC->BSRR |= (1 << 9); // Set PC9 to turn on the green LED
		}
		else {
			GPIOC->BSRR |= (1 << (9 + 16)); // Clear PC9 to turn off the green LED
		}
		
		HAL_Delay(100);
	}

}


void write(char val) {
  // Set the transaction parameters in the CR2 register
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0)); //clear SADD and NBYTES
	// Set to write
	I2C2->CR2 &= ~(1 << 10);
	I2C2->CR2 |= (0x69 << 1) | (1 << 16);
	
  // Set the START bit to begin the address frame
  I2C2->CR2 |= I2C_CR2_START;

  while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))) 
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle the error
  }
  
  if (I2C2->ISR & I2C_ISR_NACKF) 
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle NACK error
  }

  // Set register of CTRL_REG1
	I2C2->TXDR = val;
	
	while (!(I2C2->ISR & (I2C_ISR_TC | I2C_ISR_NACKF)))
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle the error
  }

  if (I2C2->ISR & I2C_ISR_NACKF) 
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle NACK error
  }

  //return 0;
}

char read() {
  // Reload CR2 register with the same parameters but set RD_WRN for read operation
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	I2C2->CR2 = (0x69 << 1) | (1 << 16) | I2C_CR2_RD_WRN;
	
  // Set the START bit to begin the address frame
  I2C2->CR2 |= I2C_CR2_START;

  // Wait until either RXNE or NACKF flags are set
  while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)))
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle the error
  }

  // Check if NACKF flag is set (slave did not respond)
  if (I2C2->ISR & I2C_ISR_NACKF)
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle the error
  }

  char val = I2C2->RXDR;

  // Wait until TC (Transfer Complete) flag is set
  while (!(I2C2->ISR & (I2C_ISR_TC | I2C_ISR_NACKF)))
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle the error
  }

  if (I2C2->ISR & I2C_ISR_NACKF) 
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle NACK error
  }

	return val;
}

void stop() {
	I2C2->CR2 |= (1 << 14);	// STOP I2C2
}

int16_t readXAxis() {
	
  int16_t xAxis = 0;
	write(0xA8);
	stop();

	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	I2C2->CR2 = (0x69 << 1) | (2 << 16) | I2C_CR2_RD_WRN;
	
  // Set the START bit to begin the address frame
  I2C2->CR2 |= I2C_CR2_START;

  // wait for first 8-bit data

  // Wait until either RXNE or NACKF flags are set
  while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)))
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle the error
  }

  // Check if NACKF flag is set (slave did not respond)
  if (I2C2->ISR & I2C_ISR_NACKF)
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle the error
  }
	
	xAxis = I2C2->RXDR;

  // Wait until either RXNE or NACKF flags are set
  while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)))
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle the error
  }

  // Check if NACKF flag is set (slave did not respond)
  if (I2C2->ISR & I2C_ISR_NACKF)
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle the error
  }

	xAxis |= (I2C2->RXDR << 8);
	
  // Wait until TC (Transfer Complete) flag is set
  while (!(I2C2->ISR & (I2C_ISR_TC | I2C_ISR_NACKF)))
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle the error
  }

  if (I2C2->ISR & I2C_ISR_NACKF) 
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle NACK error
  }

	return xAxis;
}

int16_t readYAxis() {
	
  int16_t yAxis = 0;
	write(0xAA);
	stop();

	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	I2C2->CR2 = (0x69 << 1) | (2 << 16) | I2C_CR2_RD_WRN;

	// Set the START bit to begin the address frame
  I2C2->CR2 |= I2C_CR2_START;
	
	
  // Wait until either RXNE or NACKF flags are set
  while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)))
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle the error
  }

  // Check if NACKF flag is set (slave did not respond)
  if (I2C2->ISR & I2C_ISR_NACKF)
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle the error
  }
	
	yAxis = I2C2->RXDR;

  // Wait until either RXNE or NACKF flags are set
  while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)))
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle the error
  }

  // Check if NACKF flag is set (slave did not respond)
  if (I2C2->ISR & I2C_ISR_NACKF)
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle the error
  }

	yAxis |= (I2C2->RXDR << 8);
	
  // Wait until TC (Transfer Complete) flag is set
  while (!(I2C2->ISR & (I2C_ISR_TC | I2C_ISR_NACKF)))
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle the error
  }

  if (I2C2->ISR & I2C_ISR_NACKF) 
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle NACK error
  }

	return yAxis;
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}