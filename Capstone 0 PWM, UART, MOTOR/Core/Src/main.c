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
static uint8_t adc_value;
static char receivedUart;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void ADC1_Init(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_ADC1_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	ADC->CCR |= 1 << 16; // PCLK2 divide by 4
	ADC1->CR2 = 0;
	ADC1->CR1 = (1 << 8); //enable scan mode for multiple channel
	ADC1->CR2 |= ADC_CR2_ADON; // A2D on
	ADC1->CR2 |= ADC_CR2_CONT; // continuous mode
	ADC1->SQR1 = (0 << 20); // one conversions (L = N-1, so 1-1 = 0)
	ADC1->SQR3 = (0 << 0); // First conversion: Channel 0
	ADC1->SMPR2 = (1 << (0 * 3)); // Sampling time for both channels
	ADC1->CR2 |= (1 << 10);    			   // EOC after each conversion
}
uint16_t ADC_Read(void)
{
	ADC1->CR2 |= ADC_CR2_SWSTART;
	while(!(ADC1->SR & ADC_SR_EOC));
	return (uint16_t)ADC1->DR;
}


void UART_TxRxInitialise(void)
{
    // 1. Enable USART1 clock
    RCC->APB2ENR |= (1 << 4);
    // 2. Enable GPIOA clock
    RCC->AHB1ENR |= (1 << 0);
    // 3. Set PA9 (TX) and PA10 (RX) as Alternate Function (AF7 for USART1)
    GPIOA->MODER &= ~((3 << (9 * 2)) | (3 << (10 * 2)));  // Clear mode bits
    GPIOA->MODER |=  ((2 << (9 * 2)) | (2 << (10 * 2)));  // Set AF mode
    // 4. Set PA9 and PA10 to AF7 (USART1)
    GPIOA->AFR[1] &= ~((0xF << (2 * 4)) | (0xF << (1 * 4)));  // Clear AF bits
    GPIOA->AFR[1] |=  ((7 << (2 * 4)) | (7 << (1 * 4)));  // Set 7(USART1) for A9 & A10 pins
    // 5. Enable USART1, TX, and RX
    USART1->CR1 = (1 << 13) | (1 << 3) | (1 << 2);  // Enable USART, TX, RX
    // 6. Set baud rate (9600 baud, assuming 16MHz clock)
    USART1->BRR = 0x682;  // Baud rate 9600, clk=16MHz
    USART1->CR1 |= (1 << 5); // Enable RXNEIE (RX interrupt)
    // Enable USART2 interrupt in NVIC
        NVIC_EnableIRQ(USART1_IRQn);
}
// Function to send a character
void UART_SendChar(char ch) {
    while (!(USART1->SR & (1 << 7)))  // Wait for TXE (bit 7)
        ;
    USART1->DR = ch;
}
void USART1_IRQHandler(void) {
    if (USART1->SR & (1 << 5)) {  // Check if RXNE (Receive Not Empty) is set
        receivedUart = USART1->DR;  // Read data from the data register
    }
}

void Timer1_PWM_Init(void) {
    // 1. Enable TIM1 clock
    RCC->APB2ENR |= (1 << 0);

    // 2. Enable GPIOA clock
    RCC->AHB1ENR |= (1 << 0);

    // 3. Set PA8 as Alternate Function (AF1 for TIM1_CH1)
    GPIOA->MODER &= ~(3 << (8 * 2));
    GPIOA->MODER |=  (2 << (8 * 2));  // AF mode

    GPIOA->AFR[1] &= ~(0xF << (0 * 4));  // Clear AF setting (PA8 is in AFR[1] at position 0)
    GPIOA->AFR[1] |=  (1 << (0 * 4));  // AF1 for TIM1_CH1

    // 4. Configure TIM1 for PWM
//    TIM1->PSC = 160 - 1;    // Prescaler: 16 MHz / 160 = 100 kHz
    TIM1->PSC = 160 - 1;
//    TIM1->ARR = 100 - 1;    // Auto-reload: 100 kHz / 100 = 1 kHz (PWM frequency)
    TIM1->ARR = 2000 - 1;
//    TIM1->CCR1 = 50;        // Duty cycle = 50% (Can be changed)
    TIM1->CCR1 = 100;

    TIM1->CCMR1 &= ~(7 << 4);  // Clear Output Compare mode bits
    TIM1->CCMR1 |= (6 << 4);   // PWM mode 1 (active until match, then low)

    TIM1->CCER |= (1 << 0);  // Enable capture/compare output for CH1
    TIM1->BDTR |= (1 << 15); // Enable main output (MOE)

    // 5. Start TIM1
    TIM1->CR1 |= TIM_CR1_CEN;
}

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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  ADC1_Init();
  Timer1_PWM_Init();
  UART_TxRxInitialise();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		// Start ADC
		ADC1->SR = 0;
		ADC1->CR2 |= ADC_CR2_SWSTART;
		// Wait until the conversion is complete
		while (!(ADC1->SR & ADC_SR_EOC))
			;
		// Read ADC results from DR register
		adc_value = (ADC1->DR)>>4;  // Read PA0 (First conversion)
		HAL_Delay(10);

		UART_SendChar(adc_value);

		TIM1->CCR1 = receivedUart;//receivedUart;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
