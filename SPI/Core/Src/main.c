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
#define CS_LOW()   (GPIOA->BSRR = (1 << 4) << 16) // PA4 = 0
#define CS_HIGH()  (GPIOA->BSRR = (1 << 4))       // PA4 = 1
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
void SPI1_Init(void) {
    // 1️⃣ Enable Clocks for SPI1 and GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;  // Enable SPI1 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable GPIOA clock

    // 2️⃣ Configure PA5 (SCK), PA6 (MISO), PA7 (MOSI) as Alternate Function
    GPIOA->MODER &= ~((3 << (5 * 2)) | (3 << (6 * 2)) | (3 << (7 * 2))); // Clear mode bits
    GPIOA->MODER |=  ((2 << (5 * 2)) | (2 << (6 * 2)) | (2 << (7 * 2))); // Set to AF mode

    // 3️⃣ Set PA4 (CS) as Output
    GPIOA->MODER &= ~(3 << (4 * 2)); // Clear bits
    GPIOA->MODER |=  (1 << (4 * 2)); // Set as output

    // 4️⃣ Select AF5 (SPI1) for SCK, MISO, MOSI
    GPIOA->AFR[0] |= (5 << (5 * 4)) | (5 << (6 * 4)) | (5 << (7 * 4));

    // 5️⃣ Configure SPI1 as Master, Mode 0 (CPOL=0, CPHA=0)
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_1; // Master mode, Baud rate = fPCLK/8
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;  // Software NSS
    SPI1->CR1 |= SPI_CR1_SPE;                // Enable SPI1
}
uint8_t SPI1_TransmitReceive(uint8_t data) {
    while (!(SPI1->SR & SPI_SR_TXE));  // Wait until TX buffer is empty
    SPI1->DR = data;                   // Send data
    while (!(SPI1->SR & SPI_SR_RXNE)); // Wait until RX buffer is full
    return SPI1->DR;                    // Return received data
}
void SPI1_Write(uint8_t reg, uint8_t value) {
    CS_LOW();
    SPI1_TransmitReceive(reg & 0x7F);  // Address + Write command
    SPI1_TransmitReceive(value);       // Write data
    CS_HIGH();
}

uint8_t SPI1_Read(uint8_t reg) {
    CS_LOW();
    SPI1_TransmitReceive(reg | 0x80); // Address + Read command
    uint8_t value = SPI1_TransmitReceive(0x00); // Read data
    CS_HIGH();
    return value;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	 SPI1_Init();

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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  SPI1_Write(0x01, 0x0F); // Example: Write a register
	  HAL_Delay(1000);
  }
  return 0;
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
