/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMEOUT 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
int32_t master_transmit[6], master_receive[6];

int flag_nss_master;
int tim_counter;

uint8_t spi_flag_2, spi_flag_1;
;
uint32_t spi_timeout;
uint8_t spi_timeout_enable;

uint32_t spi_timeout_1, spi_timeout_2;
uint8_t spi_timeout_enable_1, spi_timeout_enable_2;

int flag_spi_ready;
int nss_cond, nss_exti;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM1_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//	if (GPIO_Pin == NSS_TRIG_1_Pin) {
//		if (HAL_GPIO_ReadPin(NSS_TRIG_1_GPIO_Port, NSS_TRIG_1_Pin)
//				== GPIO_PIN_RESET) {
//			nss_cond = 0;
//
//			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) (master_transmit),
//					(uint8_t*) (master_receive), 24);
//
//		}
//		if (HAL_GPIO_ReadPin(NSS_TRIG_1_GPIO_Port, NSS_TRIG_1_Pin)
//				== GPIO_PIN_SET) {
//			flag_spi_ready = 0;
////			HAL_SPI_Abort_IT(&hspi1);
//		}
//	}
//}
//void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
//
//	if (hspi->Instance == SPI1) {
//		if (HAL_SPI_GetError(&hspi1) != HAL_SPI_ERROR_NONE) {
////			memset(master_receive, 0, 24);
//			flag_spi_ready = 0;
//		} else {
////			memset(master_receive, 0, 24);
//			flag_spi_ready = 0;
//		}
//		nss_cond = 1;
//		HAL_GPIO_WritePin(SPI_NSS_1_GPIO_Port, SPI_NSS_1_Pin, SET);
//	}
//}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI1) {
		if (HAL_SPI_GetError(&hspi1) != HAL_SPI_ERROR_NONE) {
			memset(master_receive, 0, 24);
		} else {
	//			memset(master_receive, 0, 24);
		}
		asm("nop");
		nss_cond = 1;
//		if (HAL_SPI_GetError(&hspi1) != HAL_SPI_ERROR_NONE) {
//			memset(master_receive, 0, 24);
//			flag_spi_ready = 0;
//		} else {
////			memset(master_receive, 0, 24);
//			flag_spi_ready = 0;
//		}
//		nss_cond = 1;
//		HAL_GPIO_WritePin(SPI_NSS_1_GPIO_Port, SPI_NSS_1_Pin, SET);

	}
}
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
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_TIM1_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

	HAL_GPIO_WritePin(SPI_NSS_1_GPIO_Port, SPI_NSS_1_Pin, SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(SPI_NSS_1_GPIO_Port, SPI_NSS_1_Pin, RESET);

	flag_spi_ready = 0;

	master_transmit[0] = 0xAAAAAAAA;
	master_transmit[1] = 0xBBBBBBBB;
	master_transmit[2] = 0xCCCCCCCC;
	master_transmit[3] = 0xDDDDDDDD;
	master_transmit[4] = 0xEEEEEEEE;
	master_transmit[5] = 0xFFFFFFFF;

	HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		tim_counter = HAL_GPIO_ReadPin(SPI_NSS_1_GPIO_Port, SPI_NSS_1_Pin);
//		tim_counter = HAL_GPIO_ReadPin(NSS_TRIG_1_GPIO_Port, NSS_TRIG_1_Pin);
//		if (flag_spi_ready == 0) {
//			HAL_GPIO_WritePin(SPI_NSS_1_GPIO_Port, SPI_NSS_1_Pin, RESET);
//			flag_spi_ready = 1;
//		}
//		if (flag_spi_ready == 2) {
//			HAL_GPIO_WritePin(SPI_NSS_1_GPIO_Port, SPI_NSS_1_Pin, SET);
//		}

//		if (nss_cond == 1){
//			nss_cond = 0;
//			HAL_GPIO_WritePin(SPI_NSS_1_GPIO_Port, SPI_NSS_1_Pin, RESET);
//		}

//		HAL_GPIO_WritePin(SPI_NSS_1_GPIO_Port, SPI_NSS_1_Pin, RESET);
//
//		HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) (master_transmit),
//				(uint8_t*) (master_receive), 24);
//
//		HAL_Delay(5);
//		HAL_GPIO_WritePin(SPI_NSS_1_GPIO_Port, SPI_NSS_1_Pin, SET);
//
//		HAL_Delay(5);
		if (nss_cond == 0) {
			HAL_GPIO_WritePin(SPI_NSS_1_GPIO_Port, SPI_NSS_1_Pin, RESET);
			nss_cond = 2;

			HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) (master_transmit),
								(uint8_t*) (master_receive), 24);
		}
		if (nss_cond == 1){

			HAL_GPIO_WritePin(SPI_NSS_1_GPIO_Port, SPI_NSS_1_Pin, SET);
//			HAL_Delay(1);

			nss_cond = 0;
		}

//		if (__HAL_SPI_GET_FLAG(&hspi1,
//				SPI_FLAG_RXNE) && __HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE)) {
//			if (HAL_SPI_GetError(&hspi1) != HAL_SPI_ERROR_NONE) {
//				memset(master_receive, 0, 24);
//			} else {
//	//			memset(master_receive, 0, 24);
//			}
//			asm("nop");
//			nss_cond = 1;
//		}

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8400-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_NSS_1_GPIO_Port, SPI_NSS_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CH1_GPIO_Port, CH1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI_NSS_1_Pin */
  GPIO_InitStruct.Pin = SPI_NSS_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_NSS_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NSS_TRIG_1_Pin */
  GPIO_InitStruct.Pin = NSS_TRIG_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NSS_TRIG_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CH1_Pin */
  GPIO_InitStruct.Pin = CH1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CH1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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
	while (1) {
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
