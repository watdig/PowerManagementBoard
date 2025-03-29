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
#include "modbus.h"
#include "error_codes.h"
#include "ee.h"
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

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

uint16_t holding_register_database[NUM_HOLDING_REGISTERS] = {
    0x0007, // MODBUS_ID
    0x0003, // MB_BAUD_RATE
	  1000, // MB_TRANSMIT_TIMEOUT
	     2, // MB_TRANSMIT_RETRIES
	0x0000, // MB_ERRORS
    0x0000,	// GPIO_READ
	0x0000,	// GPIO_WRITE
	0x03E8	// WDG_TIME
};

uint16_t prev_gpio_write_register;
uint32_t wdg_time;
uint8_t shutdown;

uint8_t prev_gpio_state;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
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

  /* USER CODE BEGIN 1 */
	int8_t modbus_status = HAL_OK;
	uint8_t modbus_tx_len = 0;
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  EE_Init(&prev_gpio_state, sizeof(uint8_t));
  EE_Read();

  if(modbus_set_rx() != HAL_OK)
  {
	  Error_Handler();
  }
  wdg_time = 0;
  shutdown = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(HAL_GPIO_ReadPin(MANUAL_GPIO_Port, MANUAL_Pin) == GPIO_PIN_SET)
	  {
		  if(shutdown)
		  {
			  // Set all GPIO pins to previous_state
			  if((prev_gpio_state & RELAY_120_MASK) != 0)
			  {
				  HAL_GPIO_WritePin(RELAY_120_GPIO_Port, RELAY_120_Pin, GPIO_PIN_SET);
			  }
			  HAL_Delay(1000);
			  if((prev_gpio_state & RELAY_480_MASK) != 0)
			  {
				  HAL_GPIO_WritePin(RELAY_480_GPIO_Port, RELAY_480_Pin, GPIO_PIN_SET);
			  }
			  wdg_time = HAL_GetTick();

			  // Carry the pin changes to the register database
			  holding_register_database[GPIO_WRITE] = prev_gpio_state;

			  // Restart the Modbus
			  modbus_status = modbus_startup();
			  if(modbus_status != 0)
			  {
				  holding_register_database[MB_ERRORS] |= 1U << ((modbus_status) + (MB_FATAL_ERROR - RANGE_ERROR));
			  }

			  // Ensure this code only executes once
			  shutdown = 0;
		  }
		  // Update the GPIO_READ register
		  GPIO_PinState estop_sense = HAL_GPIO_ReadPin(ESTOP_SENSE_GPIO_Port, ESTOP_SENSE_Pin);
		  GPIO_PinState sense_120 = HAL_GPIO_ReadPin(SENSE_120_GPIO_Port, SENSE_120_Pin);

		  holding_register_database[GPIO_READ] = ((estop_sense << ESTOP_SENSE_POS) | (sense_120 << SENSE_120_POS));

		  // Handle adjustment of the GPIO_WRITE pins
		  if(prev_gpio_state != holding_register_database[GPIO_WRITE])
		  {
			  if((prev_gpio_state & RELAY_120_MASK) != (holding_register_database[GPIO_WRITE] & RELAY_120_MASK))
			  {
				  HAL_GPIO_WritePin(RELAY_120_GPIO_Port, RELAY_120_Pin, (holding_register_database[GPIO_WRITE] & RELAY_120_MASK));
			  }
			  if((prev_gpio_state & RELAY_480_MASK) != (holding_register_database[GPIO_WRITE] & RELAY_480_MASK))
			  {
				  HAL_GPIO_WritePin(RELAY_480_GPIO_Port, RELAY_480_Pin, (holding_register_database[GPIO_WRITE] & RELAY_480_MASK));
			  }
			  prev_gpio_state = holding_register_database[GPIO_WRITE];
			  EE_Write();
			  wdg_time = HAL_GetTick();
		  }

		  // Handle Watchdog Timeout
		  if(HAL_GetTick() - wdg_time > (uint32_t)(holding_register_database[WDG_TIMEOUT]))
		  {
			  // Turn off the TBM
			  HAL_GPIO_WritePin(RELAY_480_GPIO_Port, RELAY_480_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(RELAY_120_GPIO_Port, RELAY_120_Pin, GPIO_PIN_RESET);

			  // Update the holding register database
			  holding_register_database[GPIO_WRITE] = 0;
			  prev_gpio_state = 0;
			  EE_Write();
		  }

		  // Handle Modbus Communication
		  if(modbus_rx())
		  {
			  if(get_rx_buffer(0) == holding_register_database[MODBUS_ID]) // Check Slave ID
			  {
				  wdg_time = HAL_GetTick();
				  switch(get_rx_buffer(1))
				  {
					  case 0x03:
					  {
						  // Return holding registers
						  modbus_status = return_holding_registers(&modbus_tx_len);
						  break;
					  }
					  case 0x10:
					  {
						  // Write holding registers
						  modbus_status = edit_multiple_registers(&modbus_tx_len);
						  break;
					  }
					  default:
					  {
						  modbus_status = modbus_exception(MB_ILLEGAL_FUNCTION);
						  break;
					  }
				  }
				  if(modbus_status != 0)
				  {
					  holding_register_database[MB_ERRORS] |= 1U << ((modbus_status) + (MB_FATAL_ERROR - RANGE_ERROR));
				  }
			  }
			  // Special case where you retrieve the modbus ID
			  else if((get_rx_buffer(0) == 0xFF) && // modbus_id = 0xFF = 255
				(get_rx_buffer(1) == 0x03) && // Function code = read_holding_registers
				(((get_rx_buffer(2) << 8) | get_rx_buffer(3)) == 0x00) && // Address to read = 0
				(((get_rx_buffer(4) << 8) | get_rx_buffer(5)) == 1)) // # of registers to read = 1
			  {
				  modbus_status = return_holding_registers(&modbus_tx_len);
				  if(modbus_status != 0)
				  {
					  holding_register_database[MB_ERRORS] |= 1U << ((modbus_status) + (MB_FATAL_ERROR - RANGE_ERROR));
				  }
			  }
		  }
		  modbus_status = monitor_modbus();
		  if(modbus_status != HAL_OK && modbus_status != HAL_BUSY)
		  {
			  switch(modbus_status)
			  {
				  case MB_TX_TIMEOUT:
				  {
					  for(uint8_t i = 0; i < holding_register_database[MB_TRANSMIT_RETRIES]; i++)
					  {
						  modbus_status = modbus_send(modbus_tx_len);
						  while(monitor_modbus() == HAL_BUSY);
						  if(modbus_status != HAL_OK)
						  {
							  holding_register_database[MB_ERRORS] |= 1U << ((modbus_status) + (MB_FATAL_ERROR - RANGE_ERROR));
						  }
					  }
					  break;
				  }
				  case MB_RX_TIMEOUT:
				  {
					  // Error only relates to Modbus Master Nodes
					  break;
				  }
				  case MB_UART_ERROR:
				  {
					  if(modbus_status != 0)
					  {
						  holding_register_database[MB_ERRORS] |= 1U << ((modbus_status) + (MB_FATAL_ERROR - RANGE_ERROR));
					  }
					  break;
				  }
				  case MB_FATAL_ERROR:
				  {
					  while(modbus_status != HAL_OK)
					  {
						  modbus_status = modbus_reset();
					  }
					  if(modbus_status != 0)
					  {
						  holding_register_database[MB_ERRORS] |= 1U << ((modbus_status) + (MB_FATAL_ERROR - RANGE_ERROR));
					  }
					  break;
				  }
				  default:
				  {
					  // Unknown error
				  }
			  }
		  }
	  }
	  else
	  {
		  if(!shutdown)
		  {
			  // Shutdown the Modbus
			  int8_t status = modbus_shutdown();
			  if(status != 0)
			  {
				  // log error in a queue
			  }

			  // Set all GPIO pins high
			  HAL_GPIO_WritePin(RELAY_120_GPIO_Port, RELAY_120_Pin, GPIO_PIN_SET);
			  HAL_Delay(1000);
			  HAL_GPIO_WritePin(RELAY_480_GPIO_Port, RELAY_480_Pin, GPIO_PIN_SET);

			  // Ensure this code only executes once
			  shutdown = 1;
		  }
	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RELAY_480_Pin|RELAY_120_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SENSE_120_Pin ESTOP_SENSE_Pin */
  GPIO_InitStruct.Pin = SENSE_120_Pin|ESTOP_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MANUAL_Pin */
  GPIO_InitStruct.Pin = MANUAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MANUAL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY_480_Pin RELAY_120_Pin */
  GPIO_InitStruct.Pin = RELAY_480_Pin|RELAY_120_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
