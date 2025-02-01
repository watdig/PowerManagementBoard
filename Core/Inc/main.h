/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum holding_register_e
{
	MODBUS_ID,
	MB_BAUD_RATE,
	GPIO_READ,
	GPIO_WRITE,
	NUM_HOLDING_REGISTERS
}holding_register_t;

typedef enum gpio_read_e
{
	ESTOP_SENSE_POS,
	SENSE_120_POS,
	NUM_GPIO_READ_PINS,
}gpio_read_t;

typedef enum gpio_write_e
{
	RELAY_120_POS,
	RELAY_480_POS,
	NUM_GPIO_WRITE_PINS,
}gpio_write_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

// GPIO_WRITE MASKS
#define RELAY_120_MASK	(1 << RELAY_120_POS)
#define RELAY_480_MASK 	(1 << RELAY_480_POS)
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SENSE_120_Pin GPIO_PIN_15
#define SENSE_120_GPIO_Port GPIOB
#define RELAY_480_Pin GPIO_PIN_7
#define RELAY_480_GPIO_Port GPIOB
#define RELAY_120_Pin GPIO_PIN_8
#define RELAY_120_GPIO_Port GPIOB
#define ESTOP_SENSE_Pin GPIO_PIN_9
#define ESTOP_SENSE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
