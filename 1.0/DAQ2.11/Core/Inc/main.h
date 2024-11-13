/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HC_STATE_Pin GPIO_PIN_0
#define HC_STATE_GPIO_Port GPIOA
#define HC_KEY_Pin GPIO_PIN_1
#define HC_KEY_GPIO_Port GPIOA
#define SPI1_CS1_Pin GPIO_PIN_4
#define SPI1_CS1_GPIO_Port GPIOC
#define SPI1_CS2_Pin GPIO_PIN_5
#define SPI1_CS2_GPIO_Port GPIOC
#define SPI1_CS3_Pin GPIO_PIN_0
#define SPI1_CS3_GPIO_Port GPIOB
#define SPI1_CS4_Pin GPIO_PIN_1
#define SPI1_CS4_GPIO_Port GPIOB
#define SPI1_CS5_Pin GPIO_PIN_2
#define SPI1_CS5_GPIO_Port GPIOB
#define SPI2_CS1_Pin GPIO_PIN_8
#define SPI2_CS1_GPIO_Port GPIOD
#define SPI2_CS2_Pin GPIO_PIN_9
#define SPI2_CS2_GPIO_Port GPIOD
#define SPI2_CS3_Pin GPIO_PIN_10
#define SPI2_CS3_GPIO_Port GPIOD
#define SPI2_CS4_Pin GPIO_PIN_11
#define SPI2_CS4_GPIO_Port GPIOD
#define SPI2_CS5_Pin GPIO_PIN_12
#define SPI2_CS5_GPIO_Port GPIOD
#define CRX_LED_Pin GPIO_PIN_9
#define CRX_LED_GPIO_Port GPIOC
#define CTX_LED_Pin GPIO_PIN_8
#define CTX_LED_GPIO_Port GPIOA
#define ADS_DIN_Pin GPIO_PIN_10
#define ADS_DIN_GPIO_Port GPIOG
#define ADS_DOUT_Pin GPIO_PIN_11
#define ADS_DOUT_GPIO_Port GPIOG
#define ADS_SCLK_Pin GPIO_PIN_12
#define ADS_SCLK_GPIO_Port GPIOG
#define ADS_START_Pin GPIO_PIN_13
#define ADS_START_GPIO_Port GPIOG
#define ADS_RESET_Pin GPIO_PIN_14
#define ADS_RESET_GPIO_Port GPIOG
#define DRDY_Pin GPIO_PIN_15
#define DRDY_GPIO_Port GPIOG
#define SPI3_CS1_Pin GPIO_PIN_6
#define SPI3_CS1_GPIO_Port GPIOB
#define SPI3_CS2_Pin GPIO_PIN_7
#define SPI3_CS2_GPIO_Port GPIOB
#define SPI3_CS3_Pin GPIO_PIN_8
#define SPI3_CS3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
