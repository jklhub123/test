/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
extern uint16_t pressure[48];
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
	typedef struct {
	GPIO_TypeDef *      GPIOx;
	uint16_t            Pin;
	SPI_HandleTypeDef * hspi;
	uint16_t            Mechanical_Angel[4];
} spi_channel;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI1_CS1_Pin GPIO_PIN_4
#define SPI1_CS1_GPIO_Port GPIOC
#define SPI1_CS2_Pin GPIO_PIN_5
#define SPI1_CS2_GPIO_Port GPIOC
#define LED0_Pin GPIO_PIN_11
#define LED0_GPIO_Port GPIOF
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOF
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOF
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOF
#define SPI2_CS1_Pin GPIO_PIN_12
#define SPI2_CS1_GPIO_Port GPIOE
#define SPI2_CS2_Pin GPIO_PIN_13
#define SPI2_CS2_GPIO_Port GPIOE
#define SPI2_CS3_Pin GPIO_PIN_14
#define SPI2_CS3_GPIO_Port GPIOE
#define SPI2_CS4_Pin GPIO_PIN_15
#define SPI2_CS4_GPIO_Port GPIOE
#define SPI3_CS1_Pin GPIO_PIN_0
#define SPI3_CS1_GPIO_Port GPIOD
#define SPI3_CS2_Pin GPIO_PIN_1
#define SPI3_CS2_GPIO_Port GPIOD
#define HC_STATE_Pin GPIO_PIN_7
#define HC_STATE_GPIO_Port GPIOD
#define HC_EN_Pin GPIO_PIN_9
#define HC_EN_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */
  typedef struct
  {
    TIM_HandleTypeDef *htim;
    uint32_t tim_channel;
    uint16_t in_value;
    // ADC_HandleTypeDef *hadc;
    // uint32_t adc_channel;
    // uint16_t adc_value;
  } ctrl_channel;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
