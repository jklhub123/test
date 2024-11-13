/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
extern uint16_t Mechanical_Angel[2][4];
extern uint16_t Angel_Zero[2][4];

extern int16_t Hand_Angel[20];
extern float Hand_Angel_float[20];
extern int16_t Arm_Angel[8];
extern float Arm_Angel_float[8];
	
typedef struct {
	GPIO_TypeDef *      GPIOx;
	uint16_t            Pin;
	SPI_HandleTypeDef * hspi;
} spi_channel;
/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi1;

extern SPI_HandleTypeDef hspi2;

extern SPI_HandleTypeDef hspi3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_SPI1_Init(void);
void MX_SPI2_Init(void);
void MX_SPI3_Init(void);

/* USER CODE BEGIN Prototypes */
void Angle_Reset(void);
void Read_RTB(void);
void Read_B(void);
void Read_Arm(void);
void Read_Hand(void);
void READ_CMD(uint8_t num,uint16_t *data);
void READ_Angle(uint8_t num);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

